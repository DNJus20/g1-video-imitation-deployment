#!/usr/bin/env python3
"""Banco sim2sim del binario de despliegue (G4 del lote CR).

MuJoCo (g1_29dof) <-> DDS <-> g1_ctrl --network=lo, sin mando fisico:
los botones se inyectan en LowState_.wireless_remote segun un guion.
Se ejecuta dentro de g1-mujoco:latest (mujoco + unitree_sdk2py).

Guion (s de pared desde el arranque):
  0      Passive, robot sostenido por una "banda elastica" en la pelvis (como el arnes)
  T_FIX  LT+up  -> FixStand
  T_VEL  RT+A   -> Velocity
  T_REL  se suelta la banda (rampa de 1 s)
  T_MIM  RB+<boton> -> Mimic_<baile>
  T_MIM+dur+3  RT+A -> Velocity
  fin    T_MIM+dur+7

Uso: python sim2sim_bridge.py <boton A|B|X|Y> <motion.npz> <salida.npz>
"""
import json
import struct
import sys
import time

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                         ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowStateDefault
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

SCENE = "/workspace/unitree_mujoco/unitree_robots/g1/scene_29dof.xml"
DT = 0.001                      # 1 kHz, como el robot real
T_FIX, T_VEL, T_REL, T_MIM = 5.0, 9.0, 11.0, 15.0
PRESS = 0.3                     # s que se mantiene pulsado un combo
Z_BAND = 0.80                   # altura de la pelvis que sostiene la banda

# bits de wireless_remote[2] y [3] (unitree_joystick.hpp / unitree_mujoco)
B2 = {"R1": 0, "L1": 1, "start": 2, "select": 3, "R2": 4, "L2": 5}
B3 = {"A": 0, "B": 1, "X": 2, "Y": 3, "up": 4, "right": 5, "down": 6, "left": 7}

boton, motion_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
mot = np.load(motion_path)
fps = float(np.ravel(mot["fps"])[0]) if "fps" in mot.files else 50.0
ref_q = mot["joint_pos"]
dur = ref_q.shape[0] / fps
T_BACK = T_MIM + dur + 3.0
T_END = T_BACK + 4.0
GUION = [(T_FIX, ("L2", "up")), (T_VEL, ("R2", "A")),
         (T_MIM, ("R1", boton)), (T_BACK, ("R2", "A"))]

m = mujoco.MjModel.from_xml_path(SCENE)
m.opt.timestep = DT
d = mujoco.MjData(m)
nu = m.nu
assert nu == 29, nu
qadr = [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in range(nu)]
vadr = [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in range(nu)]
d.qpos[2] = Z_BAND
mujoco.mj_forward(m, d)
pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")

ChannelFactoryInitialize(0, "lo")
pub = ChannelPublisher("rt/lowstate", LowState_)
pub.Init()
ls = LowStateDefault()
ls.mode_machine = 5
cmd = {"q": np.zeros(nu), "dq": np.zeros(nu), "kp": np.zeros(nu),
       "kd": np.zeros(nu), "tau": np.zeros(nu), "n": 0}


def on_cmd(msg):
    for i in range(nu):
        mc = msg.motor_cmd[i]
        cmd["q"][i], cmd["dq"][i], cmd["kp"][i] = mc.q, mc.dq, mc.kp
        cmd["kd"][i], cmd["tau"][i] = mc.kd, mc.tau
    cmd["n"] += 1


sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
sub.Init(on_cmd, 10)


def botones(t):
    """Modificador (LT/RT/RB) primero; la segunda tecla 0,5 s despues, como un operador.
    LT/RT son ejes suavizados: si se pulsan a la vez, el on_pressed de la tecla llega
    antes de que el eje cruce el umbral y el combo no dispara."""
    b2 = b3 = 0
    l2 = 0.0
    for t0, (mod, key) in GUION:
        if t0 <= t < t0 + 0.5 + PRESS:
            b2 |= 1 << B2[mod]
            if mod == "L2":
                l2 = 1.0
        if t0 + 0.5 <= t < t0 + 0.5 + PRESS:
            b3 |= 1 << B3[key]
    return b2, b3, l2


LOG = {k: [] for k in ("t", "q", "z", "rpy", "ncmd", "lag")}
wall0_utc = time.time()
t0w = time.perf_counter()
k = 0
while True:
    t = k * DT
    if t >= T_END:
        break
    q = np.array([d.qpos[a] for a in qadr])
    dq = np.array([d.qvel[a] for a in vadr])
    tau = cmd["tau"] + cmd["kp"] * (cmd["q"] - q) + cmd["kd"] * (cmd["dq"] - dq)
    d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])

    # banda elastica en la pelvis hasta T_REL (+1 s de rampa)
    w = 1.0 if t < T_REL else max(0.0, 1.0 - (t - T_REL))
    d.xfrc_applied[pelvis, :] = 0
    if w > 0:
        p = d.xpos[pelvis]
        d.xfrc_applied[pelvis, 0:3] = w * (np.array([-2000 * p[0], -2000 * p[1], 8000 * (Z_BAND - p[2])])
                                           - 100 * d.qvel[0:3])
        # resorte de orientacion (parte vectorial del cuaternion) + amortiguador suave
        d.xfrc_applied[pelvis, 3:6] = w * (-400 * d.qpos[4:7] - 30 * d.qvel[3:6])
    mujoco.mj_step(m, d)

    # LowState_ (hg)
    for i in range(nu):
        ms = ls.motor_state[i]
        ms.q, ms.dq, ms.tau_est = float(q[i]), float(dq[i]), float(d.ctrl[i])
    quat = d.qpos[3:7]
    ls.imu_state.quaternion = [float(x) for x in quat]
    ls.imu_state.gyroscope = [float(x) for x in d.qvel[3:6]]   # marco del cuerpo (free joint)
    rpy = np.zeros(3)
    w_, x_, y_, z_ = quat
    rpy[0] = np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
    rpy[1] = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
    rpy[2] = np.arctan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))
    ls.imu_state.rpy = [float(x) for x in rpy]
    b2, b3, l2 = botones(t)
    wr = [0] * 40
    wr[2], wr[3] = b2, b3
    wr[16:20] = list(struct.pack("<f", l2))
    ls.wireless_remote = wr
    ls.tick = k
    pub.Write(ls)

    if k % 10 == 0:
        LOG["t"].append(t)
        LOG["q"].append(q.astype(np.float32))
        LOG["z"].append(float(d.qpos[2]))
        LOG["rpy"].append(rpy.astype(np.float32))
        LOG["ncmd"].append(cmd["n"])
        LOG["lag"].append(time.perf_counter() - t0w - t)
    k += 1
    # ritmo de tiempo real
    lag = (t + DT) - (time.perf_counter() - t0w)
    if lag > 0:
        time.sleep(lag)

np.savez_compressed(out_path, **{k_: np.array(v) for k_, v in LOG.items()},
                    ref_q=ref_q, fps=fps, guion=json.dumps(GUION), t_mim=T_MIM,
                    t_back=T_BACK, wall0_utc=wall0_utc)
print("[bridge] fin", out_path, "ncmd", cmd["n"], "lag_max_s", round(max(LOG["lag"]), 3))
