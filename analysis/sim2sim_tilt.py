#!/usr/bin/env python3
"""Banco sim2sim: DAMPING POR INCLINACION del binario de despliegue (pendiente 2 del camera-ready).

Mismo puente que sim2sim_bridge.py (MuJoCo <-> DDS <-> g1_ctrl --network=lo, botones inyectados),
pero en vez de bailar: el robot queda COLGADO de la banda (pies en el aire) en Velocity y la banda
gira la pelvis con una rampa lenta (RATE rad/s) alrededor del eje pedido. Se registra el instante
en que g1_ctrl pasa a Passive (kp -> 0 en rt/lowcmd) y la inclinacion del robot en ese instante,
medida como lo hace `bad_orientation`: acos(-g_z en el cuerpo) = acos(R[2,2]).

Guion (s de pared): 0 Passive colgado | T_FIX LT+up FixStand | T_VEL RT+A Velocity | T_TILT rampa.
Uso: python sim2sim_tilt.py <eje: pitch+|pitch-|roll+|roll-> <salida.npz> [rate_rad_s] [K_ori] [D_ori]
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
DT = 0.001
T_FIX, T_VEL, T_TILT = 5.0, 9.0, 13.0
RATE = 0.10                     # rad/s de la rampa (5,7 grados/s)
T_END = T_TILT + 1.6 / RATE + 2.0   # hasta ~1,6 rad o hasta 2 s despues del disparo
PRESS = 0.3
Z_BAND = 0.95                   # pelvis alta: pies en el aire (colgado)
B2 = {"R1": 0, "L1": 1, "start": 2, "select": 3, "R2": 4, "L2": 5}
B3 = {"A": 0, "B": 1, "X": 2, "Y": 3, "up": 4, "right": 5, "down": 6, "left": 7}

eje, out_path = sys.argv[1], sys.argv[2]
RATE = float(sys.argv[3]) if len(sys.argv) > 3 else RATE
K_ORI = float(sys.argv[4]) if len(sys.argv) > 4 else 3000.0
D_ORI = float(sys.argv[5]) if len(sys.argv) > 5 else 150.0
T_END = T_TILT + 1.6 / RATE + 2.0
sign = 1.0 if eje.endswith("+") else -1.0
axis = np.array([0.0, 1.0, 0.0]) if eje.startswith("pitch") else np.array([1.0, 0.0, 0.0])
GUION = [(T_FIX, ("L2", "up")), (T_VEL, ("R2", "A"))]

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
       "kd": np.zeros(nu), "tau": np.zeros(nu), "n": 0, "t_rx": []}


def on_cmd(msg):
    for i in range(nu):
        mc = msg.motor_cmd[i]
        cmd["q"][i], cmd["dq"][i], cmd["kp"][i] = mc.q, mc.dq, mc.kp
        cmd["kd"][i], cmd["tau"][i] = mc.kd, mc.tau
    cmd["n"] += 1


sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
sub.Init(on_cmd, 10)


def botones(t):
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


def quat_from_axis_angle(ax, ang):
    s = np.sin(ang / 2)
    return np.array([np.cos(ang / 2), ax[0] * s, ax[1] * s, ax[2] * s])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


LOG = {k: [] for k in ("t", "theta_cmd", "tilt", "rpy", "kp_sum", "ncmd", "z", "lag")}
trigger = None
in_velocity_seen = False
t0w = time.perf_counter()
k = 0
while True:
    t = k * DT
    if t >= T_END or (trigger is not None and t > trigger["t"] + 2.0):
        break
    q = np.array([d.qpos[a] for a in qadr])
    dq = np.array([d.qvel[a] for a in vadr])
    tau = cmd["tau"] + cmd["kp"] * (cmd["q"] - q) + cmd["kd"] * (cmd["dq"] - dq)
    d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])

    # banda: siempre puesta (colgado). Posicion: resorte a Z_BAND. Orientacion: servo hacia la rampa.
    theta = sign * RATE * max(0.0, t - T_TILT)
    q_des = quat_from_axis_angle(axis, theta)
    q_cur = d.qpos[3:7].copy()
    q_conj = q_cur * np.array([1, -1, -1, -1])
    q_err = quat_mul(q_des, q_conj)             # rotacion que falta, en mundo
    if q_err[0] < 0:
        q_err = -q_err
    err_vec = 2.0 * q_err[1:4]                  # ~ angulo*eje (pequenos angulos)
    p = d.xpos[pelvis]
    d.xfrc_applied[pelvis, 0:3] = (np.array([-2000 * p[0], -2000 * p[1], 8000 * (Z_BAND - p[2])])
                                   - 100 * d.qvel[0:3])
    d.xfrc_applied[pelvis, 3:6] = K_ORI * err_vec - D_ORI * d.qvel[3:6]
    mujoco.mj_step(m, d)

    for i in range(nu):
        ms = ls.motor_state[i]
        ms.q, ms.dq, ms.tau_est = float(q[i]), float(dq[i]), float(d.ctrl[i])
    quat = d.qpos[3:7]
    ls.imu_state.quaternion = [float(x) for x in quat]
    ls.imu_state.gyroscope = [float(x) for x in d.qvel[3:6]]
    w_, x_, y_, z_ = quat
    rpy = np.array([np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_)),
                    np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1)),
                    np.arctan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))])
    ls.imu_state.rpy = [float(x) for x in rpy]
    b2, b3, l2 = botones(t)
    wr = [0] * 40
    wr[2], wr[3] = b2, b3
    wr[16:20] = list(struct.pack("<f", l2))
    ls.wireless_remote = wr
    ls.tick = k
    pub.Write(ls)

    # inclinacion como bad_orientation: acos(-g_z en cuerpo) = acos(R[2,2]) con R de quat
    zz = 1 - 2 * (x_ * x_ + y_ * y_)
    tilt = float(np.degrees(np.arccos(np.clip(zz, -1, 1))))
    kp_sum = float(cmd["kp"].sum())
    if t > T_VEL + 2.0 and kp_sum > 0:
        in_velocity_seen = True
    if trigger is None and in_velocity_seen and t > T_TILT and kp_sum == 0.0:
        trigger = {"t": t, "t_since_tilt": t - T_TILT, "theta_cmd_deg": float(np.degrees(abs(theta))),
                   "tilt_deg": tilt, "roll_deg": float(np.degrees(rpy[0])), "pitch_deg": float(np.degrees(rpy[1])),
                   "ncmd": cmd["n"]}
        print("[tilt] PASSIVE detectado:", json.dumps(trigger), flush=True)

    if k % 10 == 0:
        LOG["t"].append(t)
        LOG["theta_cmd"].append(theta)
        LOG["tilt"].append(tilt)
        LOG["rpy"].append(rpy.astype(np.float32))
        LOG["kp_sum"].append(kp_sum)
        LOG["ncmd"].append(cmd["n"])
        LOG["z"].append(float(d.qpos[2]))
        LOG["lag"].append(time.perf_counter() - t0w - t)
    k += 1
    lag = (t + DT) - (time.perf_counter() - t0w)
    if lag > 0:
        time.sleep(lag)

# ultimo tilt con kp>0 antes del disparo (cota inferior del umbral observado)
tl = np.array(LOG["tilt"]); kp = np.array(LOG["kp_sum"]); tt = np.array(LOG["t"])
last_ctrl = None
if trigger is not None:
    idx = np.where((tt < trigger["t"]) & (kp > 0) & (tt > T_TILT))[0]
    if len(idx):
        last_ctrl = {"t": float(tt[idx[-1]]), "tilt_deg": float(tl[idx[-1]])}
res = {"eje": eje, "rate_rad_s": RATE, "k_ori": K_ORI, "d_ori": D_ORI, "z_band": Z_BAND, "umbral_codigo_deg": float(np.degrees(1.0)),
       "trigger": trigger, "ultimo_bajo_control": last_ctrl, "ncmd_total": cmd["n"],
       "lag_max_s": round(max(LOG["lag"]), 3), "velocity_alcanzado": in_velocity_seen}
np.savez_compressed(out_path, **{k_: np.array(v) for k_, v in LOG.items()}, res=json.dumps(res))
with open(out_path.replace(".npz", ".json"), "w") as f:
    json.dump(res, f, indent=1)
print("[tilt] fin", json.dumps(res), flush=True)
