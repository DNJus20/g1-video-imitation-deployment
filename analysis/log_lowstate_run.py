#!/usr/bin/env python3
"""
log_lowstate_run.py — Captura READ-ONLY de rt/lowstate para las corridas del paper HRFEST.
VERSIÓN ENDURECIDA (2026-07-30): autosave periódico + captura de SIGHUP + snapshot
alineado al cerrar + escritura atómica. Blinda contra pérdida de una corrida por caída
de ssh/cable (que ya ha pasado en este setup). NO publica ningún comando: solo suscribe.

Uso (en la PC GPU, dentro del contenedor g1-mujoco, con el cable Ethernet al robot):

  docker run --rm -it --network host \
    -v $HOME/g1_ws:/workspace \
    g1-mujoco:latest \
    python3 /workspace/log_lowstate_run.py --iface enp130s0 --out /workspace/paper_runs --run run01_baileaxe_arnes

  # Flujo por corrida (mando UNITREE, nomenclatura xbox):
  #   1. Lanzar este script ANTES de iniciar el baile (queda grabando; autosave cada 5 s).
  #   2. Ejecutar la corrida con el mando:
  #        L2+R2 (debug mode) -> L1+R1+A (launcher) -> L2+Arriba (FixStand)
  #        -> R2+A (Velocity)  -> R1+B (baileaxe)  -> [al terminar vuelve a Velocity]
  #   3. Al volver a Velocity, Ctrl+C aquí -> guarda <run>.npz + <run>.json.
  #   4. Anotar en el JSON impreso: outcome (success/fall/aborted), condition (arnés/sin), notes.

Guarda a ~frecuencia completa del lowstate (~500 Hz):
  t (perf_counter local), tick (reloj del firmware), q/dq/tau_est/mode/vol (29),
  temperaturas (29x2), motorstate (29), IMU (rpy, gyro, accel), mode_machine, y la
  palabra de botones del mando (para alinear el inicio del baile con el combo R1+B).

NOTAS PARA EL ANÁLISIS (mañana):
  - Para dt preciso usar `tick` (reloj del firmware, ms), NO `t` (perf_counter tiene jitter de red/callback).
  - El RMSE de q se compara contra baileaxe57.npz (orden mjlab). Los datos aquí están en el
    ORDEN CRUDO de motores del robot (0..28): aplicar el `joint_ids_map` del deploy.yaml antes
    de comparar (para este robot era identidad [0..28], pero CONFIRMAR).
  - `mode` != 0 confirma que el motor estaba bajo control (no en damping) durante el baile.

Robot: G1 EDU 29 DOF (unitree_hg). Dominio DDS 0, interfaz real (p.ej. enp130s0).
"""

import argparse
import json
import os
import signal
import sys
import time

import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

NUM_JOINTS = 29  # G1 EDU 29 DOF


class Recorder:
    def __init__(self):
        self.t = []            # tiempo local (perf_counter) por muestra
        self.tick = []         # tick del firmware (reloj autoritativo)
        self.q = []            # (29,) posiciones [rad]
        self.dq = []           # (29,) velocidades [rad/s]
        self.tau = []          # (29,) torque estimado [Nm]
        self.mode = []         # (29,) modo del motor (0=damping, !=0 bajo control)
        self.vol = []          # (29,) voltaje de bus [V]
        self.temp = []         # (29,2) bobinado + driver [°C]
        self.motorstate = []   # (29,) palabra de error por motor (0 = sin falla)
        self.rpy = []          # (3,) roll/pitch/yaw [rad]
        self.gyro = []         # (3,)
        self.accel = []        # (3,)
        self.imu_temp = []     # temperatura de la IMU [°C] — vive DENTRO de la caja de .161:
                               # es el mejor proxy de la temperatura interna del controlador
        self.buttons = []      # uint16 de wireless_remote bytes[2:4]
        self.mode_machine = None
        self.n = 0

    def on_lowstate(self, msg: LowState_):
        # OJO: n se incrementa AL FINAL. Así, un lector que tome k=self.n y corte [:k]
        # obtiene todos los campos con >= k entradas completas => siempre alineados.
        now = time.perf_counter()
        ms = msg.motor_state
        self.t.append(now)
        self.tick.append(msg.tick)
        self.q.append([ms[i].q for i in range(NUM_JOINTS)])
        self.dq.append([ms[i].dq for i in range(NUM_JOINTS)])
        self.tau.append([ms[i].tau_est for i in range(NUM_JOINTS)])
        self.mode.append([ms[i].mode for i in range(NUM_JOINTS)])
        self.vol.append([ms[i].vol for i in range(NUM_JOINTS)])
        self.temp.append([[ms[i].temperature[0], ms[i].temperature[1]] for i in range(NUM_JOINTS)])
        self.motorstate.append([ms[i].motorstate for i in range(NUM_JOINTS)])
        imu = msg.imu_state
        self.rpy.append(list(imu.rpy))
        self.gyro.append(list(imu.gyroscope))
        self.accel.append(list(imu.accelerometer))
        self.imu_temp.append(float(imu.temperature))
        wr = msg.wireless_remote
        self.buttons.append(int(wr[2]) | (int(wr[3]) << 8))
        self.mode_machine = msg.mode_machine
        self.n += 1
        if self.n % 2500 == 0:  # ~cada 5 s a 500 Hz
            r, p = self.rpy[-1][0], self.rpy[-1][1]
            print(f"  [{self.n:>7d} muestras] rpy=({r:+.2f},{p:+.2f}) "
                  f"btn=0x{self.buttons[-1]:04x} tmax={np.max(self.temp[-1]):.0f}C "
                  f"IMU={self.imu_temp[-1]:.0f}C",
                  flush=True)


def _atomic_savez(npz_path, compress=True, **arrays):
    """Checkpoints SIN comprimir: np.savez_compressed bloquea el hilo de captura y el
    costo crece con el buffer (medido: 0,3 s a 30k muestras -> 1,44 s a 137k), lo que
    producía huecos de hasta ~900 ms en el registro. El guardado final sí comprime."""
    base = npz_path[:-4] if npz_path.endswith(".npz") else npz_path
    tmp = base + ".part.npz"
    (np.savez_compressed if compress else np.savez)(tmp, **arrays)
    os.replace(tmp, npz_path)  # rename atómico: el archivo final nunca queda a medias


def _atomic_write_json(json_path, obj):
    tmp = json_path + ".part"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)
    os.replace(tmp, json_path)


def save_snapshot(rec, npz_path, json_path, t_start_wall, final):
    """Guarda un snapshot consistente de las primeras k=rec.n muestras. Seguro concurrente:
    solo lee las primeras k entradas (que ya están completas y no se mueven)."""
    k = rec.n
    if k == 0:
        return 0, None
    t = np.asarray(rec.t[:k], dtype=np.float64)
    t = t - t[0]
    dur = float(t[-1]) if k > 1 else 0.0
    hz = k / dur if dur > 0 else 0.0

    _atomic_savez(
        npz_path,
        compress=final,
        t=t.astype(np.float32),
        tick=np.asarray(rec.tick[:k], dtype=np.uint32),
        q=np.asarray(rec.q[:k], dtype=np.float32),
        dq=np.asarray(rec.dq[:k], dtype=np.float32),
        tau_est=np.asarray(rec.tau[:k], dtype=np.float32),
        mode=np.asarray(rec.mode[:k], dtype=np.int8),
        vol=np.asarray(rec.vol[:k], dtype=np.float32),
        temperature=np.asarray(rec.temp[:k], dtype=np.float32),
        motorstate=np.asarray(rec.motorstate[:k], dtype=np.int32),
        rpy=np.asarray(rec.rpy[:k], dtype=np.float32),
        gyro=np.asarray(rec.gyro[:k], dtype=np.float32),
        accel=np.asarray(rec.accel[:k], dtype=np.float32),
        imu_temp=np.asarray(rec.imu_temp[:k], dtype=np.float32),
        buttons=np.asarray(rec.buttons[:k], dtype=np.uint16),
    )

    rpy_arr = np.asarray(rec.rpy[:k])
    temp_arr = np.asarray(rec.temp[:k])
    ms_arr = np.asarray(rec.motorstate[:k])
    meta = {
        "run": os.path.splitext(os.path.basename(npz_path))[0],
        "complete": bool(final),           # False = checkpoint de autosave; True = corrida cerrada bien
        "wall_start": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_start_wall)),
        "duration_s": round(dur, 2),
        "samples": k,
        "rate_hz": round(hz, 1),           # <-- VERIFICAR ~500 en la Corrida 0
        "mode_machine": rec.mode_machine,
        "max_abs_roll_deg": round(float(np.max(np.abs(np.degrees(rpy_arr[:, 0])))), 1),
        "max_abs_pitch_deg": round(float(np.max(np.abs(np.degrees(rpy_arr[:, 1])))), 1),
        "max_motor_temp_C": round(float(np.max(temp_arr)), 1),
        "imu_temp_C_inicio": round(float(rec.imu_temp[0]), 1) if rec.imu_temp else None,
        "imu_temp_C_final": round(float(rec.imu_temp[k - 1]), 1) if rec.imu_temp else None,
        "imu_temp_C_max": round(float(np.max(rec.imu_temp[:k])), 1) if rec.imu_temp else None,
        "any_motor_fault": bool(np.any(ms_arr != 0)),
        "capture_gaps_gt_50ms": int(np.sum(np.diff(np.asarray(rec.tick[:k], dtype=np.int64)) > 50)),
        # === COMPLETAR A MANO tras la corrida ===
        "outcome": "TODO: success | fall | aborted",
        "condition": "TODO: harness_loose | no_harness",
        "notes": "TODO: incidencias, observaciones",
    }
    _atomic_write_json(json_path, meta)
    return k, meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="enp130s0", help="interfaz de red hacia el robot")
    ap.add_argument("--domain", type=int, default=0, help="dominio DDS (0 = robot real)")
    ap.add_argument("--out", default=".", help="directorio de salida")
    ap.add_argument("--run", required=True, help="nombre de la corrida, p.ej. run01_baileaxe_arnes")
    ap.add_argument("--autosave", type=float, default=15.0, help="segundos entre checkpoints (0 = off)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    npz_path = os.path.join(args.out, f"{args.run}.npz")
    json_path = os.path.join(args.out, f"{args.run}.json")
    if os.path.exists(npz_path):
        sys.exit(f"ERROR: {npz_path} ya existe. Usa otro nombre de corrida.")

    print(f"[log] Inicializando DDS: dominio {args.domain}, iface {args.iface} (solo lectura)")
    ChannelFactoryInitialize(args.domain, args.iface)

    rec = Recorder()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(rec.on_lowstate, 50)

    t_start_wall = time.time()
    print(f"[log] Grabando '{args.run}'. Autosave cada {args.autosave:g}s. "
          f"Ejecuten la corrida. Ctrl+C para cerrar y guardar.")

    stop = {"flag": False}
    def _stop(*_):
        stop["flag"] = True
    for sig in ("SIGINT", "SIGTERM", "SIGHUP"):  # SIGHUP = caída de ssh/tty
        if hasattr(signal, sig):
            signal.signal(getattr(signal, sig), _stop)

    last_save = time.perf_counter()
    while not stop["flag"]:
        time.sleep(0.2)
        if args.autosave > 0 and (time.perf_counter() - last_save) >= args.autosave:
            if rec.n > 0:
                save_snapshot(rec, npz_path, json_path, t_start_wall, final=False)  # checkpoint
            last_save = time.perf_counter()

    print(f"\n[log] Cerrando: {rec.n} muestras.")
    if rec.n == 0:
        sys.exit("ERROR: no llegó ningún lowstate. ¿Interfaz correcta? ¿Cable conectado?")

    k, meta = save_snapshot(rec, npz_path, json_path, t_start_wall, final=True)
    print(f"[log] Guardado FINAL: {npz_path}  ({os.path.getsize(npz_path)/1e6:.1f} MB)")
    print(f"[log] Metadata: {json_path}  <-- COMPLETAR outcome/condition/notes")
    if meta["rate_hz"] < 400:
        print(f"[log] ⚠️  rate_hz={meta['rate_hz']} < 400: Python no siguió los 500 Hz "
              f"(se perdieron muestras). Data usable para tilt/térmica; RMSE con menor resolución.")
    print(json.dumps(meta, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
