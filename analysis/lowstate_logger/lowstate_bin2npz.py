#!/usr/bin/env python3
"""
lowstate_bin2npz.py — Convierte el .bin de lowstate_logger (Jetson) al MISMO esquema .npz que
log_lowstate_run.py, para que analyze_runs.py / analyze_app_run.py funcionen sin cambios.
Claves: t (s desde el inicio, perf local), tick, q, dq, tau_est, mode, vol, temperature (29x2),
motorstate, rpy, gyro, accel, imu_temp, buttons. Además escribe un .json resumen con los mismos
campos que el logger de Python (outcome/condition/notes a completar).

Uso: python3 lowstate_bin2npz.py entrada.bin salida.npz [--outcome success --condition harness --notes "..."]
"""
import argparse
import json
import os
import time

import numpy as np

NJ = 29
REC = np.dtype([
    ("t_wall", "<f8"), ("tick", "<u4"),
    ("q", "<f4", (NJ,)), ("dq", "<f4", (NJ,)), ("tau_est", "<f4", (NJ,)),
    ("mode", "i1", (NJ,)), ("vol", "<f4", (NJ,)), ("temp", "<i2", (NJ, 2)),
    ("motorstate", "<u4", (NJ,)),
    ("rpy", "<f4", (3,)), ("gyro", "<f4", (3,)), ("accel", "<f4", (3,)),
    ("imu_temp", "<i2"), ("buttons", "<u2"), ("mode_machine", "u1"), ("mode_pr", "u1"),
])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("npz")
    ap.add_argument("--outcome", default="TODO: success | fall | aborted")
    ap.add_argument("--condition", default="TODO: harness | no_harness")
    ap.add_argument("--notes", default="")
    a = ap.parse_args()

    with open(a.bin, "rb") as f:
        magic = f.read(8)
        recsz = int(np.frombuffer(f.read(4), "<u4")[0])
        assert magic[:6] == b"G1LSv1", f"magic inesperado: {magic!r}"
        assert recsz == REC.itemsize, f"tamaño de registro {recsz} != dtype {REC.itemsize}"
        data = np.frombuffer(f.read(), dtype=REC)
    n = len(data)
    if n == 0:
        raise SystemExit("archivo vacío")
    t = data["t_wall"] - data["t_wall"][0]
    tick = data["tick"].astype(np.int64)
    dt = np.diff(tick)
    np.savez_compressed(
        a.npz,
        t=t.astype(np.float32), tick=data["tick"].astype(np.uint32),
        q=data["q"], dq=data["dq"], tau_est=data["tau_est"], mode=data["mode"], vol=data["vol"],
        temperature=data["temp"].astype(np.float32), motorstate=data["motorstate"].astype(np.int32),
        rpy=data["rpy"], gyro=data["gyro"], accel=data["accel"],
        imu_temp=data["imu_temp"].astype(np.float32), buttons=data["buttons"].astype(np.uint16),
    )
    rpy = np.degrees(data["rpy"].astype(np.float64))
    meta = {
        "run": os.path.splitext(os.path.basename(a.npz))[0],
        "complete": True,
        "logger": "lowstate_logger (C++ en el Jetson, DDS interno eth0, sin cable externo)",
        "wall_start": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(float(data["t_wall"][0]))),
        "duration_s": round(float(t[-1]), 2),
        "samples": int(n),
        "rate_hz": round(1000.0 / float(np.mean(dt)), 1) if n > 1 else None,
        "rate_hz_wall": round(n / float(t[-1]), 1) if t[-1] > 0 else None,
        "mode_machine": int(data["mode_machine"][-1]),
        "max_abs_roll_deg": round(float(np.max(np.abs(rpy[:, 0]))), 1),
        "max_abs_pitch_deg": round(float(np.max(np.abs(rpy[:, 1]))), 1),
        "max_motor_temp_C": int(np.max(data["temp"])),
        "max_casing_C_idx0": int(np.max(data["temp"][:, :, 0])),
        "max_winding_C_idx1": int(np.max(data["temp"][:, :, 1])),
        "imu_temp_C_max": int(np.max(data["imu_temp"])),
        "any_motor_fault": bool(np.any(data["motorstate"] != 0)),
        "capture_gaps_gt_50ms": int(np.sum(dt > 50)),
        "n_mode0_max": int(np.max(np.sum(data["mode"] == 0, axis=1))),
        "outcome": a.outcome, "condition": a.condition, "notes": a.notes,
    }
    jp = os.path.splitext(a.npz)[0] + ".json"
    with open(jp, "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(json.dumps(meta, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
