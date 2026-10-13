#!/usr/bin/env python3
"""Continuidad y dinamismo de todas las referencias .npz (50 Hz) del proyecto.
Mismo criterio que analysis/discontinuidades.py: salto > 0,5 rad entre cuadros y cota por
actuador v_lim/fps. Dinamismo: recorrido de la pelvis y velocidades articulares.
Se ejecuta en g1-mujoco:latest. Salida: /workspace/cr/out/motion_stats.json"""
import glob
import json
import os

import numpy as np

D = "/workspace/unitree_rl_mjlab/src/assets/motions/g1"
# limites de velocidad por articulacion (orden DDS), de analysis/outputs/discontinuidades.json
VLIM = np.array([32, 20, 32, 20, 37, 37, 32, 20, 32, 20, 37, 37, 32, 37, 37,
                 37, 37, 37, 37, 37, 22, 22, 37, 37, 37, 37, 37, 22, 22], float)
LEGS, ARMS = slice(0, 12), slice(15, 29)

out = {}
for p in sorted(glob.glob(os.path.join(D, "*.npz"))):
    m = np.load(p)
    q = m["joint_pos"]
    fps = float(np.ravel(m["fps"])[0]) if "fps" in m.files else 50.0
    dq = np.diff(q, axis=0)
    a = np.abs(dq)
    big = a > 0.5
    over = a > (VLIM / fps)
    r = {"cuadros": int(q.shape[0]), "fps": fps, "duracion_s": round(q.shape[0] / fps, 2),
         "saltos_gt_0p5rad_pares": int(big.sum()), "saltos_gt_0p5rad_cuadros": int(big.any(1).sum()),
         "salto_max_rad": round(float(a.max()), 4),
         "salto_max_piernas_rad": round(float(a[:, LEGS].max()), 4),
         "pares_sobre_cota_actuador": int(over.sum()),
         "vel_art_brazos_media_rad_s": round(float(a[:, ARMS].mean() * fps), 3),
         "vel_art_brazos_p95_rad_s": round(float(np.percentile(a[:, ARMS], 95) * fps), 3),
         "vel_art_piernas_media_rad_s": round(float(a[:, LEGS].mean() * fps), 3)}
    if "body_pos_w" in m.files:
        pel = m["body_pos_w"][:, 0, :]
        v = np.linalg.norm(np.diff(pel[:, :2], axis=0), axis=1) * fps
        r.update({"pelvis_recorrido_xy_m": round(float(np.sum(v) / fps), 3),
                  "pelvis_rango_lateral_y_m": round(float(np.ptp(pel[:, 1])), 3),
                  "pelvis_rango_frontal_x_m": round(float(np.ptp(pel[:, 0])), 3),
                  "pelvis_vel_horizontal_max_m_s": round(float(v.max()), 3),
                  "pelvis_rango_vertical_m": round(float(np.ptp(pel[:, 2])), 3)})
    out[os.path.basename(p)] = r

json.dump(out, open("/workspace/cr/out/motion_stats.json", "w"), indent=2)
for k, v in out.items():
    print(k, v["duracion_s"], "s | saltos>0.5:", v["saltos_gt_0p5rad_pares"],
          "| lateral:", v.get("pelvis_rango_lateral_y_m"), "| brazos p95:", v["vel_art_brazos_p95_rad_s"])
