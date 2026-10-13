#!/usr/bin/env python3
"""Continuidad y viabilidad fisica de las referencias .npz mas alla del salto articular:
raiz (posicion y orientacion), velocidad de cuerpos, inclinacion del torso, altura de la pelvis
y contacto con el suelo. Se ejecuta en gmr:latest. Salida: /d/cr/out/ref_viabilidad.json
Uso: ref_viabilidad.py <a.npz> <b.npz> ..."""
import json
import os
import sys

import numpy as np

out = {}
for p in sys.argv[1:]:
    m = np.load(p)
    fps = float(np.ravel(m["fps"])[0])
    q, bp, bq = m["joint_pos"], m["body_pos_w"], m["body_quat_w"]
    n = len(q)
    t = np.arange(n) / fps
    dq = np.abs(np.diff(q, axis=0))
    acc = np.abs(np.diff(q, n=2, axis=0)) * fps * fps
    # raiz: velocidad lineal y paso angular entre cuadros (cuaternion wxyz)
    pel = bp[:, 0]
    vroot = np.linalg.norm(np.diff(pel, axis=0), axis=1) * fps
    qr = bq[:, 0]
    dot = np.clip(np.abs(np.sum(qr[1:] * qr[:-1], axis=1)), 0, 1)
    wroot = np.degrees(2 * np.arccos(dot)) * fps
    # inclinacion: angulo entre el eje z de la pelvis y la vertical
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (qr[:, 1] ** 2 + qr[:, 2] ** 2), -1, 1)))
    vbody = np.linalg.norm(np.diff(bp, axis=0), axis=2) * fps
    zmin = bp[:, :, 2].min(axis=1)
    ib = bp[:, :, 2].argmin(axis=1)
    # tramos en el aire: el origen del cuerpo mas bajo a mas de 15 cm del suelo
    air = zmin > 0.15
    runs, k = [], 0
    while k < n:
        if air[k]:
            j = k
            while j < n and air[j]:
                j += 1
            runs.append((round(float(t[k]), 2), round((j - k) / fps, 2)))
            k = j
        else:
            k += 1
    seg = lambda a, lo, hi: a[(t[:len(a)] >= lo) & (t[:len(a)] < hi)]
    r = {
        "cuadros": n, "fps": fps, "duracion_s": round(n / fps, 2),
        "art_saltos_gt_0p5": int((dq > 0.5).sum()), "art_saltos_gt_0p2": int((dq > 0.2).sum()),
        "art_salto_max_rad": round(float(dq.max()), 3),
        "art_acel_p99_rad_s2": round(float(np.percentile(acc, 99)), 1),
        "art_acel_max_rad_s2": round(float(acc.max()), 1),
        "raiz_vel_max_m_s": round(float(vroot.max()), 2), "raiz_vel_p99_m_s": round(float(np.percentile(vroot, 99)), 2),
        "raiz_cuadros_gt_2m_s": int((vroot > 2).sum()),
        "raiz_velang_max_deg_s": round(float(wroot.max()), 0), "raiz_velang_p99_deg_s": round(float(np.percentile(wroot, 99)), 0),
        "raiz_cuadros_gt_400deg_s": int((wroot > 400).sum()),
        "cuerpo_vel_max_m_s": round(float(vbody.max()), 2), "cuerpo_pares_gt_6m_s": int((vbody > 6).sum()),
        "inclin_media_deg": round(float(tilt.mean()), 1), "inclin_p95_deg": round(float(np.percentile(tilt, 95)), 1),
        "inclin_max_deg": round(float(tilt.max()), 1), "inclin_frac_gt_30deg": round(float((tilt > 30).mean()), 3),
        "inclin_frac_gt_57deg": round(float((tilt > 57.3).mean()), 3),
        "pelvis_z_min_m": round(float(pel[:, 2].min()), 3), "pelvis_z_media_m": round(float(pel[:, 2].mean()), 3),
        "pelvis_z_max_m": round(float(pel[:, 2].max()), 3), "pelvis_frac_lt_0p5m": round(float((pel[:, 2] < 0.5).mean()), 3),
        "cuerpo_mas_bajo_z_min_m": round(float(zmin.min()), 3), "cuerpo_mas_bajo_z_media_m": round(float(zmin.mean()), 3),
        "frac_en_el_aire": round(float(air.mean()), 3), "tramos_en_el_aire": len(runs),
        "tramo_aire_mas_largo_s": max([d for _, d in runs], default=0.0),
        "cuerpo_mas_bajo_no_pie_frac": None,
        "primeros_4s": {"inclin_max_deg": round(float(seg(tilt, 0, 4).max()), 1),
                        "pelvis_z_min_m": round(float(seg(pel[:, 2], 0, 4).min()), 3),
                        "raiz_vel_max_m_s": round(float(seg(vroot, 0, 4).max()), 2),
                        "raiz_velang_max_deg_s": round(float(seg(wroot, 0, 4).max()), 0),
                        "frac_en_el_aire": round(float(seg(air, 0, 4).mean()), 3)},
        # perfil cada 5 s para ver en que tramo se concentra el problema
        "perfil_5s": [{"t0": lo, "inclin_max": round(float(seg(tilt, lo, lo + 5).max()), 0),
                       "pelvis_z_min": round(float(seg(pel[:, 2], lo, lo + 5).min()), 2),
                       "raiz_vel_max": round(float(seg(vroot, lo, lo + 5).max()), 1),
                       "raiz_velang_max": round(float(seg(wroot, lo, lo + 5).max()), 0),
                       "aire": round(float(seg(air, lo, lo + 5).mean()), 2)}
                      for lo in range(0, int(t[-1]), 5)],
        "cuerpo_mas_bajo_indices": {int(i): int(c) for i, c in zip(*np.unique(ib, return_counts=True))},
    }
    del r["cuerpo_mas_bajo_no_pie_frac"]
    out[os.path.basename(p)] = r

json.dump(out, open("/d/cr/out/ref_viabilidad.json", "w"), indent=1)
keys = [k for k in next(iter(out.values())) if k not in ("perfil_5s", "primeros_4s", "cuerpo_mas_bajo_indices")]
names = list(out)
print("metrica".ljust(30) + "".join(x.replace(".npz", "").rjust(15) for x in names))
for k in keys:
    print(k.ljust(30) + "".join(str(out[x][k]).rjust(15) for x in names))
for x in names:
    print("\n##", x, "primeros_4s", out[x]["primeros_4s"], "| cuerpo mas bajo", out[x]["cuerpo_mas_bajo_indices"])
    for s in out[x]["perfil_5s"]:
        print("  ", s)
