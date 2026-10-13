#!/usr/bin/env python3
"""Verifica que las referencias 'g' solo difieren de las originales en la altura, y mide la suela.
Se ejecuta en gmr:latest. Salida: /d/cr/out/verifica_g.json"""
import json
import sys

import numpy as np

sys.path.insert(0, "/d/cr")
from suelo_csv import suela  # noqa: E402

D = "/d/unitree_rl_mjlab/src/assets/motions/g1/"
out = {}
for n in ("bailemix57", "baileaxe57"):
    a, b = np.load(D + n + ".npz"), np.load(D + n + "g.npz")
    r = {"cuadros": [int(len(a["joint_pos"])), int(len(b["joint_pos"]))]}
    r["dif_max_joint_pos_rad"] = float(np.abs(a["joint_pos"] - b["joint_pos"]).max())
    r["dif_max_body_quat"] = float(np.abs(a["body_quat_w"] - b["body_quat_w"]).max())
    d = a["body_pos_w"] - b["body_pos_w"]
    r["dif_max_body_xy_m"] = float(np.abs(d[..., :2]).max())
    # el desplazamiento vertical debe ser el mismo para todos los cuerpos en cada cuadro (rigido)
    r["dif_z_no_rigida_max_m"] = float(np.abs(d[..., 2] - d[:, :1, 2]).max())
    r["bajada_z_m"] = {"media": round(float(d[:, 0, 2].mean()), 3), "max": round(float(d[:, 0, 2].max()), 3)}
    r["salto_art_max_rad"] = round(float(np.abs(np.diff(b["joint_pos"], axis=0)).max()), 3)
    for tag, f in (("antes", n), ("despues", n + "g")):
        h, fps = suela(D + f + ".npz")
        t = np.arange(len(h)) / fps
        r["suela_" + tag] = {"media": round(float(h.mean()), 3), "p95": round(float(np.percentile(h, 95)), 3),
                             "max": round(float(h.max()), 3), "min": round(float(h.min()), 4),
                             "frac_gt_0p10": round(float((h > 0.10).mean()), 3),
                             "frac_gt_0p25": round(float((h > 0.25).mean()), 3),
                             "media_por_5s": [round(float(h[(t >= s) & (t < s + 5)].mean()), 2)
                                              for s in range(0, int(t[-1]), 5)]}
    out[n + "g"] = r
json.dump(out, open("/d/cr/out/verifica_g.json", "w"), indent=1)
print(json.dumps(out, indent=1))
