#!/usr/bin/env python3
"""G6 (lote CR): temperaturas de la campana del 31-07 contra los limites del SDK oficial.

Limites: unitree_sdk2 (commit 21d0a3b, 2026-07-09), include/unitree/robot/g1/common/terminations.hpp
  motor_winding_overheat: temperature[1] > 120 C   (bobinado)
  motor_casing_overheat:  temperature[0] > 85 C    (carcasa)
El SDK los presenta como funciones de proteccion recomendadas ("set the motor to passive"),
no como el umbral del firmware del actuador.
Salida: paper/data/cr/g6_termico.json
"""
import glob
import json
import os

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(AQUI, "..", "data")
JOINTS = json.load(open(os.path.join(AQUI, "outputs", "discontinuidades.json")))["joints"]
LIM = {"carcasa": (0, 85.0), "bobinado": (1, 120.0)}

res = {"fuente_limites": "unitree_sdk2 21d0a3b2c46e (2026-07-09) include/unitree/robot/g1/common/terminations.hpp:50-65",
       "limites_C": {k: v[1] for k, v in LIM.items()}, "corridas": {}}
for p in sorted(glob.glob(os.path.join(DATA, "run0[1-7]_*.npz"))):
    T = np.load(p)["temperature"]
    r = {}
    for nom, (c, lim) in LIM.items():
        m = T[:, :, c].max(0)
        i = int(m.argmax())
        r[nom] = {"max_C": float(m.max()), "motor": i, "articulacion": JOINTS[i],
                  "pct_del_limite": round(100 * float(m.max()) / lim, 1)}
    res["corridas"][os.path.basename(p)[:-4]] = r
pico = max(res["corridas"].values(), key=lambda r: r["bobinado"]["max_C"])["bobinado"]
res["pico_sesion"] = pico
os.makedirs(os.path.join(DATA, "cr"), exist_ok=True)
json.dump(res, open(os.path.join(DATA, "cr", "g6_termico.json"), "w"), indent=2, ensure_ascii=False)
print(json.dumps(pico))
