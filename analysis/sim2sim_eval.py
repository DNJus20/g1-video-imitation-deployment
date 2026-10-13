#!/usr/bin/env python3
"""Evalua una corrida del banco sim2sim (salida de sim2sim_bridge.py + log de g1_ctrl).

Uso: python sim2sim_eval.py <sim2sim_TAG.npz> <g1ctrl_TAG.log> <estado_mimic> <salida.json>
Criterios (los mismos que en hardware): caida = altura de pelvis < 0,45 m o |roll|/|pitch| > 1,0 rad.
RMSE articular con la misma formula que analyze_runs.py (todas / piernas / cintura / brazos),
alineando el frame 0 de la referencia con la transicion Velocity->Mimic del log del controlador.
"""
import datetime as dt
import json
import re
import sys

import numpy as np

npz_path, log_path, estado, out_path = sys.argv[1:5]
r = np.load(npz_path)
t, q, z, rpy = r["t"], r["q"], r["z"], r["rpy"]
ref, fps = r["ref_q"], float(r["fps"])
wall0 = float(r["wall0_utc"])

# transiciones del FSM (hora del contenedor = UTC)
trans = []
for line in open(log_path, encoding="utf-8", errors="replace"):
    m = re.match(r"\[(\S+ \S+)\] \[info\] FSM: Change state from (\S+) to (\S+)", line)
    if m:
        ts = dt.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f").replace(tzinfo=dt.timezone.utc).timestamp()
        trans.append({"t_s": round(ts - wall0, 3), "de": m.group(2), "a": m.group(3)})

ini = next((x["t_s"] for x in trans if x["a"] == estado), None)
fin = next((x["t_s"] for x in trans if x["de"] == estado), None)
res = {"transiciones": trans, "estado": estado, "entro_a_mimic": ini is not None}
if ini is not None:
    fin = fin if fin is not None else float(t[-1])
    w = (t >= ini) & (t < fin)
    res["dur_mimic_s"] = round(fin - ini, 2)
    res["altura_pelvis_min_m"] = round(float(z[w].min()), 3)
    res["roll_max_deg"] = round(float(np.degrees(np.abs(rpy[w, 0]).max())), 1)
    res["pitch_max_deg"] = round(float(np.degrees(np.abs(rpy[w, 1]).max())), 1)
    cayo = bool((z[w] < 0.45).any() or (np.abs(rpy[w, :2]) > 1.0).any())
    res["caida"] = cayo
    # altura despues del mimic (vuelta a Velocity): debe seguir de pie
    wpost = t >= fin
    res["altura_pelvis_min_post_m"] = round(float(z[wpost].min()), 3) if wpost.any() else None
    # RMSE articular
    idx = np.clip(np.round((t[w] - ini) * fps).astype(int), 0, len(ref) - 1)
    e = q[w] - ref[idx]
    G = {"all": slice(0, 29), "legs": slice(0, 12), "waist": slice(12, 15), "arms": slice(15, 29)}
    for k, sl in G.items():
        v = float(np.sqrt(np.mean(e[:, sl] ** 2)))
        res["rmse_%s_rad" % k] = round(v, 4)
        res["rmse_%s_deg" % k] = round(float(np.degrees(v)), 2)
res["lag_tiempo_real_max_s"] = round(float(np.max(r["lag"])), 4)
res["lowcmd_hz_medio"] = round(float(r["ncmd"][-1] / t[-1]), 1)
json.dump(res, open(out_path, "w"), indent=2)
print(json.dumps({k: v for k, v in res.items() if k != "transiciones"}))
