#!/usr/bin/env python3
"""N5 (revisor 3): la misma busqueda de desfase que se aplica en hardware (analyze_runs.rmse_tracking),
aplicada al banco del binario y al simulador de entrenamiento. Se ejecuta en gmr:latest (CPU).
Salida: /workspace/cr/out/n5_lag.json"""
import glob, json, sys
import numpy as np
sys.path.insert(0, "/workspace/cr")
import analyze_runs as ar

out = {"banco": {}, "sim_entrenamiento": {}}
for p in sorted(glob.glob("/workspace/cr/out/sim2sim_*_r[0-9].npz")):
    z = np.load(p, allow_pickle=True); j = json.load(open(p[:-4] + ".json"))
    ini = next(x["t_s"] for x in j["transiciones"] if x["a"] == j["estado"])
    ref = z["ref_q"].astype(float)
    w = (z["t"] >= ini) & (z["t"] < ini + len(ref) / 50.0)
    rm = ar.rmse_tracking(z["q"][w].astype(float), z["t"][w] - ini, ref, None)
    out["banco"][p.split("/")[-1][:-4]] = {"rmse_sin_lag_deg": j["rmse_all_deg"], "lag_s": rm["align_lag_s"],
                                           "rmse_con_lag_deg": round(float(np.degrees(rm["rmse_all_rad"])), 2)}
for tag in ["sim_eval_baileaxe_final", "sim_eval_tinkus_final"]:
    z = np.load("/workspace/cr/out/%s.npz" % tag); ref, rob = z["ref"], z["rob"]      # (T, E, 29) a 50 Hz
    r = {}
    for k in range(-3, 6):                                 # robot retrasado k pasos de 20 ms
        a, b = (ref[:len(ref) - k], rob[k:]) if k >= 0 else (ref[-k:], rob[:len(rob) + k])
        r[k] = float(np.degrees(np.sqrt(np.mean((a - b) ** 2))))
    kb = min(r, key=r.get)
    out["sim_entrenamiento"][tag] = {"rmse_sin_lag_deg": round(r[0], 2), "mejor_desfase_pasos": kb,
                                     "mejor_desfase_s": round(-kb * 0.02, 3), "rmse_con_lag_deg": round(r[kb], 2),
                                     "barrido_deg": {str(k): round(v, 2) for k, v in r.items()}}
json.dump(out, open("/workspace/cr/out/n5_lag.json", "w"), indent=1)
print(json.dumps(out, indent=1))
