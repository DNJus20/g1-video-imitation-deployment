#!/usr/bin/env python3
"""Camera-ready, revisor 3: N3 (desviacion muestral), N4 (Tabla III con dos filas),
N7 (tasa del lazo) y N11 (deriva termica vs orden de corrida).
Entradas: analysis/outputs/per_run.csv, data/cr/sim2sim_*_r*.json, data/cr/sim_eval_*.json
Salida:   data/cr/n_stats.json"""
import csv, glob, json, os
import numpy as np

B = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
rows = list(csv.DictReader(open(os.path.join(B, "analysis/outputs/per_run.csv"))))
rows = [r for r in rows if r["usable"] == "True"]
G = ["all", "legs", "waist", "arms"]
hw = {g: np.degrees(np.array([float(r["rmse_%s_rad" % g]) for r in rows])) for g in G}
out = {"n_hw": len(rows)}

# --- N3
a = hw["all"]
out["N3"] = {"media_deg": round(a.mean(), 3), "desv_poblacional_ddof0_deg": round(a.std(ddof=0), 3),
             "desv_muestral_ddof1_deg": round(a.std(ddof=1), 3),
             "cv_ddof0_pct": round(100 * a.std(ddof=0) / a.mean(), 2),
             "cv_ddof1_pct": round(100 * a.std(ddof=1) / a.mean(), 2),
             "por_corrida_deg": [round(x, 2) for x in a],
             "por_grupo_ddof1": {g: [round(hw[g].mean(), 2), round(hw[g].std(ddof=1), 2)] for g in G}}

# --- N4
def banco(tag):
    js = [json.load(open(p)) for p in sorted(glob.glob(os.path.join(B, "data/cr/sim2sim_%s_r*.json" % tag)))]
    return {g: np.array([j["rmse_%s_deg" % g] for j in js]) for g in G}, js
bk, bjs = banco("baileaxe")
tr = json.load(open(os.path.join(B, "data/cr/sim_eval_baileaxe_final.json")))
t4 = {}
for g in G:
    h, b, s = hw[g].mean(), bk[g].mean(), tr["rmse_%s_deg" % g]
    t4[g] = {"hardware_deg": round(h, 2), "hardware_sd1": round(hw[g].std(ddof=1), 2),
             "banco_binario_deg": round(b, 2), "banco_sd1": round(bk[g].std(ddof=1), 2),
             "sim_entrenamiento_deg": round(s, 2),
             "hw_sobre_banco": round(h / b, 3), "hw_sobre_sim_entrenamiento": round(h / s, 3),
             "banco_sobre_sim_entrenamiento": round(b / s, 3)}
out["N4"] = {"tabla": t4, "n_hw": len(rows), "n_banco": len(bjs), "n_envs_sim_entrenamiento": tr["n_envs"],
             "banco_completa_sin_caida": "%d/%d" % (sum(not j["caida"] for j in bjs), len(bjs)),
             "banco_pitch_max_deg": max(j["pitch_max_deg"] for j in bjs),
             "banco_roll_max_deg": max(j["roll_max_deg"] for j in bjs)}

# --- N5 (lo que hay en los datos)
out["N5"] = {"hw_lag_por_corrida_s": [float(r["align_lag_s"]) for r in rows],
             "hw_lag_medio_s": round(float(np.mean([float(r["align_lag_s"]) for r in rows])), 4),
             "banco_lag_tiempo_real_max_s": [j["lag_tiempo_real_max_s"] for j in bjs]}

# --- N7
out["N7"] = {"banco_lowcmd_hz": [j["lowcmd_hz_medio"] for j in bjs],
             "banco_rmse_deg": [j["rmse_all_deg"] for j in bjs],
             "hw_captura_lowstate_hz": [float(r["rate_hz_mean"]) for r in rows],
             "r_captura_vs_rmse_hw": round(float(np.corrcoef([float(r["rate_hz_mean"]) for r in rows], a)[0, 1]), 3)}

# --- N11
orden = np.arange(1, len(rows) + 1, dtype=float)
T = np.array([float(r["max_motor_temp_C"]) for r in rows])
P = np.array([float(r["max_abs_pitch_deg"]) for r in rows])
Pr = np.array([float(r["rms_pitch_deg"]) for r in rows])
R = lambda x, y: float(np.corrcoef(x, y)[0, 1])
def parcial(x, y, z):  # r(x,y | z)
    rxy, rxz, ryz = R(x, y), R(x, z), R(y, z)
    return (rxy - rxz * ryz) / np.sqrt((1 - rxz ** 2) * (1 - ryz ** 2))
out["N11"] = {"temp_max_C": T.tolist(), "pitch_max_deg": P.tolist(), "pitch_rms_deg": Pr.tolist(),
              "r_temp_orden": round(R(T, orden), 4),
              "r_pitchmax_temp": round(R(P, T), 3), "r_pitchmax_orden": round(R(P, orden), 3),
              "r_pitchrms_temp": round(R(Pr, T), 3), "r_pitchrms_orden": round(R(Pr, orden), 3),
              "r_rmse_temp": round(R(a, T), 3), "r_rmse_orden": round(R(a, orden), 3),
              "parcial_pitchrms_temp_dado_orden": round(float(parcial(Pr, T, orden)), 3),
              "parcial_pitchrms_orden_dado_temp": round(float(parcial(Pr, orden, T)), 3)}
json.dump(out, open(os.path.join(B, "data/cr/n_stats.json"), "w"), indent=1)
print(json.dumps(out, indent=1))
