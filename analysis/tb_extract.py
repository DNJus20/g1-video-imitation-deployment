#!/usr/bin/env python3
"""Metricas de entrenamiento (TensorBoard) de las corridas del proyecto, en iteraciones fijas.
Se ejecuta en mjlab:latest. Salida: /workspace/cr/out/tb_metricas.json"""
import glob
import json
import os

from tensorboard.backend.event_processing import event_accumulator as ea

BASE = "/workspace/unitree_rl_mjlab/logs/rsl_rl/g1_tracking"
RUNS = {
    "2026-07-21_19-47-02": "baileaxe57 (politica desplegada)",
    "2026-07-22_17-19-12": "bailemix57 (no convergio, detenido)",
    "2026-09-08_17-04-34": "bailetinkus30 (segundo movimiento)",
    "2026-07-31_01-06-19": "baileaxe57 config sim2real-hardened (segfault)",
}
CLAVES = ["Metrics/motion/error_body_pos", "Metrics/motion/error_joint_pos",
          "Episode_Termination/ee_body_pos", "Episode_Termination/anchor_pos",
          "Episode_Termination/anchor_ori", "Episode_Termination/time_out",
          "Train/mean_reward", "Train/mean_episode_length"]
ITERS = [500, 1000, 2000, 3000, 5000, 10000, 14999]

out = {}
for run, desc in RUNS.items():
    ev = glob.glob(os.path.join(BASE, run, "events.out.tfevents.*"))
    if not ev:
        out[run] = {"descripcion": desc, "error": "sin archivo de eventos"}
        continue
    acc = ea.EventAccumulator(ev[0], size_guidance={ea.SCALARS: 0})
    acc.Reload()
    tags = acc.Tags()["scalars"]
    r = {"descripcion": desc, "tags_disponibles": len(tags)}
    for c in CLAVES:
        if c not in tags:
            continue
        s = acc.Scalars(c)
        steps = [x.step for x in s]
        r[c] = {"ultima_iter": steps[-1], "valor_final": round(s[-1].value, 4)}
        for it in ITERS:
            if it <= steps[-1]:
                x = min(s, key=lambda e: abs(e.step - it))
                r[c]["iter_%d" % it] = round(x.value, 4)
    r["iter_max"] = max((v["ultima_iter"] for k, v in r.items() if isinstance(v, dict)), default=None)
    out[run] = r

json.dump(out, open("/workspace/cr/out/tb_metricas.json", "w"), indent=2)
print(json.dumps({k: {"iter_max": v.get("iter_max"),
                      "ebp_3000": v.get("Metrics/motion/error_body_pos", {}).get("iter_3000"),
                      "ebp_final": v.get("Metrics/motion/error_body_pos", {}).get("valor_final")}
                  for k, v in out.items()}, indent=1))
