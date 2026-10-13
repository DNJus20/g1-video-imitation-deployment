#!/usr/bin/env python3
"""Curvas de entrenamiento (TensorBoard) cada 500 iteraciones, media en ventana de 200.
Uso (en mjlab:latest): tb_curvas.py <salida.json> <run_dir> [<run_dir> ...]   (nombres bajo logs/rsl_rl/g1_tracking)"""
import glob, json, os, sys
from tensorboard.backend.event_processing import event_accumulator as ea

BASE = "/workspace/unitree_rl_mjlab/logs/rsl_rl/g1_tracking"
salida, runs = sys.argv[1], sys.argv[2:]
CLAVES = ["Metrics/motion/error_body_pos", "Metrics/motion/error_joint_pos", "Episode_Termination/ee_body_pos",
          "Train/mean_reward", "Train/mean_episode_length"]
out = {}
for run in runs:
    acc = ea.EventAccumulator(glob.glob(os.path.join(BASE, run, "events.out.tfevents.*"))[0], size_guidance={ea.SCALARS: 0})
    acc.Reload()
    r = {}
    for c in CLAVES:
        s = acc.Scalars(c); st = [x.step for x in s]; v = [x.value for x in s]
        row = {}
        for it in range(500, 15001, 500):
            lo = [v[i] for i in range(len(st)) if it - 100 <= st[i] < it + 100]
            if lo:
                row[str(it)] = round(sum(lo) / len(lo), 4)
        r[c] = row
    out[run] = r
json.dump(out, open(salida, "w"), indent=1)
print("ok", salida, list(out))
