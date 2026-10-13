#!/usr/bin/env python3
"""Reconstruye la inclinacion (angulo de la gravedad proyectada, = bad_orientation del ejecutor)
alrededor del disparo del damping en una corrida. Uso: tilt_disparo.py run.npz t0 t_passive"""
import sys, math, numpy as np
d = np.load(sys.argv[1]); t0 = float(sys.argv[2]); tp = float(sys.argv[3])
t = d["t"]; rpy = d["rpy"].astype(float); mode = d["mode"]; tw = d["temperature"]
tilt = np.degrees(np.arccos(np.clip(np.cos(rpy[:, 0]) * np.cos(rpy[:, 1]), -1, 1)))
w = (t >= t0) & (t < tp); i = np.where(w)[0]
print("tilt max antes del disparo (deg):", round(float(tilt[i].max()), 1), "a t=", round(float(t[i][tilt[i].argmax()] - t0), 2))
j = np.searchsorted(t, tp)
print("tilt en el instante Passive:", round(float(tilt[j]), 1), "roll", round(math.degrees(rpy[j, 0]), 1), "pitch", round(math.degrees(rpy[j, 1]), 1))
k = np.where((t >= t0) & (tilt > 57.3))[0]
print("primer cruce de 57.3 deg a t=", round(float(t[k[0]] - t0), 3) if len(k) else None)
for s in [20, 22, 24, 25, 26, 26.5, 27, 27.28, 27.5, 28]:
    m = np.searchsorted(t, t0 + s)
    print("  t=%5.2fs tilt=%5.1f roll=%6.1f pitch=%6.1f modes0=%d kp_hint=%s" % (s, tilt[m], math.degrees(rpy[m, 0]), math.degrees(rpy[m, 1]), int((mode[m] == 0).sum()), ""))
hot = np.unravel_index(tw[w].argmax(), tw[w].shape)
print("temp max en ventana", float(tw[w].max()), "motor", int(hot[1]), "idx", int(hot[2]))
# pico de gyro antes del disparo (velocidad de caida)
g = d["gyro"].astype(float); print("gyro max |w| en ultimos 2 s antes del disparo (rad/s):", round(float(np.abs(g[(t >= tp - 2) & (t < tp)]).max()), 2))
