#!/usr/bin/env python3
"""Forense de la jornada 2026-07-31: linea de tiempo, ventanas de baile,
huecos de captura vs huecos del tick del robot, y cortes de .161."""
import json, os, sys, datetime as dt
import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

P = os.environ.get("PAPER_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))  # carpeta con data/ (disposicion original del proyecto)
RUNS = [
    ("run00_prueba",            os.path.join(P, "evidencia_no_protocolo")),
    ("run01_baileaxe_arnes",    os.path.join(P, "data")),
    ("run02_baileaxe_sinarnes", os.path.join(P, "evidencia_no_protocolo")),
    ("run03_baileaxe_sinarnes", os.path.join(P, "data")),
    ("run04_baileaxe_sinarnes", os.path.join(P, "data")),
    ("run05_baileaxe_sinarnes", os.path.join(P, "data")),
    ("run06_baileaxe_sinarnes", os.path.join(P, "data")),
    ("run07_baileaxe_sinarnes", os.path.join(P, "data")),
]
BTN_R1, BTN_L1, BTN_START, BTN_R2, BTN_A, BTN_B = 1<<0, 1<<1, 1<<2, 1<<4, 1<<8, 1<<9
REF_FPS = 50.0

def fmt(t):  # datetime -> HH:MM:SS.mmm
    return t.strftime("%H:%M:%S.") + f"{t.microsecond//1000:03d}"

out = {}
print("="*110)
for name, d in RUNS:
    npz = os.path.join(d, name + ".npz")
    js  = os.path.join(d, name + ".json")
    z = np.load(npz)
    meta = json.load(open(js, encoding="utf-8"))
    t = z["t"].astype(np.float64); t -= t[0]          # reloj local (perf_counter), s
    tick = z["tick"].astype(np.int64)
    tk = (tick - tick[0]).astype(np.float64)/1000.0    # reloj del firmware, s
    buttons = z["buttons"]
    mode = z["mode"]
    w0 = dt.datetime.strptime(meta["wall_start"], "%Y-%m-%d %H:%M:%S")

    # --- ventana del baile: flanco R1+B sin L1 ---
    m = ((buttons & BTN_R1)!=0) & ((buttons & BTN_B)!=0) & ((buttons & BTN_L1)==0)
    edges = np.flatnonzero(m & ~np.r_[False, m[:-1]])
    n_press = len(edges)
    i0 = int(edges[0]) if n_press else None

    # --- huecos: en el reloj LOCAL vs en el TICK del robot ---
    dt_loc  = np.diff(t)
    dt_tick = np.diff(tk)
    gaps = np.flatnonzero(dt_loc > 0.050)              # >50 ms en nuestra captura
    # congelamiento del tick: el reloj local avanza pero el del robot no
    frozen = np.flatnonzero((dt_loc > 0.050) & (dt_tick < 0.5*dt_loc))
    regress = int(np.sum(dt_tick < 0))                 # tick hacia atras
    dup = int(np.sum(dt_tick == 0))                    # mismo tick repetido

    rec = {
        "run": name,
        "outcome": meta.get("outcome"),
        "condition": meta.get("condition"),
        "wall_start": fmt(w0),
        "wall_end": fmt(w0 + dt.timedelta(seconds=float(t[-1]))),
        "dur_local_s": round(float(t[-1]),2),
        "dur_tick_s": round(float(tk[-1]),2),
        "samples": int(len(t)),
        "n_dance_presses": n_press,
        "n_gaps_gt50ms": int(len(gaps)),
        "max_gap_ms": round(float(dt_loc.max()*1000),1),
        "n_tick_frozen_in_gaps": int(len(frozen)),
        "tick_regressions": regress,
        "tick_duplicates": dup,
    }
    if i0 is not None:
        rec["dance_start"] = fmt(w0 + dt.timedelta(seconds=float(t[i0])))
        rec["dance_end"]   = fmt(w0 + dt.timedelta(seconds=float(t[i0]) + 56.98))
    else:
        rec["dance_start"] = rec["dance_end"] = None

    # --- damping: motores que pasan a mode 0 ---
    ctrl = (mode != 0).mean(axis=1)                    # fraccion de motores bajo control
    lost = np.flatnonzero(ctrl < 0.5)
    rec["primer_mode0"] = fmt(w0 + dt.timedelta(seconds=float(t[lost[0]]))) if len(lost) else None
    rec["frac_bajo_control_final"] = round(float(ctrl[-1]),3)

    # --- los 5 huecos mayores, con su contraparte en el tick ---
    if len(gaps):
        top = gaps[np.argsort(dt_loc[gaps])[::-1][:5]]
        rec["huecos_mayores"] = [
            {"t_s": round(float(t[g]),2),
             "local_ms": round(float(dt_loc[g]*1000),1),
             "tick_ms": round(float(dt_tick[g]*1000),1)} for g in sorted(top)]
    else:
        rec["huecos_mayores"] = []

    out[name] = rec
    print(f"{name:26s} {rec['wall_start']} -> {rec['wall_end']}  "
          f"({rec['dur_local_s']:7.2f}s local / {rec['dur_tick_s']:7.2f}s tick)  "
          f"baile={rec['dance_start']}  presses={n_press}  "
          f"huecos>50ms={rec['n_gaps_gt50ms']:3d} (max {rec['max_gap_ms']:7.1f} ms)  "
          f"tick_congelado={rec['n_tick_frozen_in_gaps']}  retro={regress}  dup={dup}")

print("="*110)
json.dump(out, open(os.path.join(P,"analysis","outputs","forense_jornada.json"),"w",encoding="utf-8"),
          indent=2, ensure_ascii=False)
print("escrito analysis/outputs/forense_jornada.json")
