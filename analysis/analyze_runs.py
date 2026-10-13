#!/usr/bin/env python3
"""
analyze_runs.py — Análisis de las corridas del paper HRFEST 2026 (lado Code).
v2 (2026-07-31) — corregido tras la revisión adversarial de Desktop (2 bugs + 3 menores).

Lee las capturas read-only de `log_lowstate_run.py` (data/runNN_*.npz + .json) y la
referencia (data/baileaxe57.npz), y produce las métricas de la Sección V.

Salidas en `--out`:
  - `<run>.analysis.json` — un JSON por corrida.
  - `per_run.csv` — CSV agregado (una fila por corrida) para volcar a la Tabla I.
  - `summary.json` — agregado: N, tasa de éxito, RMSE articular medio±σ (col. hardware Tabla II).
  - `<run>_fig3.png` — roll/pitch vs t con el umbral 57° marcado (Figura 3), si hay matplotlib.

VENTANA DEL BAILE (clave): se detecta por la marca de botones.
  - Inicio: flanco de **R1+B** (lanzar baileaxe) EXCLUYENDO L1 (para no confundir con el
    STOP del lanzador L1+R1+B). Se cuenta cuántas veces aparece el patrón (a la CSV).
  - Fin: primer **R2+A o Start** después del inicio (retorno a balanceo); si no aparece,
    se cierra en inicio + duración de la referencia (56,98 s). TODAS las métricas se
    calculan SOLO dentro de la ventana.

Alineación fina: tras el flanco de botón hay latencia de transición de estado (~100-150 ms).
Se busca el desfase (lag) que minimiza el RMSE global con búsqueda GRUESA (20 ms, ±0,6 s)
seguida de REFINAMIENTO FINO (2 ms, la resolución de la captura a 500 Hz) alrededor del mínimo,
y se reporta `align_lag_s` (bandera si sale al borde del rango).
  CONVENCIÓN DE SIGNO: se aplica `tm = t_medido + lag` antes de interpolar la referencia.
  `lag < 0` ⇒ el movimiento medido empieza |lag| s DESPUÉS de la marca de botón (latencia).
  Ej.: movimiento 150 ms tarde ⇒ `align_lag_s ≈ -0,15`.

USABILIDAD PARA EL AGREGADO: cada corrida lleva `usable = tick_ok AND outcome_valid AND
align_ok (lag no al borde) AND dance_start_found AND hay_RMSE`. El RMSE agregado de hardware
se calcula SOLO sobre corridas `usable` con outcome=success; `summary.json` reporta
`n_excluded_from_rmse` con motivos. Las columnas booleanas van también al CSV (una corrida
marcada rota NO entra al promedio sin dejar rastro en el archivo).

Requiere: numpy. matplotlib OPCIONAL (para PNGs).

NOTA DE MÉTRICA (Tabla II): este RMSE es ARTICULAR (rad), NO comparable con
`error_body_pos` del training (cartesiano, m). La celda comparable sería este mismo RMSE
articular sobre la corrida sim2sim.
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

# --- Grupos articulares (orden csv_to_npz / mjlab, G1 29 DOF) ---
GROUPS = {"legs": list(range(0, 12)), "waist": list(range(12, 15)),
          "arms": list(range(15, 29)), "all": list(range(29))}

# --- Bits del mando (layout verificado en hardware) ---
BTN_R1 = 1 << 0
BTN_L1 = 1 << 1
BTN_START = 1 << 2
BTN_R2 = 1 << 4
BTN_A = 1 << 8
BTN_B = 1 << 9

REF_FPS = 50.0
VALID_OUTCOMES = {"success", "fall", "aborted"}


def _rising_edges(mask_bool):
    edges = list(np.where(mask_bool[1:] & ~mask_bool[:-1])[0] + 1)
    if mask_bool[0]:
        edges = [0] + edges
    return edges


def find_dance_start(buttons):
    """Flanco de R1+B con L1 NO pulsado (excluye el STOP L1+R1+B). Devuelve (i0|None, n_apariciones)."""
    match = ((buttons & BTN_R1) != 0) & ((buttons & BTN_B) != 0) & ((buttons & BTN_L1) == 0)
    edges = _rising_edges(match)
    return (edges[0] if edges else None), len(edges)


def find_return(buttons, i0):
    """Primer R2+A o Start después de i0 (retorno a balanceo). Devuelve índice o None."""
    ret = (((buttons & BTN_R2) != 0) & ((buttons & BTN_A) != 0)) | ((buttons & BTN_START) != 0)
    idx = np.where(ret[i0 + 1:])[0]
    return int(idx[0] + i0 + 1) if len(idx) else None


def _rmse_err(q_meas, t_meas, ref_q, lag, joint_map):
    """Error articular (F',29) alineando ref al medido desplazado por `lag`. None si no hay solape."""
    if joint_map is not None:
        q_meas = q_meas[:, joint_map]
    ref_t = np.arange(ref_q.shape[0]) / REF_FPS
    tm = t_meas + lag
    lo, hi = max(0.0, float(tm[0])), min(float(tm[-1]), float(ref_t[-1]))
    keep = (ref_t >= lo) & (ref_t <= hi)
    if keep.sum() < 2:
        return None
    ref_t_k, ref_q_k = ref_t[keep], ref_q[keep]
    interp = np.empty((keep.sum(), 29))
    for j in range(29):
        interp[:, j] = np.interp(ref_t_k, tm, q_meas[:, j])
    return interp - ref_q_k


def _best_lag(q_meas, t_meas, ref_q, joint_map, lags):
    best = None
    for lag in lags:
        err = _rmse_err(q_meas, t_meas, ref_q, float(lag), joint_map)
        if err is None:
            continue
        rmse_all = float(np.sqrt(np.mean(err ** 2)))
        if best is None or rmse_all < best[0]:
            best = (rmse_all, float(lag), err)
    return best


def rmse_tracking(q_meas, t_meas, ref_q, joint_map, lag_search=0.6, coarse=0.02, fine=0.002):
    """Lag que minimiza el RMSE global: barrido grueso (coarse) + refinamiento fino (fine)."""
    best = _best_lag(q_meas, t_meas, ref_q, joint_map,
                     np.arange(-lag_search, lag_search + 1e-9, coarse))
    if best is None:
        return None
    c = best[1]  # refinamiento a paso `fine` (2 ms) en ±coarse alrededor del mínimo grueso
    fine_best = _best_lag(q_meas, t_meas, ref_q, joint_map,
                          np.arange(c - coarse, c + coarse + 1e-9, fine))
    if fine_best is not None and fine_best[0] < best[0]:
        best = fine_best
    _, lag, err = best
    out = {"align_lag_s": round(lag, 4),
           "align_lag_at_edge": bool(abs(abs(lag) - lag_search) < coarse)}
    for name, idx in GROUPS.items():
        e = err[:, idx]
        out[f"rmse_{name}_rad"] = float(np.sqrt(np.mean(e ** 2)))
    out["rmse_per_joint_rad"] = np.sqrt(np.mean(err ** 2, axis=0)).tolist()
    out["ref_frames_used"] = int(err.shape[0])
    return out


def analyze_one(npz_path, ref_q, joint_map):
    z = np.load(npz_path)
    js_path = npz_path[:-4] + ".json"
    meta = json.load(open(js_path, encoding="utf-8")) if os.path.exists(js_path) else {}
    F = {k: z[k] for k in z.files}

    buttons = F["buttons"]
    q = F["q"].astype(np.float64)
    rpy, temp, motorstate = F["rpy"], F["temperature"], F["motorstate"]
    mode = F.get("mode")

    # --- línea de tiempo: tick (firmware, se ASUME ms) con chequeo de sanidad vs perf_counter ---
    t_perf = F["t"].astype(np.float64); t_perf -= t_perf[0]
    warnings = []
    tick_ok = True
    if "tick" in F:
        tick = F["tick"].astype(np.int64)
        tt = (tick - tick[0]).astype(np.float64) / 1000.0     # ms -> s
        dur_tick, dur_perf = tt[-1], t_perf[-1]
        if dur_perf > 0 and abs(dur_tick - dur_perf) / dur_perf > 0.01:
            tick_ok = False
            warnings.append(f"tick vs perf_counter difieren {abs(dur_tick-dur_perf)/dur_perf*100:.1f}% "
                            f"(¿tick no está en ms?): dur_tick={dur_tick:.2f}s dur_perf={dur_perf:.2f}s")
        t_src = "tick_ms"
    else:
        tt = t_perf; t_src = "perf_counter"; warnings.append("sin 'tick'; usando perf_counter")

    # --- tasa de muestreo desde tick: media, desv, muestras perdidas ---
    dt = np.diff(tt); dt = dt[dt > 0]
    rate_mean = float(1.0 / np.mean(dt)) if len(dt) else 0.0
    rate_std = float(np.std(1.0 / dt)) if len(dt) else 0.0
    med = np.median(dt) if len(dt) else 0.0
    n_gaps = int(np.sum(dt > 1.8 * med)) if med > 0 else 0     # saltos ~= muestras perdidas

    r = {
        "run": os.path.splitext(os.path.basename(npz_path))[0],
        "outcome": meta.get("outcome", "?"),
        "condition": meta.get("condition", "?"),
        "samples": int(len(q)),
        "time_source": t_src,
        "rate_hz_mean": round(rate_mean, 1),
        "rate_hz_std": round(rate_std, 1),
        "n_sample_gaps": n_gaps,
    }

    # --- ventana del baile ---
    i0, n_press = find_dance_start(buttons)
    r["dance_start_found"] = i0 is not None
    r["n_dance_combo_presses"] = int(n_press)
    ref_dur = ref_q.shape[0] / REF_FPS
    if i0 is None:
        warnings.append("NO se detectó el flanco R1+B (sin L1); métricas sobre todo el registro")
        i0 = 0
    i_ret = find_return(buttons, i0)
    end_time = tt[i0] + ref_dur
    i_end_dur = int(np.searchsorted(tt, end_time))
    i_end = min([x for x in [i_ret, i_end_dur, len(q)] if x is not None])
    i_end = max(i_end, i0 + 2)
    r["window_end_source"] = ("return_mark" if (i_ret is not None and i_ret <= i_end_dur)
                              else "ref_duration_cap")
    r["window_s"] = round(float(tt[min(i_end, len(tt)) - 1] - tt[i0]), 2)

    sl = slice(i0, i_end)
    q_d, tt_d = q[sl], tt[sl] - tt[i0]
    rpy_d, temp_d, ms_d = rpy[sl], temp[sl], motorstate[sl]

    # --- estabilidad (grados), SOLO en la ventana ---
    roll, pitch = np.degrees(rpy_d[:, 0]), np.degrees(rpy_d[:, 1])
    r["max_abs_roll_deg"] = round(float(np.max(np.abs(roll))), 1)
    r["max_abs_pitch_deg"] = round(float(np.max(np.abs(pitch))), 1)
    r["rms_roll_deg"] = round(float(np.sqrt(np.mean(roll ** 2))), 2)
    r["rms_pitch_deg"] = round(float(np.sqrt(np.mean(pitch ** 2))), 2)

    # --- térmica: máx global y por grupo, en la ventana ---
    r["max_motor_temp_C"] = round(float(np.max(temp_d)), 1)
    r["hottest_motor"] = int(np.unravel_index(np.argmax(temp_d), temp_d.shape)[1])
    for name, idx in GROUPS.items():
        if name == "all":
            continue
        r[f"max_temp_{name}_C"] = round(float(np.max(temp_d[:, idx, :])), 1)

    # --- sanidad ---
    r["any_motor_fault"] = bool(np.any(ms_d != 0))
    if mode is not None:
        md = mode[sl]
        r["frac_in_control"] = round(float(np.mean(md != 0)), 3)
        r["any_motor_uncontrolled"] = bool(np.any(md == 0))  # mode==0 durante el baile = mal

    # --- RMSE articular vs referencia (con alineación fina) ---
    rm = rmse_tracking(q_d, tt_d, ref_q, joint_map)
    if rm is None:
        warnings.append("ventana demasiado corta para RMSE")
    else:
        for name in ("all", "legs", "waist", "arms"):
            r[f"rmse_{name}_rad"] = round(rm[f"rmse_{name}_rad"], 4)
        r["rmse_all_deg"] = round(float(np.degrees(rm["rmse_all_rad"])), 2)
        r["align_lag_s"] = rm["align_lag_s"]
        if rm["align_lag_at_edge"]:
            warnings.append(f"align_lag_s={rm['align_lag_s']} al borde del rango de búsqueda")
        r["ref_frames_used"] = rm["ref_frames_used"]
        r["_rmse_full"] = rm

    # --- flags de usabilidad (para el agregado y la CSV) ---
    r["tick_ok"] = bool(tick_ok)
    r["outcome_valid"] = r["outcome"] in VALID_OUTCOMES
    r["align_lag_at_edge"] = bool(rm["align_lag_at_edge"]) if rm else True
    has_rmse = "rmse_all_rad" in r
    r["usable"] = bool(r["tick_ok"] and r["outcome_valid"] and has_rmse
                       and not r["align_lag_at_edge"] and r["dance_start_found"])
    if not r["usable"]:
        reasons = [k for k, ok in [("tick", r["tick_ok"]), ("outcome", r["outcome_valid"]),
                                   ("rmse", has_rmse), ("align", not r["align_lag_at_edge"]),
                                   ("dance_mark", r["dance_start_found"])] if not ok]
        r["unusable_reasons"] = reasons

    r["joint_map"] = ("identity[0..28]" if joint_map is None else "custom")
    r["warnings"] = warnings
    return r, F, i0, i_end


def main():
    try:  # consola/pipe Windows (cp1252) no imprime ⚠️/acentos; forzar UTF-8 (o una corrida con warning se saltaría)
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="../data")
    ap.add_argument("--ref", default="../data/baileaxe57.npz")
    ap.add_argument("--out", default="outputs")
    ap.add_argument("--joint-map", default=None, help=".json con 29 índices (joint_ids_map); por defecto identidad")
    ap.add_argument("--plots", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    ref = np.load(args.ref)
    ref_q = ref["joint_pos"].astype(np.float64)
    assert ref_q.shape[1] == 29, f"referencia con {ref_q.shape[1]} DOF, se esperaban 29"
    print(f"[an] referencia: {args.ref}  {ref_q.shape[0]} frames ({ref_q.shape[0]/REF_FPS:.2f}s @{REF_FPS:g}Hz)")

    joint_map = None
    if args.joint_map:
        joint_map = np.array(json.load(open(args.joint_map, encoding="utf-8")), dtype=int)
        assert len(joint_map) == 29
        print(f"[an] joint_map: {args.joint_map} (custom)")
    else:
        print("[an] joint_map: IDENTIDAD [0..28] (no se pasó --joint-map). CONFIRMAR con el deploy.yaml.")

    runs = sorted(glob.glob(os.path.join(args.data, "run*_*.npz")))
    if not runs:
        print(f"[an] NO hay corridas en {args.data} (aún). Script listo para cuando lleguen.")
        return

    rows, reps = [], []
    for p in runs:
        try:
            r, F, i0, i_end = analyze_one(p, ref_q, joint_map)
            # aviso ruidoso: outcome sin completar
            if r["outcome"] not in VALID_OUTCOMES:
                print(f"  ⚠️⚠️  {r['run']}: outcome='{r['outcome']}' NO válido "
                      f"(esperado success|fall|aborted). NO cuenta como éxito. ¡COMPLETAR EL JSON!")
            for w in r["warnings"]:
                print(f"  ⚠️  {r['run']}: {w}")
            print(f"  {r['run']:26s} outcome={r['outcome']:8s} win={r['window_s']}s "
                  f"RMSE={r.get('rmse_all_deg','?')}° tilt=({r['max_abs_roll_deg']},{r['max_abs_pitch_deg']})° "
                  f"Tmax={r['max_motor_temp_C']}°C rate={r['rate_hz_mean']}±{r['rate_hz_std']}")
            json.dump({k: v for k, v in r.items() if k != "_rmse_full"},
                      open(os.path.join(args.out, f"{r['run']}.analysis.json"), "w", encoding="utf-8"),
                      indent=2, ensure_ascii=False)
            rows.append(r)
            reps.append((r, F, i0, i_end))
        except Exception as e:
            print(f"  {os.path.basename(p)}: ERROR {e}")

    csv_cols = ["run", "outcome", "condition", "usable", "tick_ok", "outcome_valid", "align_lag_at_edge",
                "samples", "rate_hz_mean", "rate_hz_std", "n_sample_gaps",
                "dance_start_found", "n_dance_combo_presses", "window_s", "window_end_source", "align_lag_s",
                "rmse_all_deg", "rmse_all_rad", "rmse_legs_rad", "rmse_waist_rad", "rmse_arms_rad",
                "max_abs_roll_deg", "max_abs_pitch_deg", "rms_roll_deg", "rms_pitch_deg",
                "max_motor_temp_C", "max_temp_legs_C", "max_temp_waist_C", "max_temp_arms_C", "hottest_motor",
                "frac_in_control", "any_motor_uncontrolled", "any_motor_fault", "joint_map"]
    with open(os.path.join(args.out, "per_run.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=csv_cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    outcomes = [r["outcome"] for r in rows]
    n = len(rows)
    n_succ = sum(1 for o in outcomes if o == "success")
    n_invalid = sum(1 for o in outcomes if o not in VALID_OUTCOMES)
    succ = [r for r in rows if r["outcome"] == "success"]
    usable_succ = [r for r in succ if r.get("usable")]
    excluded = [r for r in succ if not r.get("usable")]
    rmse_usable = [r["rmse_all_rad"] for r in usable_succ]
    rmse_all_succ = [r["rmse_all_rad"] for r in succ if "rmse_all_rad" in r]  # incluye excluidas (transparencia)
    summary = {
        "n_runs": n, "n_success": n_succ, "n_invalid_outcome": n_invalid,
        "success_rate": round(n_succ / n, 3) if n else None,
        "outcomes": {o: outcomes.count(o) for o in set(outcomes)},
        # AGREGADO SOLO sobre corridas USABLE (titular del paper):
        "n_usable_success": len(usable_succ),
        "n_excluded_from_rmse": len(excluded),
        "excluded_from_rmse": [{"run": r["run"], "reasons": r.get("unusable_reasons", [])} for r in excluded],
        "hardware_joint_rmse_rad_mean": round(float(np.mean(rmse_usable)), 4) if rmse_usable else None,
        "hardware_joint_rmse_rad_std": round(float(np.std(rmse_usable)), 4) if rmse_usable else None,
        "hardware_joint_rmse_deg_mean": round(float(np.degrees(np.mean(rmse_usable))), 2) if rmse_usable else None,
        # Contraste (NO usar en el paper; solo para ver el efecto de excluir):
        "_rmse_incl_excluded_mean": round(float(np.mean(rmse_all_succ)), 4) if rmse_all_succ else None,
        "_rmse_incl_excluded_std": round(float(np.std(rmse_all_succ)), 4) if rmse_all_succ else None,
        "max_session_motor_temp_C": round(max((r["max_motor_temp_C"] for r in rows), default=0), 1),
        "any_run_with_fault": any(r["any_motor_fault"] for r in rows),
        "nota_metrica": "RMSE en espacio articular (rad); NO comparable con error_body_pos (cartesiano). Ver cabecera.",
    }
    if n_invalid:
        summary["ALERTA"] = f"{n_invalid} corrida(s) con outcome sin completar; success_rate puede estar mal."
    if excluded:
        summary.setdefault("ALERTA_rmse", f"{len(excluded)} corrida(s) success EXCLUIDA(s) del RMSE (ver excluded_from_rmse). "
                                          f"Media usable={summary['hardware_joint_rmse_rad_mean']} vs incluyendo excluidas={summary['_rmse_incl_excluded_mean']}.")
    json.dump({"summary": summary,
               "per_run": [{k: v for k, v in r.items() if k != "_rmse_full"} for r in rows]},
              open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8"), indent=2, ensure_ascii=False)

    print("\n[an] " + os.path.join(args.out, "per_run.csv"))
    print("[an] " + os.path.join(args.out, "summary.json"))
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    if args.plots:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            for r, F, i0, i_end in reps:
                tt = (F["tick"].astype(float) - F["tick"][0]) / 1000.0 if "tick" in F else F["t"].astype(float)
                rpy = F["rpy"]
                fig, ax = plt.subplots(figsize=(7, 3))
                ax.plot(tt, np.degrees(rpy[:, 0]), label="roll", lw=0.8)
                ax.plot(tt, np.degrees(rpy[:, 1]), label="pitch", lw=0.8)
                ax.axhline(57.3, ls="--", c="r", lw=0.8, label="57° (umbral damping)")
                ax.axhline(-57.3, ls="--", c="r", lw=0.8)
                ax.axvspan(tt[i0], tt[min(i_end, len(tt)) - 1], color="gray", alpha=0.15, label="ventana baile")
                ax.set_xlabel("t [s]"); ax.set_ylabel("tilt [deg]"); ax.legend(fontsize=7)
                fig.tight_layout(); fig.savefig(os.path.join(args.out, f"{r['run']}_fig3.png"), dpi=150)
                plt.close(fig)
            print(f"[an] PNGs de Figura 3 en {args.out}/")
        except ImportError:
            print("[an] matplotlib no disponible; se omiten PNGs (los CSV/JSON están).")


if __name__ == "__main__":
    main()
