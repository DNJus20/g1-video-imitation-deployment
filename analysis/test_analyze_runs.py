#!/usr/bin/env python3
"""
test_analyze_runs.py — Test de regresión de `analyze_runs.py` (lado Code).
Autocontenido (solo numpy). NO toca robot ni GPU. Correr CADA VEZ que se toque el análisis:

    python test_analyze_runs.py

Cubre los bugs/hallazgos que encontró Desktop (2026-07-31):
  A1. combo STOP L1+R1+B antes del baile NO adelanta el inicio.
  A2. tropiezo DESPUÉS del baile NO contamina el tilt de la ventana.
  A3. marca de retorno (R2+A) corta la ventana antes.
  A4. outcome sin completar -> inválido.
  B1. corrida con timebase roto (tick en unidad mala) -> tick_ok=False, usable=False,
      y EXCLUIDA del RMSE agregado (no contamina el titular).
  B2. latencia de 150 ms -> align_lag_s ≈ -0,15 y RMSE ≈ 0,020 (búsqueda fina de 2 ms).
Ruido inyectado = 0,02 rad => RMSE correcto ≈ 0,02 rad.
"""
import os, sys, json, tempfile, subprocess
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_runs as A

FS, NJ, REF_F, REF_FPS = 500.0, 29, 2849, 50.0
NOISE = 0.02
rng = np.random.default_rng(7)


def ref_signal(tsec):
    return np.stack([0.3 * np.sin(2 * np.pi * (0.4 + 0.05 * j) * tsec + j) for j in range(NJ)], axis=1)


def make_run(d, name, outcome="success", pre_stop=False, tail_tilt_deg=0.0, return_at_s=None,
             tick_unit_ms=True, lag_s=0.0, T=120.0):
    N = int(T * FS); t = np.arange(N) / FS
    tick = (np.arange(N) * (2 if tick_unit_ms else 1)).astype(np.uint32)  # ms si *2; roto si *1
    buttons = np.zeros(N, np.uint16)
    if pre_stop:
        buttons[int(10 * FS):int(10 * FS) + 40] = 0x0203     # L1+R1+B (STOP) -> NO debe contar
    i_d = int(30 * FS); buttons[i_d:i_d + 40] = 0x0201         # R1+B (marca de baile) @30s FIJA
    if return_at_s:
        j = int(return_at_s * FS); buttons[j:j + 40] = 0x0110  # R2+A (retorno)
    q = np.zeros((N, NJ), np.float32)
    dur = REF_F / REF_FPS; m0 = 30.0 + lag_s                   # movimiento arranca lag_s tras el botón
    idx = (t >= m0) & (t < m0 + dur); tin = t[idx] - m0
    q[idx] = ref_signal(tin).astype(np.float32)
    q += rng.normal(0, NOISE, q.shape).astype(np.float32)
    rpy = rng.normal(0, np.radians(0.5), (N, 3)).astype(np.float32)
    rpy[idx, 1] += (np.radians(8) * np.sin(2 * np.pi * 0.5 * (t[idx] - m0))).astype(np.float32)
    if tail_tilt_deg:
        k = int(100 * FS); rpy[k:k + 500, 1] = np.radians(tail_tilt_deg)   # POST-baile
    temp = np.full((N, NJ, 2), 40, np.float32); temp[:, 22:29, :] = 58
    np.savez_compressed(os.path.join(d, f"{name}.npz"),
        t=t.astype(np.float32), tick=tick, q=q, dq=np.zeros_like(q), tau_est=np.zeros_like(q),
        mode=np.ones((N, NJ), np.int8), vol=np.full((N, NJ), 48, np.float32),
        temperature=temp, motorstate=np.zeros((N, NJ), np.int32), rpy=rpy,
        gyro=np.zeros((N, 3), np.float32), accel=np.zeros((N, 3), np.float32), buttons=buttons)
    json.dump({"run": name, "outcome": outcome, "condition": "harness_loose", "notes": ""},
              open(os.path.join(d, f"{name}.json"), "w", encoding="utf-8"))


def approx(x, target, tol):
    return x is not None and abs(x - target) <= tol


def main():
    d = tempfile.mkdtemp()
    tref = np.arange(REF_F) / REF_FPS
    ref_npz = os.path.join(d, "baileaxe57.npz")
    np.savez_compressed(ref_npz, joint_pos=ref_signal(tref).astype(np.float32))
    ref_q = np.load(ref_npz)["joint_pos"].astype(np.float64)

    make_run(d, "run01_clean")
    make_run(d, "run02_prestop", pre_stop=True)                 # A1
    make_run(d, "run03_tailtilt", tail_tilt_deg=35.0)           # A2
    make_run(d, "run04_earlyreturn", return_at_s=45.0)          # A3
    make_run(d, "run05_todo", outcome="TODO: success | fall | aborted")  # A4
    make_run(d, "run06_tickbad", tick_unit_ms=False)            # B1
    make_run(d, "run07_latency", lag_s=0.150)                   # B2

    res = {}
    for name in ["run01_clean", "run02_prestop", "run03_tailtilt", "run04_earlyreturn",
                 "run05_todo", "run06_tickbad", "run07_latency"]:
        r, _, _, _ = A.analyze_one(os.path.join(d, f"{name}.npz"), ref_q, None)
        res[name] = r

    checks = []
    def chk(cond, msg): checks.append((bool(cond), msg))

    r = res["run01_clean"]
    chk(r["dance_start_found"], "run01: inicio detectado")
    chk(approx(r["rmse_all_rad"], 0.02, 0.004), f"run01: RMSE={r['rmse_all_rad']} ~ 0.02")
    chk(approx(r["max_abs_pitch_deg"], 8.0, 3.0), f"run01: max_pitch={r['max_abs_pitch_deg']} ~ 8")
    chk(approx(r["window_s"], 56.98, 2.0), f"run01: window={r['window_s']} ~ 57s")
    chk(r["usable"], "run01: usable=True")

    r = res["run02_prestop"]
    chk(r["n_dance_combo_presses"] == 1, f"run02: n_dance_combo_presses={r['n_dance_combo_presses']}==1 (L1+R1+B excluido)")
    chk(approx(r["rmse_all_rad"], 0.02, 0.004), f"run02: RMSE={r['rmse_all_rad']} ~ 0.02 (no inflado)")

    r = res["run03_tailtilt"]
    chk(r["max_abs_pitch_deg"] < 15.0, f"run03: max_pitch={r['max_abs_pitch_deg']} < 15 (cola fuera)")

    r = res["run04_earlyreturn"]
    chk(approx(r["window_s"], 15.0, 2.0), f"run04: window={r['window_s']} ~ 15s")
    chk(r["window_end_source"] == "return_mark", f"run04: window_end_source={r['window_end_source']}")

    r = res["run05_todo"]
    chk(not r["outcome_valid"], "run05: outcome inválido detectado")
    chk(not r["usable"], "run05: usable=False")

    # B1 — timebase roto: tick_ok=False, usable=False
    r = res["run06_tickbad"]
    chk(not r["tick_ok"], "run06: tick_ok=False (discrepancia detectada)")
    chk(not r["usable"], "run06: usable=False (excluible del agregado)")

    # B2 — latencia 150 ms: lag ≈ -0.15, RMSE recuperado
    r = res["run07_latency"]
    chk(approx(r["align_lag_s"], -0.15, 0.01), f"run07: align_lag_s={r['align_lag_s']} ~ -0.15")
    chk(approx(r["rmse_all_rad"], 0.02, 0.003), f"run07: RMSE={r['rmse_all_rad']} ~ 0.02 (fino corrige la latencia)")

    # --- Verificación del AGREGADO (hallazgo A): run06 debe quedar EXCLUIDA del RMSE ---
    out = os.path.join(d, "outputs")
    subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_runs.py"),
                    "--data", d, "--ref", ref_npz, "--out", out],
                   check=True, capture_output=True)
    summ = json.load(open(os.path.join(out, "summary.json"), encoding="utf-8"))["summary"]
    excl = [e["run"] for e in summ.get("excluded_from_rmse", [])]
    chk("run06_tickbad" in excl, f"agregado: run06_tickbad EXCLUIDA del RMSE (excluidas={excl})")
    chk(approx(summ["hardware_joint_rmse_rad_mean"], 0.02, 0.004),
        f"agregado: RMSE usable={summ['hardware_joint_rmse_rad_mean']} ~ 0.02 (no contaminado)")

    ok = all(c for c, _ in checks)
    for c, m in checks:
        print(f"  [{'OK ' if c else 'FALLO'}] {m}")
    print("\n=== TODOS OK ===" if ok else "\n=== HAY FALLOS ===")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
