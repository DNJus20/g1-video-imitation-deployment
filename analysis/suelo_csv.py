# Correccion de altura al suelo de un CSV de GMR (root_pos 3 | root_rot 4 | dof 29, 30 fps).
# UNICO cambio: se resta a la z de la raiz la deriva lenta de la altura de la suela mas baja.
# No toca articulaciones, orientacion de la raiz ni su posicion horizontal.
#   h(t)  = altura sobre el suelo del punto mas bajo de las dos suelas (capsulas de colision de
#           g1.xml: radio 0,01 m en z local -0,025), calculada del .npz ya convertido (50 fps).
#   b(t)  = media movil(W) del minimo movil(W) de h, W = 1,5 s centrada. Por construccion
#           b(t) <= h(t): no introduce penetracion, y un salto real (< W) se conserva.
#   z_raiz(t) -= b(t), interpolada a los instantes del CSV.
# Uso: suelo_csv.py <entrada.csv> <referencia.npz> <salida.csv> [fps_csv=30] [W_s=1.5]
import json
import sys

import numpy as np

ANKLES = (6, 12)  # left_ankle_roll_link, right_ankle_roll_link en el orden de cuerpos del .npz
R_CAP = 0.01
# extremos de las capsulas del pie en el marco del tobillo (simetricas en y)
PTS = np.array([[x, s * y, -0.025] for x, y in
                [(0.1, 0.026), (0.05, 0.027), (-0.044, 0.018), (0.123, 0.018), (-0.052, 0.01),
                 (0.13, 0.01), (-0.054, 0.0), (0.132, 0.0)] for s in (1, -1)])


def rot(q, v):
    """Rota v (P,3) por los cuaterniones wxyz q (N,4) -> (N,P,3)."""
    w, u = q[:, None, :1], q[:, None, 1:]
    v = v[None]
    t = 2 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def suela(npz):
    m = np.load(npz)
    fps = float(np.ravel(m["fps"])[0])
    z = [m["body_pos_w"][:, b, None, 2] + rot(m["body_quat_w"][:, b], PTS)[..., 2] for b in ANKLES]
    return np.concatenate(z, axis=1).min(axis=1) - R_CAP, fps


def movil(x, k, f):
    pad = np.pad(x, k // 2, mode="edge")
    return np.array([f(pad[i:i + k]) for i in range(len(x))])


if __name__ == "__main__":
    src, ref, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    fps_csv = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0
    W_S = float(sys.argv[5]) if len(sys.argv) > 5 else 1.5
    h, fps = suela(ref)
    k = int(round(W_S * fps)) | 1
    b = movil(movil(h, k, np.min), k, np.mean)
    assert (b <= h + 1e-9).all()
    m = np.loadtxt(src, delimiter=",", dtype=np.float64)
    assert m.shape[1] == 36, m.shape
    t_csv, t_ref = np.arange(len(m)) / fps_csv, np.arange(len(h)) / fps
    assert abs(t_csv[-1] - t_ref[-1]) < 2 / fps, (t_csv[-1], t_ref[-1])
    bc = np.interp(t_csv, t_ref, b)
    out = m.copy()
    out[:, 2] -= bc
    np.savetxt(dst, out, delimiter=",", fmt="%.18e")
    cols = [c for c in range(36) if c != 2]
    print(json.dumps({
        "entrada": src, "referencia": ref, "salida": dst, "ventana_s": W_S, "cuadros": int(len(m)),
        "suela_antes_m": {"media": round(float(h.mean()), 3), "p95": round(float(np.percentile(h, 95)), 3),
                          "max": round(float(h.max()), 3), "min": round(float(h.min()), 3),
                          "frac_gt_0p25": round(float((h > 0.25).mean()), 3)},
        "correccion_m": {"media": round(float(bc.mean()), 3), "max": round(float(bc.max()), 3),
                         "min": round(float(bc.min()), 3)},
        "suela_despues_estimada_m": {"media": round(float((h - b).mean()), 3), "max": round(float((h - b).max()), 3),
                                     "min": round(float((h - b).min()), 4),
                                     "frac_gt_0p25": round(float(((h - b) > 0.25).mean()), 3)},
        "columnas_no_z_identicas": bool(np.array_equal(out[:, cols], m[:, cols])),
    }))
