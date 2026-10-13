# Aplica a un CSV de GMR (root_pos 3 | root_rot 4 | dof 29, 30 fps) EXACTAMENTE el
# mismo filtro que pipe/fix_motion.py (mediana 5 + Savitzky-Golay 9/3 en articulaciones,
# 9/2 en raiz, limitador 0,35 rad/cuadro), SIN recorte y SIN cierre en pose de pie:
# la unica diferencia entre la referencia original y la filtrada es el filtro.
# Uso: fix_csv.py <entrada.csv> <salida.csv> [fps=30]
import sys, json, numpy as np
from scipy.signal import medfilt, savgol_filter

src, dst = sys.argv[1], sys.argv[2]
fps = int(sys.argv[3]) if len(sys.argv) > 3 else 30
m = np.loadtxt(src, delimiter=",", dtype=np.float64)
assert m.shape[1] == 36, m.shape
rp, rr, q = m[:, :3], m[:, 3:7], m[:, 7:]


def smooth(x, med=5, win=9, poly=3):
    y = x.copy()
    for j in range(y.shape[1]):
        y[:, j] = medfilt(y[:, j], med)
    return savgol_filter(y, win, poly, axis=0)


def stats(qq):
    dq = np.abs(np.diff(qq, axis=0)); mx = dq.max(axis=1)
    return {"cuadros_gt_0.5": int((mx > 0.5).sum()), "pares_gt_0.5": int((dq > 0.5).sum()),
            "cuadros_gt_0.3": int((mx > 0.3).sum()), "max_rad": round(float(mx.max()), 4),
            "max_rad_s": round(float(mx.max() * fps), 2)}


rep = {"entrada": src, "salida": dst, "fps": fps, "cuadros": int(len(q)),
       "quat_flips": int((np.sum(rr[1:] * rr[:-1], axis=1) < 0).sum()), "antes": stats(q)}
q_s = smooth(q)
rp_s = smooth(rp, med=5, win=9, poly=2)
rr_s = smooth(rr, med=5, win=9, poly=2)
rr_s /= np.linalg.norm(rr_s, axis=1, keepdims=True)
rep["suavizado"] = stats(q_s)
VMAX = 0.35
for i in range(1, len(q_s)):
    q_s[i] = q_s[i - 1] + np.clip(q_s[i] - q_s[i - 1], -VMAX, VMAX)
rep["despues"] = stats(q_s)
rep["rms_cambio_rad"] = round(float(np.sqrt(np.mean((q_s - q) ** 2))), 4)
rep["max_cambio_rad"] = round(float(np.abs(q_s - q).max()), 4)
rep["max_cambio_raiz_m"] = round(float(np.abs(rp_s - rp).max()), 4)
np.savetxt(dst, np.hstack([rp_s, rr_s, q_s]), delimiter=",", fmt="%.18e")
print(json.dumps(rep))
