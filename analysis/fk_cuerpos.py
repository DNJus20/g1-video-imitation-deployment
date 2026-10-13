#!/usr/bin/env python3
"""N6 (revisor 3): error de posicion de cuerpos en HARDWARE y en el BANCO del binario, por
cinematica directa (MuJoCo) desde q + roll/pitch de la IMU. Se ejecuta en g1-mujoco:latest (CPU).
Salida: /workspace/cr/out/n6_cuerpos_fk.json"""
import csv, glob, json, sys
import mujoco, numpy as np
sys.path.insert(0, "/workspace/cr")
from cuerpos_comun import CUERPOS, ANCLA, relativo, resumen
import analyze_runs as ar

XML = "/workspace/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1.xml"
m = mujoco.MjModel.from_xml_path(XML); d = mujoco.MjData(m)
assert m.nq == 36, m.nq
bid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in CUERPOS]
nombres = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(1, m.nbody)]


def rpy2quat(r, p, y=0.0):
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def fk(q, quat_raiz, pos_raiz=None):
    T = len(q); P = np.zeros((T, 14, 3)); QA = np.zeros((T, 4)); todos = np.zeros((T, m.nbody - 1, 3))
    for i in range(T):
        d.qpos[:3] = 0 if pos_raiz is None else pos_raiz[i]
        d.qpos[3:7] = quat_raiz[i]; d.qpos[7:] = q[i]
        mujoco.mj_kinematics(m, d)
        P[i] = d.xpos[bid]; QA[i] = d.xquat[bid[ANCLA]]; todos[i] = d.xpos[1:]
    return P, QA, todos


def interp(t_ref, t, x):
    return np.stack([np.interp(t_ref, t, x[:, j]) for j in range(x.shape[1])], axis=1)


def medir(ref, q, rpy, t, lag):
    """t: tiempo medido relativo al inicio del baile. Devuelve el resumen de error."""
    N = ref["joint_pos"].shape[0]; ref_t = np.arange(N) / 50.0
    tm = t + lag
    keep = (ref_t >= max(0.0, tm[0])) & (ref_t <= min(tm[-1], ref_t[-1]))
    qi, ri = interp(ref_t[keep], tm, q), interp(ref_t[keep], tm, rpy[:, :2])
    quat = np.stack([rpy2quat(a, b) for a, b in ri])
    P, QA, _ = fk(qi, quat)
    nb = ref["_idx"]
    rel_ref = relativo(ref["body_pos_w"][keep][:, nb], ref["body_quat_w"][keep][:, nb[ANCLA]])
    r = resumen(relativo(P, QA), rel_ref); r["cuadros"] = int(keep.sum())
    r["rmse_art_deg"] = round(float(np.degrees(np.sqrt(np.mean((qi - ref["joint_pos"][keep]) ** 2)))), 2)
    return r


def cargar_ref(p):
    z = np.load(p); ref = {k: z[k] for k in z.files}
    assert ref["body_pos_w"].shape[1] == len(nombres), (ref["body_pos_w"].shape, len(nombres))
    ref["_idx"] = [nombres.index(n) for n in CUERPOS]
    # validacion: FK propio sobre la referencia debe reproducir body_pos_w de la referencia
    k = slice(0, len(ref["joint_pos"]), 25)
    _, _, todos = fk(ref["joint_pos"][k], ref["body_quat_w"][k][:, 0], ref["body_pos_w"][k][:, 0])
    ref["_valid_mm"] = round(float(np.abs(todos - ref["body_pos_w"][k]).max() * 1000), 3)
    return ref


M = "/workspace/unitree_rl_mjlab/src/assets/motions/g1/"
out = {"metrica": "media de ||p_rel_robot - p_rel_ref|| sobre cuerpos y cuadros; p_rel relativo a torso_link, yaw alineado; m",
       "cuerpos": list(CUERPOS), "hardware": {}, "banco": {}}
ref = cargar_ref(M + "baileaxe57.npz"); out["validacion_fk_vs_referencia_mm"] = {"baileaxe57": ref["_valid_mm"]}
print("validacion FK (mm):", ref["_valid_mm"], flush=True)
lags = {r["run"]: float(r["align_lag_s"]) for r in csv.DictReader(open("/workspace/cr/per_run.csv"))}
for p in sorted(glob.glob("/workspace/paper_runs/run0[1-7]_baileaxe_*.npz")):
    run = p.split("/")[-1][:-4]
    if run not in lags:
        continue
    z = np.load(p); tt = (z["tick"].astype(np.int64) - z["tick"][0]) / 1000.0
    i0, _ = ar.find_dance_start(z["buttons"]); i1 = int(np.searchsorted(tt, tt[i0] + 57.5))
    u = np.r_[True, np.diff(tt[i0:i1]) > 0]                  # ticks duplicados fuera
    r = medir(ref, z["q"][i0:i1][u].astype(float), z["rpy"][i0:i1][u].astype(float), (tt[i0:i1] - tt[i0])[u], lags[run])
    r["lag_s"] = lags[run]; out["hardware"][run] = r; print(run, r, flush=True)
for tag, mot in [("baileaxe", "baileaxe57.npz"), ("tinkus", "bailetinkus30.npz")]:
    rf = ref if tag == "baileaxe" else cargar_ref(M + mot)
    out["validacion_fk_vs_referencia_mm"][mot[:-4]] = rf["_valid_mm"]
    for p in sorted(glob.glob("/workspace/cr/out/sim2sim_%s_r*.npz" % tag)):
        z = np.load(p, allow_pickle=True); j = json.load(open(p[:-4] + ".json"))
        ini = next(x["t_s"] for x in j["transiciones"] if x["a"] == j["estado"])
        w = (z["t"] >= ini - 0.05) & (z["t"] <= ini + len(rf["joint_pos"]) / 50.0 + 0.05)
        r = medir(rf, z["q"][w].astype(float), z["rpy"][w].astype(float), z["t"][w] - ini, 0.0)
        out["banco"][p.split("/")[-1][:-4]] = r; print(p.split("/")[-1], r, flush=True)
CL = ["todos_14", "piernas_6", "brazos_6", "tobillos_2", "munecas_2", "pelvis_1", "rmse_art_deg"]
for grupo in ["hardware", "banco_baileaxe", "banco_tinkus"]:
    src = out["hardware"] if grupo == "hardware" else {k: v for k, v in out["banco"].items() if grupo.split("_")[1] in k}
    if src:
        out["media_" + grupo] = {g: [round(float(np.mean([v[g] for v in src.values()])), 4),
                                 round(float(np.std([v[g] for v in src.values()], ddof=1)), 4)] for g in CL}
json.dump(out, open("/workspace/cr/out/n6_cuerpos_fk.json", "w"), indent=1)
print(json.dumps({k: v for k, v in out.items() if k.startswith("media") or k.startswith("valid")}, indent=1))
