"""Error de posicion de cuerpos RELATIVO AL ANCLA (torso_link) y alineado en yaw.
Es la parte de `error_body_pos` de la tarea de tracking que se puede medir sin conocer la
posicion global de la base: p_rel = Rz(-yaw_ancla) (p_cuerpo - p_ancla). No incluye el
termino de altura del ancla (en hardware no hay estimacion de la altura de la base)."""
import numpy as np

CUERPOS = ("pelvis", "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link",
           "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link", "torso_link",
           "left_shoulder_roll_link", "left_elbow_link", "left_wrist_yaw_link",
           "right_shoulder_roll_link", "right_elbow_link", "right_wrist_yaw_link")
ANCLA = 7
GRUPOS = {"todos_14": list(range(14)), "piernas_6": [1, 2, 3, 4, 5, 6], "brazos_6": [8, 9, 10, 11, 12, 13],
          "tobillos_2": [3, 6], "munecas_2": [10, 13], "pelvis_1": [0]}


def yaw(q):  # q[...,4] wxyz
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def relativo(pos, quat_ancla):
    """pos [...,14,3], quat_ancla [...,4] -> posiciones relativas al ancla, yaw alineado."""
    d = pos - pos[..., ANCLA:ANCLA + 1, :]
    psi = yaw(quat_ancla)[..., None]
    c, s = np.cos(-psi), np.sin(-psi)
    out = d.copy()
    out[..., 0] = c * d[..., 0] - s * d[..., 1]
    out[..., 1] = s * d[..., 0] + c * d[..., 1]
    return out


def resumen(rel_rob, rel_ref):
    e = np.linalg.norm(rel_rob - rel_ref, axis=-1)          # [..., 14]
    return {k: round(float(e[..., idx].mean()), 4) for k, idx in GRUPOS.items()}
