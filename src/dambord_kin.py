"""
Kinematica van de xArm7 (voorwaarts en inverse), gedeeld door dambord_move.py en
dambord_setup.py. Gebaseerd op xarm_description/config/kinematics/default/
xarm7_default_kinematics.yaml. Het eindpunt ligt TCP_OFFSET_Z voorbij de flens
(0 zonder gripper, 0,172 m met gripper), zie dambord_config.py.
"""

import math

import numpy as np

import dambord_config as cfg

_KIN = [  # x, y, z, roll, pitch, yaw per gewricht
    (0.0, 0.0, 0.267, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, -1.5708, 0.0, 0.0),
    (0.0, -0.293, 0.0, 1.5708, 0.0, 0.0),
    (0.0525, 0.0, 0.0, 1.5708, 0.0, 0.0),
    (0.0775, -0.3425, 0.0, 1.5708, 0.0, 0.0),
    (0.0, 0.0, 0.0, 1.5708, 0.0, 0.0),
    (0.076, 0.097, 0.0, -1.5708, 0.0, 0.0),
]
LOWER = np.array([-2 * math.pi, -2.059, -2 * math.pi, -0.19198,
                  -2 * math.pi, -1.69297, -2 * math.pi])
UPPER = np.array([2 * math.pi, 2.0944, 2 * math.pi, 3.927,
                  2 * math.pi, math.pi, 2 * math.pi])
# Grijper wijst recht omlaag: quaternion (x=1, y=0, z=0, w=0)
R_OMLAAG = np.diag([1.0, -1.0, -1.0])


def _rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


_A = []
for _x, _y, _z, _r, _p, _yw in _KIN:
    _m = np.eye(4)
    _m[:3, :3] = _rpy(_r, _p, _yw)
    _m[:3, 3] = (_x, _y, _z)
    _A.append(_m)


def fk(q):
    """Voorwaartse kinematica: geeft (4x4 transformatie van het eindpunt, frames)."""
    t = np.eye(4)
    frames = []
    for i in range(7):
        c, s = math.cos(q[i]), math.sin(q[i])
        rz = np.eye(4)
        rz[:2, :2] = [[c, -s], [s, c]]
        t = t @ _A[i] @ rz
        frames.append(t.copy())
    tcp = t.copy()
    tcp[:3, 3] = t[:3, 3] + t[:3, 2] * cfg.TCP_OFFSET_Z
    return tcp, frames


def _rotvec(r):
    c = (np.trace(r) - 1.0) / 2.0
    hoek = math.acos(max(-1.0, min(1.0, c)))
    if hoek < 1e-9:
        return np.zeros(3)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return v / (2.0 * math.sin(hoek)) * hoek


def _ik_dls(pos, q_start, iters=300, lam=0.02):
    """Eenvoudige IK (damped least squares); alleen gebruikt voor de klaarstand."""
    q = np.clip(np.array(q_start, dtype=float), LOWER, UPPER)
    for _ in range(iters):
        t, frames = fk(q)
        ep = np.asarray(pos) - t[:3, 3]
        eo = _rotvec(R_OMLAAG @ t[:3, :3].T)
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(eo) < 1e-3:
            return q
        jac = _jacobiaan(t, frames)
        jp = jac.T @ np.linalg.inv(jac @ jac.T + lam ** 2 * np.eye(6))
        dq = jp @ np.concatenate([ep, eo])
        m = np.max(np.abs(dq))
        if m > 0.2:
            dq *= 0.2 / m
        q = np.clip(q + dq, LOWER, UPPER)
    return None


def _jacobiaan(t, frames):
    jac = np.zeros((6, 7))
    for i in range(7):
        z = frames[i][:3, 2]
        jac[:3, i] = np.cross(z, t[:3, 3] - frames[i][:3, 3])
        jac[3:, i] = z
    return jac


_KLAAR = None


def klaar_q():
    """Gewrichtshoeken van de veilige klaarstand (gripperpunt op cfg.KLAAR_TCP)."""
    global _KLAAR
    if _KLAAR is None:
        q = _ik_dls(cfg.KLAAR_TCP, np.zeros(7))
        if q is None:
            raise RuntimeError(
                "Klaarstand niet bereikbaar; pas KLAAR_TCP aan in dambord_config.py"
            )
        _KLAAR = q
    return _KLAAR.copy()


def ik(pos, q_start, iters=200, gain=0.3):
    """
    Inverse kinematica vertrekkend vanaf q_start, voor positie 'pos' met de gripper omlaag.
    De arm heeft 7 gewrichten voor een taak van 6 vrijheidsgraden; die overtollige
    vrijheid wordt gebruikt om dicht bij de klaarstand te blijven. Zo drijft de armstand
    niet weg na veel bewegingen. Geeft de gewrichtshoeken of None.
    """
    voorkeur = klaar_q()
    q = np.clip(np.array(q_start, dtype=float), LOWER, UPPER)
    for _ in range(iters):
        t, frames = fk(q)
        ep = np.asarray(pos) - t[:3, 3]
        eo = _rotvec(R_OMLAAG @ t[:3, :3].T)
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(eo) < 1e-3:
            return q
        jac = _jacobiaan(t, frames)
        jp = np.linalg.pinv(jac, rcond=1e-3)
        dq = jp @ np.concatenate([ep, eo]) + (np.eye(7) - jp @ jac) @ (voorkeur - q) * gain
        m = np.max(np.abs(dq))
        if m > 0.2:
            dq *= 0.2 / m
        q = np.clip(q + dq, LOWER, UPPER)
    return None
