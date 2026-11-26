#!/usr/bin/env python3
"""
Bestuurt de xArm7 met G2-gripper over het 10x10 dambord (online én offline in ROS):
- Zwart staat aan de robotzijde (A1-J4), wit aan de overkant (A7-J10). Wit begint.
- Werkt automatisch met de echte xArm7 (192.168.1.237) of offline in RViz/MoveIt.
- Pakt schijven op, laat ze vanaf ~1 cm hoogte vallen en legt geslagen schijven naast het bord.
"""

import math
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import time

try:
    import rclpy
except ImportError:
    _cmd = (
        "source /opt/ros/*/setup.bash 2>/dev/null; "
        "source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null; "
        f"exec {sys.executable} {' '.join(sys.argv)}"
    )
    os.execvp("bash", ["bash", "-c", _cmd])

import numpy as np
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from std_msgs.msg import ColorRGBA
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from xarm_msgs.srv import Call, GripperMove, MoveJoint, PlanExec, PlanJoint, SetInt16, SetInt16ById

try:
    from tf2_ros import Buffer, TransformListener
except ImportError:
    Buffer = None

# ======================= CONFIGURATIE =======================
ROBOT_IP = "192.168.1.237"
AUTO_START_PLANNER = True

BASE_FRAME = "link_base"
EEF_FRAME = "link_tcp"
TOUCH_LINKS = [
    "link_tcp",
    "xarm_gripper_base_link",
    "left_finger",
    "right_finger",
    "left_inner_knuckle",
    "right_inner_knuckle",
    "left_outer_knuckle",
    "right_outer_knuckle",
]

TCP_OFFSET_Z = 0.0
KLAAR_TCP = [0.40, 0.0, 0.275]

# --- Bord & schijven (45,0 x 45,0 cm incl. 1,1 cm boord -> 42,8 cm speelveld) ---
BORD_STL_PAD = "/home/ubuntu/src/dambord_volledig.stl"
BORD_STL_SCHAAL = 0.001
BORD_X_NABIJ = 0.191          # begin van het 10x10 speelveld (m)
BORD_Y_MIDDEN = 0.0           # midden van het bord in Y (m)
BORD_BREEDTE = 0.428          # 10 vakjes van 4,28 cm (m)
BORD_RAND = 0.011             # boord rondom het speelveld (1,1 cm)
VAK = 0.0428                  # breedte van 1 vakje (4,28 cm)
BORD_HOOGTE = 0.032           # bovenkant van het bord boven het tafelblad (3,2 cm)

SCHIJF_HOOGTE = 0.012         # dikte van 1 damschijf (1,2 cm)
SCHIJF_DIAMETER = 0.036       # diameter van 1 damschijf (3,6 cm)
SCHIJF_PREFIX = "schijf_"
SCHIJVEN_WIT = range(1, 21)   # A1-J4: wit (overkant, A1 links onder bij wit)
SCHIJVEN_ZWART = range(31, 51)# A7-J10: zwart (robotzijde, J10 rechts boven bij wit)

# --- Grijp- en beweginghoogtes ---
GRIPPER = True
GREEP_DIEPTE = 0.008          # diepte onder bovenkant schijf (8 mm)
OPNEEM_HOOGTE_EXTRA = -0.0005 # ~3,5 mm boven het bord bij vastpakken
LOSLAAT_HOOGTE_EXTRA = 0.010  # 1,0 cm hoger openen zodat het schijfje valt
HOVER_HOOGTE = 0.10           # 10 cm boven het bord bij verplaatsen
PAUZE = 0.6                   # seconden wachten bij grijpen/loslaten
AFLEG_POSITIE = (0.22, -0.28) # plek naast het bord voor geslagen schijven

# --- Veiligheidszone & gewrichtslimieten ---
ZONE_MARGE = 0.20
ZONE_MIN_BOVEN_BORD = 0.002
ZONE_MAX_BOVEN_BORD = 0.30
MIN_AFSTAND_BASIS = 0.15
GEWRICHT_LIMIETEN_GRADEN = {1: 100, 3: 180, 5: 180, 7: 180}
MAX_GEWRICHTSSPRONG_GRADEN = 120
MAX_KINEMATICA_AFWIJKING = 0.02
MAX_AFSTAND_SCHIJF_VAKJE = 0.03
# ============================================================

KOLOMMEN = "ABCDEFGHIJ"
ZONE_X_MIN = BORD_X_NABIJ - ZONE_MARGE
ZONE_X_MAX = BORD_X_NABIJ + BORD_BREEDTE + ZONE_MARGE
ZONE_Y_MIN = BORD_Y_MIDDEN - BORD_BREEDTE / 2.0 - ZONE_MARGE
ZONE_Y_MAX = BORD_Y_MIDDEN + BORD_BREEDTE / 2.0 + ZONE_MARGE
ZONE_Z_MIN = BORD_HOOGTE + ZONE_MIN_BOVEN_BORD
ZONE_Z_MAX = BORD_HOOGTE + ZONE_MAX_BOVEN_BORD


# ---------------------------------------------------------------------------
# Kinematica (fk, ik, klaar_q)
# ---------------------------------------------------------------------------
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
    tcp[:3, 3] = t[:3, 3] + t[:3, 2] * TCP_OFFSET_Z
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
    """Gewrichtshoeken van de veilige klaarstand (gripperpunt op KLAAR_TCP)."""
    global _KLAAR
    if _KLAAR is None:
        q = _ik_dls(KLAAR_TCP, np.zeros(7))
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


# ---------------------------------------------------------------------------
# Dambord-coördinaten (vakjes 1..50 <-> A1..J10 <-> x, y in meters)
# ---------------------------------------------------------------------------
def vakje_naar_naam(nummer):
    r = (nummer - 1) // 5
    c = ((nummer - 1) % 5) * 2 + (r % 2)
    return f"{KOLOMMEN[c]}{r + 1}"


def naam_naar_vakje(tekst):
    s = tekst.strip().upper()
    if len(s) < 2 or s[0] not in KOLOMMEN or not s[1:].isdigit():
        return None, f"Ongeldig vakje '{tekst}' (gebruik bijv. A7 of B4)."
    c = KOLOMMEN.index(s[0])
    rij = int(s[1:])
    if not 1 <= rij <= 10:
        return None, f"Rij {rij} bestaat niet (kies 1 t/m 10)."
    r = rij - 1
    if c % 2 != r % 2:
        return None, f"{s} is een wit vakje (dammen speelt alleen op de donkere vakjes)."
    return r * 5 + (c // 2) + 1, None


def vakje_positie(nummer):
    r = (nummer - 1) // 5
    c = ((nummer - 1) % 5) * 2 + (r % 2)
    x = BORD_X_NABIJ + (9.5 - r) * VAK
    y = BORD_Y_MIDDEN + (c - 4.5) * VAK
    return x, y


def dichtstbij(x, y):
    beste_nr, beste_afst = 1, float("inf")
    for nr in range(1, 51):
        vx, vy = vakje_positie(nr)
        afst = math.hypot(x - vx, y - vy)
        if afst < beste_afst:
            beste_nr, beste_afst = nr, afst
    return beste_nr, beste_afst


def schijf_kleur(object_id):
    try:
        nr = int(object_id.replace(SCHIJF_PREFIX, ""))
        if nr in SCHIJVEN_WIT:
            return "wit"
        if nr in SCHIJVEN_ZWART:
            return "zwart"
    except ValueError:
        pass
    return "onbekend"


# ---------------------------------------------------------------------------
# Verbinding: echte xArm7 (online) OF offline ROS-simulatie (RViz)
# ---------------------------------------------------------------------------
def _laad_ros2_omgeving():
    if shutil.which("ros2") is not None:
        return True
    cmd = (
        "source /opt/ros/*/setup.bash 2>/dev/null; "
        "source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null; env"
    )
    res = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
    if res.returncode == 0:
        for regel in res.stdout.splitlines():
            if "=" in regel:
                k, v = regel.split("=", 1)
                os.environ[k] = v
    return shutil.which("ros2") is not None


def _service_actief(naam):
    try:
        return naam in subprocess.check_output(["ros2", "service", "list"], text=True, timeout=5)
    except Exception:
        return False


def _topic_actief(naam):
    try:
        return naam in subprocess.check_output(["ros2", "topic", "list"], text=True, timeout=5).splitlines()
    except Exception:
        return False


def _robot_bereikbaar(ip, timeout=1.5):
    for poort in (502, 18333, 30001):
        try:
            with socket.create_connection((ip, poort), timeout=timeout):
                return True
        except OSError:
            continue
    return False


def verbind_met_arm(log):
    """
    Verbindt met de echte xArm7 als deze bereikbaar is, en schakelt anders
    automatisch over op de offline ROS-simulatie (RViz / MoveIt fake).
    Geeft (True, is_echte_arm) terug bij succes, anders (False, False).
    """
    if not _laad_ros2_omgeving():
        log.error("Kan ROS 2 niet vinden.")
        return False, False

    if _service_actief("/xarm/motion_enable") and _service_actief("/xarm_joint_plan"):
        log.info(f"Verbinding met echte xArm7 ({ROBOT_IP}) is al actief.")
        return True, True

    if _robot_bereikbaar(ROBOT_IP):
        if not AUTO_START_PLANNER:
            return True, True
        cmd = (
            f"source /opt/ros/*/setup.bash && "
            f"source /home/ubuntu/ros2_ws/install/setup.bash && "
            f"ros2 launch xarm_planner xarm7_planner_realmove.launch.py "
            f"robot_ip:={ROBOT_IP} add_gripper:=true gripper_version:=G2"
        )
        log.info(f"Echte xArm7 gevonden op {ROBOT_IP}: starten van realmove-planner...")
        subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
        for _ in range(35):
            time.sleep(1.0)
            if _service_actief("/xarm/motion_enable") and _service_actief("/xarm_joint_plan"):
                log.info("Verbinding met echte xArm7 geslaagd!")
                time.sleep(2.0)
                return True, True
        log.error("Timeout bij starten van xarm7_planner_realmove.")
        return False, False

    # Offline modus (geen fysieke robot bereikbaar -> testen in ROS / RViz)
    log.info(f"Geen fysieke xArm7 op {ROBOT_IP} -> offline ROS-modus (RViz/MoveIt).")
    if _service_actief("/xarm_joint_plan") and _service_actief("/xarm_exec_plan"):
        log.info("Offline ROS-planner is al actief.")
        return True, False

    if not AUTO_START_PLANNER:
        log.error("Offline ROS-planner (/xarm_joint_plan) draait nog niet.")
        return False, False

    cmd = (
        "source /opt/ros/*/setup.bash && "
        "source /home/ubuntu/ros2_ws/install/setup.bash && "
        "ros2 launch xarm_planner xarm7_planner_fake.launch.py "
        "add_gripper:=true gripper_version:=G2"
    )
    log.info("Starten van offline ROS-planner (xarm7_planner_fake, even geduld)...")
    subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
    for _ in range(35):
        time.sleep(1.0)
        if _service_actief("/xarm_joint_plan") and _service_actief("/xarm_exec_plan"):
            log.info("Offline ROS-planner gestart!")
            time.sleep(2.0)
            return True, False

    log.error("Timeout bij starten van xarm7_planner_fake.")
    return False, False


def activeer_echte_motoren(node):
    """Wist fouten en geeft de echte motoren en G2-gripper vrij (alleen indien online)."""
    log = node.get_logger()
    c_enable = node.create_client(SetInt16ById, "/xarm/motion_enable")
    if not c_enable.wait_for_service(timeout_sec=2.0):
        return True

    for srv in ("/xarm/clean_error", "/xarm/clean_warn"):
        c = node.create_client(Call, srv)
        if c.wait_for_service(timeout_sec=1.5):
            wacht_op(node, c.call_async(Call.Request()), srv, 5.0)

    req_en = SetInt16ById.Request()
    req_en.id = 8
    req_en.data = 1
    wacht_op(node, c_enable.call_async(req_en), "motion_enable", 10.0)

    for srv, val in (("/xarm/set_mode", 0), ("/xarm/set_state", 0), ("/xarm/set_gripper_enable", 1)):
        c = node.create_client(SetInt16, srv)
        if c.wait_for_service(timeout_sec=2.0):
            req = SetInt16.Request()
            req.data = val
            res = wacht_op(node, c.call_async(req), srv, 10.0)
            if srv == "/xarm/set_state" and res is not None and getattr(res, "ret", 0) != 0:
                log.error(f"xArm kon niet vrijgegeven worden (ret={res.ret}). Controleer de noodstop!")
                return False

    log.info("Motoren en G2-gripper zijn geactiveerd.")
    return True


# ---------------------------------------------------------------------------
# Veiligheid, Arm en Gripper
# ---------------------------------------------------------------------------
def controleer_zone(x, y, z):
    if not ZONE_X_MIN <= x <= ZONE_X_MAX:
        return f"x={x:.3f} buiten [{ZONE_X_MIN:.3f}, {ZONE_X_MAX:.3f}]"
    if not ZONE_Y_MIN <= y <= ZONE_Y_MAX:
        return f"y={y:.3f} buiten [{ZONE_Y_MIN:.3f}, {ZONE_Y_MAX:.3f}]"
    if not ZONE_Z_MIN <= z <= ZONE_Z_MAX:
        return f"z={z:.3f} buiten [{ZONE_Z_MIN:.3f}, {ZONE_Z_MAX:.3f}]"
    if math.hypot(x, y) < MIN_AFSTAND_BASIS:
        return f"te dicht bij robotbasis (< {MIN_AFSTAND_BASIS:.2f} m)"
    return None


def limiet_fout(q):
    for nr, limiet in GEWRICHT_LIMIETEN_GRADEN.items():
        graden = math.degrees(q[nr - 1])
        if abs(graden) > limiet:
            return f"joint{nr} = {graden:.0f} graden (limiet ±{limiet})"
    return None


class SetupFout(Exception):
    pass


def wacht_op(node, future, naam, seconden=30.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconden)
    if not future.done() or future.exception() is not None:
        node.get_logger().error(f"{naam}: mislukt of timeout.")
        return None
    return future.result()


class Arm:
    def __init__(self, node):
        self.node = node
        log = node.get_logger()

        self.joint_plan = node.create_client(PlanJoint, "/xarm_joint_plan")
        self.execute = node.create_client(PlanExec, "/xarm_exec_plan")
        self.servo = node.create_client(MoveJoint, "/xarm/set_servo_angle")

        if not self.joint_plan.wait_for_service(timeout_sec=10.0):
            raise SetupFout("/xarm_joint_plan niet gevonden.")

        self.gebruik_servo = self.servo.wait_for_service(timeout_sec=1.5)
        if not self.gebruik_servo and not self.execute.wait_for_service(timeout_sec=5.0):
            raise SetupFout("Noch /xarm/set_servo_angle noch /xarm_exec_plan gevonden.")

        self.gewrichten = {}
        topic = "/xarm/joint_states" if _topic_actief("/xarm/joint_states") else "/joint_states"
        node.create_subscription(JointState, topic, self._joint_cb, 10)
        self.geblokkeerd = False

        eind = time.time() + 10.0
        while self.huidige_q() is None:
            if time.time() > eind:
                raise SetupFout(f"Geen gewrichtsstand ontvangen op {topic}.")
        modus_txt = "echte robot" if self.gebruik_servo else "offline ROS-simulatie"
        log.info(f"Arm gekoppeld ({modus_txt}, gewrichten: {np.round(np.degrees(self.huidige_q()), 1)}°).")

    def _joint_cb(self, msg):
        for naam, pos in zip(msg.name, msg.position):
            self.gewrichten[naam] = pos

    def huidige_q(self):
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        q = []
        for i in range(1, 8):
            for naam, pos in self.gewrichten.items():
                if naam.endswith(f"joint{i}"):
                    q.append(pos)
                    break
            else:
                return None
        return np.array(q)

    def controleer_kinematica(self):
        global TCP_OFFSET_Z, _KLAAR
        log = self.node.get_logger()
        if Buffer is None:
            return True
        buf = Buffer()
        TransformListener(buf, self.node)
        for _ in range(30):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                tf = buf.lookup_transform(BASE_FRAME, EEF_FRAME, Time())
                break
            except Exception:
                continue
        else:
            return True

        gemeten = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        q_nu = self.huidige_q()
        verschil = float(np.linalg.norm(gemeten - fk(q_nu)[0][:3, 3]))
        if verschil > MAX_KINEMATICA_AFWIJKING:
            try:
                tf_flens = buf.lookup_transform("link7", EEF_FRAME, Time())
                TCP_OFFSET_Z = float(np.linalg.norm([
                    tf_flens.transform.translation.x,
                    tf_flens.transform.translation.y,
                    tf_flens.transform.translation.z,
                ]))
                _KLAAR = None
                verschil = float(np.linalg.norm(gemeten - fk(q_nu)[0][:3, 3]))
            except Exception:
                pass
        if verschil > MAX_KINEMATICA_AFWIJKING:
            log.error(f"Kinematica-afwijking te groot: {verschil * 100:.1f} cm")
            return False
        return True

    def gewrichten_ok(self):
        q = self.huidige_q()
        fout = limiet_fout(q) if q is not None else None
        if fout:
            self.node.get_logger().error(f"VEILIGHEID: {fout}.")
            return False
        return True

    def _voer_q_uit(self, q_doel, naam, alleen_uitvoeren=False):
        log = self.node.get_logger()
        if not alleen_uitvoeren:
            req = PlanJoint.Request()
            req.target = [float(v) for v in q_doel]
            res_p = wacht_op(self.node, self.joint_plan.call_async(req), f"plannen {naam}")
            if res_p is None or not res_p.success:
                log.error(f"Plannen mislukt: {naam}")
                return False

        if self.gebruik_servo:
            m_req = MoveJoint.Request()
            m_req.angles = [float(v) for v in q_doel]
            m_req.speed = 0.35
            m_req.acc = 2.0
            m_req.mvtime = 0.0
            m_req.wait = True
            m_req.timeout = 30.0
            res_m = wacht_op(self.node, self.servo.call_async(m_req), f"uitvoeren {naam}", 35.0)
            if res_m is None or getattr(res_m, "ret", 0) != 0:
                log.error(f"Uitvoeren mislukt: {naam}")
                activeer_echte_motoren(self.node)
                return False
            return True

        # Offline ROS-simulatie: voer gepland traject uit in RViz
        ex = PlanExec.Request()
        ex.wait = True
        res_e = wacht_op(self.node, self.execute.call_async(ex), f"uitvoeren {naam}", 35.0)
        if res_e is None or not res_e.success:
            log.error(f"Uitvoeren in RViz mislukt: {naam}")
            return False
        return True

    def eerst_omhoog_en_naar_klaarstand(self):
        q_nu = self.huidige_q()
        if q_nu is None:
            return False
        pos_nu = fk(q_nu)[0][:3, 3]
        if pos_nu[2] < KLAAR_TCP[2] - 0.02:
            q_omhoog = ik([pos_nu[0], pos_nu[1], KLAAR_TCP[2]], q_nu)
            if q_omhoog is not None and limiet_fout(q_omhoog) is None:
                if not self._voer_q_uit(q_omhoog, "veilig omhoog", alleen_uitvoeren=self.gebruik_servo):
                    return False
        return self._voer_q_uit(klaar_q(), "klaarstand") and self.gewrichten_ok()

    def beweeg(self, x, y, z, naam):
        log = self.node.get_logger()
        if self.geblokkeerd:
            return False
        reden = controleer_zone(x, y, z)
        if reden:
            log.error(f"GEWEIGERD ({naam}): {reden}")
            return False

        q_nu = self.huidige_q()
        q_doel = ik([x, y, z], q_nu)
        if q_doel is None or limiet_fout(q_doel):
            q_doel = ik([x, y, z], klaar_q())
        if q_doel is None or limiet_fout(q_doel):
            log.error(f"GEWEIGERD ({naam}): punt niet veilig bereikbaar.")
            return False

        if math.degrees(float(np.max(np.abs(q_doel - q_nu)))) > MAX_GEWRICHTSSPRONG_GRADEN:
            log.error(f"GEWEIGERD ({naam}): gewrichtssprong te groot.")
            return False

        if not self._voer_q_uit(q_doel, naam):
            return False
        if not self.gewrichten_ok():
            self.geblokkeerd = True
            return False
        return True


class Gripper:
    """Opent en sluit de G2-gripper op zowel de echte robot als offline in ROS/RViz."""

    def __init__(self, node):
        self.node = node
        self.modus = None
        if not GRIPPER:
            return

        self.servo_client = node.create_client(GripperMove, "/xarm/set_gripper_position")
        if self.servo_client.wait_for_service(timeout_sec=1.5):
            self.modus = "service"
            node.get_logger().info("Echte G2-gripper gekoppeld.")
            return

        self.grip_plan = node.create_client(PlanJoint, "/xarm_gripper_joint_plan")
        self.grip_exec = node.create_client(PlanExec, "/xarm_gripper_exec_plan")
        if self.grip_plan.wait_for_service(timeout_sec=2.0) and self.grip_exec.wait_for_service(timeout_sec=2.0):
            self.modus = "planner"
            node.get_logger().info("Offline G2-gripper gekoppeld via MoveIt planner.")
            return

        self.traj_client = ActionClient(
            node, FollowJointTrajectory, "/xarm_gripper_traj_controller/follow_joint_trajectory"
        )
        if self.traj_client.wait_for_server(timeout_sec=1.5):
            self.modus = "traj"
            node.get_logger().info("Offline G2-gripper gekoppeld via trajectory controller.")
            return

        node.get_logger().info("Geen aparte gripper-controller gevonden (wordt overgeslagen).")

    def open(self):
        return self._stuur(echte_mm=850.0, sim_rad=0.0, naam="open")

    def dicht(self):
        return self._stuur(echte_mm=0.0, sim_rad=0.85, naam="dicht")

    def _stuur(self, echte_mm, sim_rad, naam):
        if self.modus is None:
            return True

        if self.modus == "service":
            req = GripperMove.Request()
            req.pos = float(echte_mm)
            req.wait = True
            req.timeout = 5.0
            res = wacht_op(self.node, self.servo_client.call_async(req), f"gripper {naam}", 10.0)
            return res is not None and getattr(res, "ret", 0) == 0

        if self.modus == "planner":
            req = PlanJoint.Request()
            req.target = [float(sim_rad)]
            res_p = wacht_op(self.node, self.grip_plan.call_async(req), f"gripper plannen {naam}", 10.0)
            if res_p is None or not res_p.success:
                return True
            ex = PlanExec.Request()
            ex.wait = True
            wacht_op(self.node, self.grip_exec.call_async(ex), f"gripper uitvoeren {naam}", 10.0)
            return True

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["drive_joint"]
        punt = JointTrajectoryPoint()
        punt.positions = [float(sim_rad)]
        punt.time_from_start.nanosec = 400_000_000
        goal.trajectory.points = [punt]
        fut = self.traj_client.send_goal_async(goal)
        gh = wacht_op(self.node, fut, f"gripper {naam}", 5.0)
        if gh is not None and gh.accepted:
            wacht_op(self.node, gh.get_result_async(), f"gripper {naam}", 5.0)
        return True


# ---------------------------------------------------------------------------
# MoveIt Planning Scene & Schijven (zichtbaar in RViz, online én offline)
# ---------------------------------------------------------------------------
def _laad_bord_stl():
    if not os.path.isfile(BORD_STL_PAD):
        return None
    driehoeken = []
    with open(BORD_STL_PAD, "rb") as f:
        data = f.read()

    if len(data) >= 84 and (84 + struct.unpack("<I", data[80:84])[0] * 50 == len(data)):
        aantal = struct.unpack("<I", data[80:84])[0]
        s = BORD_STL_SCHAAL
        for i in range(aantal):
            fl = struct.unpack("<12f", data[84 + i * 50 : 84 + i * 50 + 48])
            driehoeken.append(((fl[3]*s, fl[4]*s, fl[5]*s), (fl[6]*s, fl[7]*s, fl[8]*s), (fl[9]*s, fl[10]*s, fl[11]*s)))
    else:
        huidige = []
        s = BORD_STL_SCHAAL
        for regel in data.decode("utf-8", errors="ignore").splitlines():
            delen = regel.strip().split()
            if len(delen) == 4 and delen[0].lower() == "vertex":
                huidige.append((float(delen[1])*s, float(delen[2])*s, float(delen[3])*s))
                if len(huidige) == 3:
                    driehoeken.append(tuple(huidige))
                    huidige = []

    if not driehoeken:
        return None

    alle = [p for tri in driehoeken for p in tri]
    xs, ys, zs = [p[0] for p in alle], [p[1] for p in alle], [p[2] for p in alle]
    dx = (BORD_X_NABIJ - BORD_RAND) - min(xs)
    dy = BORD_Y_MIDDEN - (min(ys) + max(ys)) / 2.0
    dz = (BORD_HOOGTE - 0.004) - max(zs)

    mesh = Mesh()
    index = {}
    for v1, v2, v3 in driehoeken:
        idxs = []
        for vx, vy, vz in (v1, v2, v3):
            key = (round(vx + dx, 6), round(vy + dy, 6), round(vz + dz, 6))
            if key not in index:
                index[key] = len(mesh.vertices)
                mesh.vertices.append(Point(x=key[0], y=key[1], z=key[2]))
            idxs.append(index[key])
        mesh.triangles.append(MeshTriangle(vertex_indices=idxs))
    return mesh


KLEUR_BORD = (0.82, 0.68, 0.47, 1.0)        # licht hout (bord + lichte vakjes)
KLEUR_VAK_DONKER = (0.36, 0.22, 0.12, 1.0)  # donkerbruin (50 speelvakjes)
KLEUR_ZWART = (0.12, 0.12, 0.14, 1.0)       # zwarte schijven (A1-J4)
KLEUR_WIT = (0.95, 0.94, 0.90, 1.0)         # witte schijven (A7-J10)


def maak_object_kleur(object_id, rgba):
    oc = ObjectColor()
    oc.id = object_id
    oc.color = ColorRGBA(
        r=float(rgba[0]),
        g=float(rgba[1]),
        b=float(rgba[2]),
        a=float(rgba[3]),
    )
    return oc


def kleur_voor_schijf(object_id):
    rgba = KLEUR_ZWART if schijf_kleur(object_id) == "zwart" else KLEUR_WIT
    return maak_object_kleur(object_id, rgba)


def maak_schijf_object(object_id, x, y):
    obj = CollisionObject()
    obj.header.frame_id = BASE_FRAME
    obj.id = object_id
    obj.operation = CollisionObject.ADD

    cil = SolidPrimitive()
    cil.type = SolidPrimitive.CYLINDER
    cil.dimensions = [SCHIJF_HOOGTE, SCHIJF_DIAMETER / 2.0]

    obj.pose.position.x = float(x)
    obj.pose.position.y = float(y)
    obj.pose.position.z = float(BORD_HOOGTE + SCHIJF_HOOGTE / 2.0)
    obj.pose.orientation.w = 1.0

    prim_pose = Pose()
    prim_pose.orientation.w = 1.0
    obj.primitives = [cil]
    obj.primitive_poses = [prim_pose]
    return obj


def maak_donker_vak_object(nr):
    x, y = vakje_positie(nr)
    obj = CollisionObject()
    obj.header.frame_id = BASE_FRAME
    obj.id = f"vak_{nr}"
    obj.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [VAK, VAK, 0.0004]

    obj.pose.position.x = float(x)
    obj.pose.position.y = float(y)
    obj.pose.position.z = float(BORD_HOOGTE - 0.0037)
    obj.pose.orientation.w = 1.0

    prim_pose = Pose()
    prim_pose.orientation.w = 1.0
    obj.primitives = [box]
    obj.primitive_poses = [prim_pose]
    return obj


def plaats_bord_en_schijven(scene, log):
    """Plaatst het gekleurde dambord, 50 vakjes en 40 schijven in MoveIt/RViz."""
    huidige = scene.lees(
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    )
    weg_wereld, weg_vast = [], []
    if huidige:
        for obj in huidige.world.collision_objects:
            if obj.id == "bord" or obj.id.startswith(("vak_", SCHIJF_PREFIX)):
                c = CollisionObject()
                c.header.frame_id = BASE_FRAME
                c.id = obj.id
                c.operation = CollisionObject.REMOVE
                weg_wereld.append(c)
        for att in huidige.robot_state.attached_collision_objects:
            a = AttachedCollisionObject()
            a.link_name = EEF_FRAME
            a.object.header.frame_id = BASE_FRAME
            a.object.id = att.object.id
            a.object.operation = CollisionObject.REMOVE
            weg_vast.append(a)

    bord = CollisionObject()
    bord.header.frame_id = BASE_FRAME
    bord.id = "bord"
    bord.operation = CollisionObject.ADD

    mesh = _laad_bord_stl()
    if mesh is not None:
        m_pose = Pose()
        m_pose.orientation.w = 1.0
        bord.pose.orientation.w = 1.0
        bord.meshes = [mesh]
        bord.mesh_poses = [m_pose]
    else:
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [BORD_BREEDTE + 2 * BORD_RAND, BORD_BREEDTE + 2 * BORD_RAND, 0.005]
        bord.pose.position.x = BORD_X_NABIJ + BORD_BREEDTE / 2.0
        bord.pose.position.y = BORD_Y_MIDDEN
        bord.pose.position.z = BORD_HOOGTE - 0.0065
        bord.pose.orientation.w = 1.0
        b_pose = Pose()
        b_pose.orientation.w = 1.0
        bord.primitives = [box]
        bord.primitive_poses = [b_pose]

    nieuw = [bord]
    kleuren = [maak_object_kleur("bord", KLEUR_BORD)]

    for nr in range(1, 51):
        nieuw.append(maak_donker_vak_object(nr))
        kleuren.append(maak_object_kleur(f"vak_{nr}", KLEUR_VAK_DONKER))

    for nr in list(SCHIJVEN_WIT) + list(SCHIJVEN_ZWART):
        x, y = vakje_positie(nr)
        obj_id = f"{SCHIJF_PREFIX}{nr}"
        nieuw.append(maak_schijf_object(obj_id, x, y))
        kleuren.append(kleur_voor_schijf(obj_id))

    ps = PlanningScene()
    ps.world.collision_objects = weg_wereld + nieuw
    ps.robot_state.attached_collision_objects = weg_vast
    ps.object_colors = kleuren
    if not scene.pas_toe(ps):
        log.error("Kon het bord en de 40 schijven niet toevoegen aan MoveIt/RViz.")
        return False

    time.sleep(0.2)
    log.info("Gekleurd bord en 40 schijven in MoveIt/RViz geplaatst (wit: A1-J4, zwart: A7-J10).")
    return True


class Scene:
    def __init__(self, node):
        self.node = node
        self.get = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.beschikbaar = (
            self.get.wait_for_service(timeout_sec=5.0)
            and self.apply.wait_for_service(timeout_sec=5.0)
        )

    def lees(self, onderdelen):
        req = GetPlanningScene.Request()
        req.components.components = onderdelen
        res = wacht_op(self.node, self.get.call_async(req), "/get_planning_scene", 20.0)
        return res.scene if res is not None else None

    def pas_toe(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        req = ApplyPlanningScene.Request()
        req.scene = scene
        res = wacht_op(self.node, self.apply.call_async(req), "/apply_planning_scene", 20.0)
        return res is not None and res.success


class Schijven:
    """Houdt alle 40 schijven bij in Python én synchroniseert ze met RViz/MoveIt."""

    def __init__(self, scene, log):
        self.scene = scene
        self.log = log
        self.bezet = {}
        self.vast_id = None
        self.vast_info = None
        self.beurt = "wit"
        self.geslagen_teller = 0
        self.actief = True

        if self.scene.beschikbaar and self.ververs() and len(self.bezet) > 0:
            self.log.info(f"{len(self.bezet)} schijven overgenomen uit de lopende ROS-scene.")
        else:
            self.reset_bord()

    def reset_bord(self):
        if self.scene.beschikbaar:
            plaats_bord_en_schijven(self.scene, self.log)
        self.bezet = {}
        for nr in list(SCHIJVEN_WIT) + list(SCHIJVEN_ZWART):
            x, y = vakje_positie(nr)
            obj_id = f"{SCHIJF_PREFIX}{nr}"
            self.bezet[nr] = {
                "id": obj_id,
                "kleur": schijf_kleur(obj_id),
                "x": x,
                "y": y,
            }
        self.beurt = "wit"
        self.geslagen_teller = 0
        self.vast_id = None
        self.vast_info = None

    def ververs(self):
        if not self.scene.beschikbaar:
            return False
        scene = self.scene.lees(
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        if scene is None:
            return False

        gevonden = {}
        for obj in scene.world.collision_objects:
            if not obj.id.startswith(SCHIJF_PREFIX):
                continue
            px = obj.pose.position.x + (obj.primitive_poses[0].position.x if obj.primitive_poses else 0.0)
            py = obj.pose.position.y + (obj.primitive_poses[0].position.y if obj.primitive_poses else 0.0)
            vakje, afstand = dichtstbij(px, py)
            if afstand <= MAX_AFSTAND_SCHIJF_VAKJE:
                gevonden[vakje] = {
                    "id": obj.id,
                    "kleur": schijf_kleur(obj.id),
                    "x": px,
                    "y": py,
                }
        if gevonden:
            self.bezet = gevonden
        return True

    def _verwijder_in_rviz(self, object_id):
        if not self.scene.beschikbaar:
            return
        weg = CollisionObject()
        weg.header.frame_id = BASE_FRAME
        weg.id = object_id
        weg.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.world.collision_objects = [weg]
        self.scene.pas_toe(ps)

    def _plaats_in_rviz(self, object_id, x, y):
        if not self.scene.beschikbaar:
            return
        ps = PlanningScene()
        ps.world.collision_objects = [maak_schijf_object(object_id, x, y)]
        ps.object_colors = [kleur_voor_schijf(object_id)]
        self.scene.pas_toe(ps)

    def bereid_oppakken_voor(self, vakje):
        if vakje not in self.bezet:
            return False
        # Haal de schijf vlak voor het zakken uit RViz-botsingen zodat de grijper eromheen kan zakken
        self._verwijder_in_rviz(self.bezet[vakje]["id"])
        return True

    def herstel_vakje(self, vakje):
        info = self.bezet.get(vakje) or self.vast_info
        if info:
            self.bezet[vakje] = info
            self._plaats_in_rviz(info["id"], info["x"], info["y"])
        self.vast_id = None
        self.vast_info = None

    def vastmaken(self, object_id, vakje):
        self.vast_id = object_id
        self.vast_info = self.bezet.pop(vakje, None)
        self.log.info(f"Schijf {object_id} ({schijf_kleur(object_id)}) van {vakje_naar_naam(vakje)} opgepakt.")
        return True

    def loslaten(self, label_doel, doel_pos):
        object_id = self.vast_id
        nr, afstand = dichtstbij(doel_pos[0], doel_pos[1])
        if afstand <= MAX_AFSTAND_SCHIJF_VAKJE:
            self.bezet[nr] = {
                "id": object_id,
                "kleur": schijf_kleur(object_id),
                "x": doel_pos[0],
                "y": doel_pos[1],
            }
        # Zet de schijf pas weer in RViz nadat de arm omhoog is gegaan
        self._plaats_in_rviz(object_id, doel_pos[0], doel_pos[1])
        self.vast_id = None
        self.vast_info = None
        self.log.info(f"Schijf {object_id} losgelaten op {label_doel}.")
        return True

    def wissel_beurt(self):
        self.beurt = "zwart" if self.beurt == "wit" else "wit"

    def overzicht(self):
        wit = [vakje_naar_naam(v) for v, i in sorted(self.bezet.items()) if i["kleur"] == "wit"]
        zwart = [vakje_naar_naam(v) for v, i in sorted(self.bezet.items()) if i["kleur"] == "zwart"]
        return (
            f"Aan de beurt: {self.beurt.upper()}\n"
            f"  Wit   ({len(wit)} - overkant):   {', '.join(wit)}\n"
            f"  Zwart ({len(zwart)} - robotzijde): {', '.join(zwart)}"
        )


# ---------------------------------------------------------------------------
# Bewegingen en spelregels
# ---------------------------------------------------------------------------
def _opneem_z():
    return max(ZONE_Z_MIN, BORD_HOOGTE + SCHIJF_HOOGTE - GREEP_DIEPTE + OPNEEM_HOOGTE_EXTRA)


def bezoek_punt(arm, label, x, y, z_doel, voor_omlaag=None, tussenactie=None, na_omhoog=None):
    z_hover = max(BORD_HOOGTE + HOVER_HOOGTE, z_doel + 0.05)
    if not arm.beweeg(x, y, z_hover, f"boven {label}"):
        return False
    if voor_omlaag and not voor_omlaag():
        return False
    if not arm.beweeg(x, y, z_doel, f"omlaag {label}"):
        return False
    ok = tussenactie() if tussenactie else True
    time.sleep(PAUZE)
    if not arm.beweeg(x, y, z_hover, f"omhoog {label}") or not ok:
        return False
    return na_omhoog() if na_omhoog else True


def pak_schijf(arm, gripper, schijven, vakje):
    naam = vakje_naar_naam(vakje)
    info = schijven.bezet[vakje]
    ok = bezoek_punt(
        arm, naam, info["x"], info["y"], _opneem_z(),
        voor_omlaag=lambda: gripper.open() and schijven.bereid_oppakken_voor(vakje),
        tussenactie=lambda: gripper.dicht() and schijven.vastmaken(info["id"], vakje),
    )
    if not ok:
        schijven.herstel_vakje(vakje)
    return ok


def plaats_schijf(arm, gripper, schijven, label, x, y):
    return bezoek_punt(
        arm, label, x, y, _opneem_z() + LOSLAAT_HOOGTE_EXTRA,
        tussenactie=gripper.open,
        na_omhoog=lambda: schijven.loslaten(label, (x, y)),
    )


def vind_geslagen_schijven(schijven, van_nr, naar_nr, eigen_kleur):
    x1, y1 = vakje_positie(van_nr)
    x2, y2 = vakje_positie(naar_nr)
    stappen_x = round(abs(x2 - x1) / VAK)
    stappen_y = round(abs(y2 - y1) / VAK)
    if stappen_x != stappen_y or stappen_x < 2:
        return []

    geslagen = []
    for stap in range(1, stappen_x):
        tx = x1 + ((x2 - x1) / stappen_x) * stap
        ty = y1 + ((y2 - y1) / stappen_y) * stap
        nr, afstand = dichtstbij(tx, ty)
        if afstand <= 0.015 and nr in schijven.bezet and schijven.bezet[nr]["kleur"] != eigen_kleur:
            geslagen.append(nr)
    return geslagen


def verwijder_geslagen_schijf(arm, gripper, schijven, vakje):
    if vakje not in schijven.bezet:
        print(f"Op {vakje_naar_naam(vakje)} ligt geen schijf.")
        return False
    if not pak_schijf(arm, gripper, schijven, vakje):
        return False
    ax, ay = AFLEG_POSITIE
    ay_offset = ay + (schijven.geslagen_teller % 5) * 0.04
    schijven.geslagen_teller += 1
    return plaats_schijf(arm, gripper, schijven, "aflegplek", ax, ay_offset)


def voer_zetreeks_uit(arm, gripper, schijven, vakjes):
    geslagen_namen = []
    for i in range(len(vakjes) - 1):
        van_nr, naar_nr = vakjes[i], vakjes[i + 1]
        if van_nr not in schijven.bezet:
            print(f"Op {vakje_naar_naam(van_nr)} ligt geen schijf.")
            return
        kleur = schijven.bezet[van_nr]["kleur"]
        if kleur != schijven.beurt:
            print(f"Op {vakje_naar_naam(van_nr)} ligt {kleur}, maar {schijven.beurt.upper()} is aan zet (typ 'w').")
            return
        if naar_nr in schijven.bezet:
            print(f"Vakje {vakje_naar_naam(naar_nr)} is bezet.")
            return

        geslagen = vind_geslagen_schijven(schijven, van_nr, naar_nr, kleur)
        x_doel, y_doel = vakje_positie(naar_nr)
        if not pak_schijf(arm, gripper, schijven, van_nr):
            return
        if not plaats_schijf(arm, gripper, schijven, vakje_naar_naam(naar_nr), x_doel, y_doel):
            return

        for g_nr in geslagen:
            if not verwijder_geslagen_schijf(arm, gripper, schijven, g_nr):
                return
            geslagen_namen.append(vakje_naar_naam(g_nr))

    schijven.wissel_beurt()
    extra = f" (geslagen: {', '.join(geslagen_namen)})" if geslagen_namen else ""
    print(f"Zet voltooid{extra}! Nu is {schijven.beurt.upper()} aan de beurt.")


def calibratie(arm):
    for nr in (1, 5, 46, 50):
        x, y = vakje_positie(nr)
        if not arm.beweeg(x, y, BORD_HOOGTE + HOVER_HOOGTE, f"calibratie {vakje_naar_naam(nr)}"):
            return
        time.sleep(1.5)


def main():
    rclpy.init()
    node = rclpy.create_node("dambord_move")
    log = node.get_logger()

    try:
        ok, is_echt = verbind_met_arm(log)
        if not ok:
            return
        if is_echt and not activeer_echte_motoren(node):
            return

        arm = Arm(node)
        if not arm.controleer_kinematica():
            return

        scene = Scene(node)
        schijven = Schijven(scene, log)

        q_start = arm.huidige_q()
        pos_start = fk(q_start)[0][:3, 3] if q_start is not None else np.zeros(3)
        if not arm.gewrichten_ok() or pos_start[2] < BORD_HOOGTE + HOVER_HOOGTE - 0.01:
            # Maak eventuele schijf onder de startpositie vrij zodat opstijgen nooit botst
            nr_onder, afst_onder = dichtstbij(pos_start[0], pos_start[1])
            tijdelijk = schijven.bezet.get(nr_onder) if afst_onder < 0.04 else None
            if tijdelijk:
                schijven._verwijder_in_rviz(tijdelijk["id"])
            if not arm.eerst_omhoog_en_naar_klaarstand():
                return
            if tijdelijk:
                schijven._plaats_in_rviz(tijdelijk["id"], tijdelijk["x"], tijdelijk["y"])

        gripper = Gripper(node)
        print("\n" + schijven.overzicht())
        gripper.open()

        while True:
            invoer = input(
                f"\n[{schijven.beurt.upper()}] Zet (bijv. C3-D4 of slag C3-E5), 'x B4' = slaan, "
                "'w' = wissel kleur, 'l' = overzicht, 'r' = reset bord, 'o'/'d' = gripper, 'c' = cal, 'q' = stop: "
            ).strip()
            cmd = invoer.lower()

            if cmd == "q":
                break
            if cmd == "l":
                schijven.ververs()
                print(schijven.overzicht())
                continue
            if cmd == "r":
                schijven.reset_bord()
                print(schijven.overzicht())
                continue
            if cmd == "w":
                schijven.wissel_beurt()
                print(f"Nu is {schijven.beurt.upper()} aan de beurt.")
                continue
            if cmd == "o":
                gripper.open()
                continue
            if cmd == "d":
                gripper.dicht()
                continue
            if cmd == "c":
                calibratie(arm)
                continue
            if cmd.startswith("x "):
                nr, fout = naam_naar_vakje(invoer[2:].strip())
                if fout:
                    print(fout)
                else:
                    verwijder_geslagen_schijf(arm, gripper, schijven, nr)
                continue

            delen = [d for d in re.split(r"[-xX\s]+", invoer) if d]
            if len(delen) >= 2:
                vakjes = []
                for d in delen:
                    nr, fout = naam_naar_vakje(d)
                    if fout:
                        print(fout)
                        break
                    vakjes.append(nr)
                else:
                    voer_zetreeks_uit(arm, gripper, schijven, vakjes)
                continue

            nr, fout = naam_naar_vakje(invoer)
            if fout:
                print(fout)
            elif schijven.vast_id is None and nr in schijven.bezet:
                pak_schijf(arm, gripper, schijven, nr)
            elif schijven.vast_id is not None and nr not in schijven.bezet:
                x, y = vakje_positie(nr)
                if plaats_schijf(arm, gripper, schijven, vakje_naar_naam(nr), x, y):
                    schijven.wissel_beurt()
    except SetupFout as e:
        log.error(str(e))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
