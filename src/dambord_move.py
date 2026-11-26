#!/usr/bin/env python3
"""
Beweeg de xArm7 met G2-gripper over een 10x10 dambord en verplaats damschijven.

- Schakelt alleen over van de container-simulatie (fake) naar de echte xArm7
  als de echte driver nog niet draait; behoudt anders de lopende sessie + scene.
- Pakt schijven iets hoger op en laat ze vanaf ~1 cm hoogte op het doelvakje vallen.
- Neemt bij een slagzet (bijv. B4-D6 of B4xD6) automatisch de geslagen schijf weg
  en legt deze naast het bord.
"""

import os
import re
import shutil
import socket
import subprocess
import sys
import time

# Zorg dat rclpy en xarm_msgs ook werken als de container net vers is opgestart
try:
    import rclpy
except ImportError:
    _cmd = (
        "source /opt/ros/*/setup.bash 2>/dev/null; "
        "source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null; "
        f"exec {sys.executable} {' '.join(sys.argv)}"
    )
    os.execvp("bash", ["bash", "-c", _cmd])

import math
import numpy as np
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from xarm_msgs.srv import Call, GripperMove, MoveJoint, PlanExec, PlanJoint, SetFloat32, SetInt16, SetInt16ById

import dambord_config as cfg
import dambord_kin as kin
from dambord_kin import fk, ik, klaar_q

try:
    from tf2_ros import Buffer, TransformListener
except ImportError:
    Buffer = None

# ======================= INSTELLINGEN =======================
HOVER_HOOGTE = 0.10          # hoogte van het gripperpunt boven de bovenkant van het bord (m)
PICK_HOOGTE = 0.02           # alleen zonder schijven: hoogte boven het bord bij "omlaag"
OPNEEM_HOOGTE_EXTRA = 0.003  # 3 mm hoger vastpakken zodat de grijper het bord niet raakt
LOSLAAT_HOOGTE_EXTRA = 0.010 # 1,0 cm hoger loslaten zodat het schijfje op het vakje valt
PAUZE = 0.6                  # seconden wachten bij grijpen/loslaten
MAX_AFSTAND_SCHIJF_VAKJE = 0.03   # schijf telt als "op vakje" binnen 3 cm

# --- Veiligheidszone ---
ZONE_MARGE = 0.20            # 20 cm buiten het speelveld
ZONE_MIN_BOVEN_BORD = 0.002  # gripperpunt nooit lager dan 2 mm boven het bord
ZONE_MAX_BOVEN_BORD = 0.30   # gripperpunt nooit hoger dan 30 cm boven het bord
MIN_AFSTAND_BASIS = 0.15     # minimale horizontale afstand tot de robotbasis (m)

# --- Gewrichten ---
GEWRICHT_LIMIETEN_GRADEN = {
    1: 100,   # basisdraaiing: ruim genoeg voor het hele bord
    3: 180,   # bovenarm-rotatie: maximaal een halve slag per kant
    5: 180,   # onderarm-rotatie
    7: 180,   # pols: maximaal een halve slag per kant
}
MAX_GEWRICHTSSPRONG_GRADEN = 120   # grootste toegestane verandering per gewricht per beweging

# --- Kinematica-controle via TF ---
BASE_FRAME = cfg.BASE_FRAME
EEF_FRAME = cfg.EEF_FRAME
MAX_KINEMATICA_AFWIJKING = 0.02    # m
# ============================================================

BORD_X0 = cfg.BORD_X_NABIJ
BORD_X1 = cfg.BORD_X_NABIJ + cfg.BORD_BREEDTE
BORD_Y0 = cfg.BORD_Y_MIDDEN - cfg.BORD_BREEDTE / 2
BORD_Y1 = cfg.BORD_Y_MIDDEN + cfg.BORD_BREEDTE / 2

ZONE_X_MIN = BORD_X0 - ZONE_MARGE
ZONE_X_MAX = BORD_X1 + ZONE_MARGE
ZONE_Y_MIN = BORD_Y0 - ZONE_MARGE
ZONE_Y_MAX = BORD_Y1 + ZONE_MARGE
ZONE_Z_MIN = cfg.BORD_HOOGTE + ZONE_MIN_BOVEN_BORD
ZONE_Z_MAX = cfg.BORD_HOOGTE + ZONE_MAX_BOVEN_BORD


def schijf_kleur(object_id):
    """Bepaalt 'wit' of 'zwart' op basis van het startnummer in schijf_<nr>."""
    try:
        start_nr = int(object_id.replace(cfg.SCHIJF_PREFIX, ""))
        if start_nr in cfg.SCHIJVEN_A:
            return "wit"
        if start_nr in cfg.SCHIJVEN_B:
            return "zwart"
    except ValueError:
        pass
    return "onbekend"


# ---------------------------------------------------------------------------
# Automatische verbinding en activatie van de echte xArm7
# ---------------------------------------------------------------------------
def _laad_ros2_omgeving():
    """Zorgt dat het 'ros2' CLI-commando ook beschikbaar is in een verse container."""
    if shutil.which("ros2") is not None:
        return True

    cmd = (
        "source /opt/ros/*/setup.bash 2>/dev/null; "
        "source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null; "
        "env"
    )
    res = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
    if res.returncode == 0:
        for regel in res.stdout.splitlines():
            if "=" in regel:
                k, v = regel.split("=", 1)
                os.environ[k] = v
    return shutil.which("ros2") is not None


def _service_actief(naam):
    """Controleert snel of een ROS 2 service al bestaat."""
    try:
        uitvoer = subprocess.check_output(["ros2", "service", "list"], text=True, timeout=5)
        return naam in uitvoer
    except Exception:
        return False


def _topic_actief(naam):
    """Controleert snel of een ROS 2 topic bestaat."""
    try:
        uitvoer = subprocess.check_output(["ros2", "topic", "list"], text=True, timeout=5)
        return naam in uitvoer.splitlines()
    except Exception:
        return False


def _echte_planner_draait():
    """Controleert of de echte driver én planner services al draaien."""
    return (
        _service_actief("/xarm/motion_enable")
        and _service_actief("/xarm_joint_plan")
        and _service_actief("/xarm_exec_plan")
    )


def _robot_bereikbaar(ip, timeout=2.0):
    """Controleert via een directe TCP-verbinding of de xArm-controller online is."""
    for poort in (502, 18333, 30001):
        try:
            with socket.create_connection((ip, poort), timeout=timeout):
                return True
        except OSError:
            continue
    return False


def verbind_met_echte_arm(log):
    """
    Start xarm7_planner_realmove.launch.py met gripper_version:=G2 als deze nog niet draait.
    Stopt NOOIT bestaande processen (anders sluit PID 1 de Docker-container af).
    """
    if not _laad_ros2_omgeving():
        log.error("Kan ROS 2 niet vinden in deze container (/opt/ros/...).")
        return False

    if not getattr(cfg, "AUTO_START_REALMOVE", False):
        return True

    if _echte_planner_draait():
        log.info(f"Verbinding met echte xArm7 ({cfg.ROBOT_IP}) is al actief.")
        return True

    log.info(f"Verbinden met echte xArm7 op {cfg.ROBOT_IP}...")
    if not _robot_bereikbaar(cfg.ROBOT_IP):
        log.error(
            f"Geen antwoord van de echte arm op {cfg.ROBOT_IP}. "
            "Controleer of de robotbox aanstaat en de netwerkkabel is aangesloten."
        )
        return False

    cmd = (
        f"source /opt/ros/*/setup.bash && "
        f"source /home/ubuntu/ros2_ws/install/setup.bash && "
        f"ros2 launch xarm_planner xarm7_planner_realmove.launch.py "
        f"robot_ip:={cfg.ROBOT_IP} add_gripper:=true gripper_version:=G2"
    )
    log.info("Starten van xarm7_planner_realmove (echte arm + G2 gripper, even geduld)...")
    subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    for _ in range(35):
        time.sleep(1.0)
        if _echte_planner_draait():
            log.info("Verbinding met echte xArm7 + planner geslaagd!")
            time.sleep(2.0)
            return True

    log.error("Timeout: xarm7_planner_realmove kwam niet binnen 35 seconden online.")
    return False


def activeer_echte_motoren(node):
    """Wist fouten, zet de xArm in position-modus (mode 0) en activeert arm + gripper."""
    log = node.get_logger()
    c_enable = node.create_client(SetInt16ById, "/xarm/motion_enable")
    if not c_enable.wait_for_service(timeout_sec=3.0):
        return True

    c_clean_err = node.create_client(Call, "/xarm/clean_error")
    c_clean_warn = node.create_client(Call, "/xarm/clean_warn")
    c_mode = node.create_client(SetInt16, "/xarm/set_mode")
    c_state = node.create_client(SetInt16, "/xarm/set_state")
    c_grip_en = node.create_client(SetInt16, "/xarm/set_gripper_enable")
    c_coll_sens = node.create_client(SetInt16, "/xarm/set_collision_sensitivity")

    log.info("Echte xArm gedetecteerd: fouten wissen, motoren & gripper vrijgeven...")

    if c_clean_err.wait_for_service(timeout_sec=2.0):
        wacht_op(node, c_clean_err.call_async(Call.Request()), "clean_error", 5.0)
    if c_clean_warn.wait_for_service(timeout_sec=2.0):
        wacht_op(node, c_clean_warn.call_async(Call.Request()), "clean_warn", 5.0)

    if c_coll_sens.wait_for_service(timeout_sec=1.0):
        req_cs = SetInt16.Request()
        req_cs.data = 2
        wacht_op(node, c_coll_sens.call_async(req_cs), "set_collision_sensitivity", 5.0)

    req_en = SetInt16ById.Request()
    req_en.id = 8
    req_en.data = 1
    wacht_op(node, c_enable.call_async(req_en), "motion_enable", 10.0)

    c_mode.wait_for_service(timeout_sec=3.0)
    req_mode = SetInt16.Request()
    req_mode.data = 0
    wacht_op(node, c_mode.call_async(req_mode), "set_mode(0)", 10.0)

    c_state.wait_for_service(timeout_sec=3.0)
    req_state = SetInt16.Request()
    req_state.data = 0
    res_st = wacht_op(node, c_state.call_async(req_state), "set_state(0)", 10.0)

    if c_grip_en.wait_for_service(timeout_sec=2.0):
        req_ge = SetInt16.Request()
        req_ge.data = 1
        wacht_op(node, c_grip_en.call_async(req_ge), "set_gripper_enable(1)", 5.0)
        c_grip_mode = node.create_client(SetInt16, "/xarm/set_gripper_mode")
        if c_grip_mode.wait_for_service(timeout_sec=1.0):
            req_gm = SetInt16.Request()
            req_gm.data = 0
            wacht_op(node, c_grip_mode.call_async(req_gm), "set_gripper_mode(0)", 5.0)
        c_grip_spd = node.create_client(SetFloat32, "/xarm/set_gripper_speed")
        if c_grip_spd.wait_for_service(timeout_sec=1.0):
            req_gs = SetFloat32.Request()
            req_gs.data = 2000.0
            wacht_op(node, c_grip_spd.call_async(req_gs), "set_gripper_speed", 5.0)

    time.sleep(0.5)
    if res_st is not None and getattr(res_st, "ret", 0) != 0:
        log.error(
            f"xArm kon niet in READY-status gezet worden (ret={res_st.ret}). "
            "Controleer de noodstop en de vrijgave-knop op de controllerbox!"
        )
        return False

    log.info("Motoren en gripper van de echte xArm7 zijn geactiveerd (mode 0)!")
    return True


# ---------------------------------------------------------------------------
# Veiligheidscontroles
# ---------------------------------------------------------------------------
def controleer_zone(x, y, z):
    """Geeft None als het punt in de veilige zone ligt, anders de reden."""
    if not ZONE_X_MIN <= x <= ZONE_X_MAX:
        return f"x={x:.3f} buiten [{ZONE_X_MIN:.3f}, {ZONE_X_MAX:.3f}]"
    if not ZONE_Y_MIN <= y <= ZONE_Y_MAX:
        return f"y={y:.3f} buiten [{ZONE_Y_MIN:.3f}, {ZONE_Y_MAX:.3f}]"
    if not ZONE_Z_MIN <= z <= ZONE_Z_MAX:
        return f"z={z:.3f} buiten [{ZONE_Z_MIN:.3f}, {ZONE_Z_MAX:.3f}]"
    if math.hypot(x, y) < MIN_AFSTAND_BASIS:
        return f"te dicht bij de robotbasis (< {MIN_AFSTAND_BASIS:.2f} m)"
    return None


def limiet_fout(q):
    """Geeft None als alle gewrichten binnen hun limiet staan, anders de reden."""
    for nr, limiet in GEWRICHT_LIMIETEN_GRADEN.items():
        graden = math.degrees(q[nr - 1])
        if abs(graden) > limiet:
            return f"joint{nr} = {graden:.0f} graden (limiet ±{limiet})"
    return None


class SetupFout(Exception):
    pass


def wacht_op(node, future, naam, seconden=30.0):
    """Wacht op een serviceantwoord en meldt duidelijk wat er misgaat."""
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconden)
    log = node.get_logger()
    if not future.done():
        log.error(f"{naam}: geen antwoord binnen {seconden:.0f} seconden.")
        return None
    if future.exception() is not None:
        log.error(f"{naam}: serviceaanroep mislukt: {future.exception()}")
        return None
    return future.result()


class Arm:
    def __init__(self, node):
        self.node = node
        log = node.get_logger()

        self.joint_plan = node.create_client(PlanJoint, "/xarm_joint_plan")
        self.execute = node.create_client(PlanExec, "/xarm_exec_plan")
        if not self.joint_plan.wait_for_service(timeout_sec=10.0):
            raise SetupFout(
                "/xarm_joint_plan niet gevonden. Controleer met: "
                "ros2 service list | grep xarm"
            )
        self.execute.wait_for_service()

        self.gewrichten = {}
        topic = "/xarm/joint_states" if _topic_actief("/xarm/joint_states") else "/joint_states"
        log.info(f"Luisteren naar gewrichtsstanden op {topic}...")
        node.create_subscription(JointState, topic, self._joint_cb, 10)
        self.geblokkeerd = False

        eind = time.time() + 10.0
        while self.huidige_q() is None:
            if time.time() > eind:
                raise SetupFout(f"Geen gewrichtsstand ontvangen op {topic}.")
        q_start = self.huidige_q()
        log.info(f"Gewrichtsstand ontvangen (graden): {np.round(np.degrees(q_start), 1)}")

    def _joint_cb(self, msg):
        for naam, pos in zip(msg.name, msg.position):
            self.gewrichten[naam] = pos

    def huidige_q(self):
        """De huidige 7 gewrichtshoeken (rad), of None als ze nog niet bekend zijn."""
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

    def huidige_tcp(self):
        """Geeft de huidige (x, y, z) positie van het gripperpunt in meters."""
        q = self.huidige_q()
        if q is None:
            return None
        return fk(q)[0][:3, 3]

    def controleer_kinematica(self):
        """
        Vergelijkt het berekende eindpunt met TF en kalibreert indien nodig
        de TCP-offset automatisch zodat de berekening 100% aansluit op de robot.
        """
        log = self.node.get_logger()
        if Buffer is None:
            log.warn("tf2_ros niet beschikbaar: kinematica-controle overgeslagen.")
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
            log.warn(f"TF {BASE_FRAME} -> {EEF_FRAME} niet gevonden: controle overgeslagen.")
            return True

        gemeten = np.array([tf.transform.translation.x,
                            tf.transform.translation.y,
                            tf.transform.translation.z])
        q_nu = self.huidige_q()
        berekend = fk(q_nu)[0][:3, 3]
        verschil = float(np.linalg.norm(gemeten - berekend))
        log.info(f"Kinematica-controle ({EEF_FRAME}): afwijking {verschil * 100:.1f} cm")

        if verschil > MAX_KINEMATICA_AFWIJKING:
            try:
                tf_flens = buf.lookup_transform("link7", EEF_FRAME, Time())
                gemeten_offset = float(np.linalg.norm([
                    tf_flens.transform.translation.x,
                    tf_flens.transform.translation.y,
                    tf_flens.transform.translation.z,
                ]))
                log.warn(
                    f"TCP_OFFSET_Z aangepast van {cfg.TCP_OFFSET_Z:.3f} m naar "
                    f"{gemeten_offset:.3f} m op basis van robot-TF."
                )
                cfg.TCP_OFFSET_Z = gemeten_offset
                kin._KLAAR = None
                berekend = fk(q_nu)[0][:3, 3]
                verschil = float(np.linalg.norm(gemeten - berekend))
                log.info(f"Nieuwe kinematica-afwijking: {verschil * 100:.1f} cm")
            except Exception:
                pass

        if verschil > MAX_KINEMATICA_AFWIJKING:
            log.error(
                f"Berekening wijkt nog steeds {verschil * 100:.1f} cm af van de robot "
                f"(TF: {np.round(gemeten, 3)}, berekend: {np.round(berekend, 3)})."
            )
            return False
        return True

    def gewrichten_ok(self):
        q = self.huidige_q()
        if q is None:
            return True
        fout = limiet_fout(q)
        if fout:
            self.node.get_logger().error(
                f"VEILIGHEID: {fout}. Bewegingen geblokkeerd."
            )
            return False
        return True

    def _plan_en_voer_uit(self, q_doel, naam):
        """Gewrichtsplan naar q_doel plannen en uitvoeren. True bij succes."""
        log = self.node.get_logger()
        req = PlanJoint.Request()
        req.target = [float(v) for v in q_doel]
        result = wacht_op(self.node, self.joint_plan.call_async(req), f"plannen {naam}")
        if result is None or not result.success:
            log.error(f"Plannen mislukt: {naam}")
            return False

        c_servo = self.node.create_client(MoveJoint, "/xarm/set_servo_angle")
        if c_servo.wait_for_service(timeout_sec=1.5):
            m_req = MoveJoint.Request()
            m_req.angles = [float(v) for v in q_doel]
            m_req.speed = 0.35      # rad/s (rustige, veilige snelheid)
            m_req.acc = 2.0         # rad/s^2
            m_req.mvtime = 0.0
            m_req.wait = True
            m_req.timeout = 30.0
            res_m = wacht_op(self.node, c_servo.call_async(m_req), f"uitvoeren {naam}", 35.0)
            if res_m is None or getattr(res_m, "ret", 0) != 0:
                log.error(f"Uitvoeren mislukt: {naam} (ret={getattr(res_m, 'ret', 'timeout')})")
                activeer_echte_motoren(self.node)
                return False
            return True

        exec_req = PlanExec.Request()
        exec_req.wait = True
        result = wacht_op(self.node, self.execute.call_async(exec_req), f"uitvoeren {naam}", 120.0)
        if result is None or not result.success:
            log.error(f"Uitvoeren mislukt: {naam}")
            return False
        return True

    def eerst_omhoog_en_naar_klaarstand(self):
        """
        Kijkt eerst hoe de arm nu staat, tilt de grijper eerst veilig recht omhoog
        zodat hij niets op het bord raakt, en beweegt daarna naar de klaarstand.
        """
        log = self.node.get_logger()
        q_nu = self.huidige_q()
        if q_nu is None:
            return False

        pos_nu = fk(q_nu)[0][:3, 3]
        veilige_z = max(float(pos_nu[2]), cfg.KLAAR_TCP[2])
        log.info(
            f"Huidige positie gripper: x={pos_nu[0]:.3f}, y={pos_nu[1]:.3f}, z={pos_nu[2]:.3f} m"
        )

        if pos_nu[2] < cfg.KLAAR_TCP[2] - 0.02:
            log.info(f"Stap 1: Arm eerst recht omhoog tillen naar z={veilige_z:.3f} m...")
            q_omhoog = ik([pos_nu[0], pos_nu[1], veilige_z], q_nu)
            if q_omhoog is not None and limiet_fout(q_omhoog) is None:
                if not self._plan_en_voer_uit(q_omhoog, "veilig omhoog"):
                    return False
            else:
                log.warn("Recht omhoog op huidige plek niet bereikbaar; ik ga direct naar klaarstand.")

        log.info(f"Stap 2: Arm naar de beginpositie (gripperpunt op {cfg.KLAAR_TCP})...")
        if not self._plan_en_voer_uit(klaar_q(), "klaarstand"):
            return False
        return self.gewrichten_ok()

    def naar_klaarstand(self):
        return self.eerst_omhoog_en_naar_klaarstand()

    def beweeg(self, x, y, z, naam):
        """Gewrichtsbeweging naar (x, y, z) met de gripper omlaag, na vooraf controleren."""
        log = self.node.get_logger()

        if self.geblokkeerd:
            log.error("Bewegingen zijn geblokkeerd na een veiligheidsmelding.")
            return False

        reden = controleer_zone(x, y, z)
        if reden:
            log.error(f"GEWEIGERD ({naam}): {reden}")
            return False

        q_nu = self.huidige_q()
        if q_nu is None:
            log.error("Huidige gewrichtsstand onbekend.")
            return False

        q_doel = ik([x, y, z], q_nu)
        if q_doel is None or limiet_fout(q_doel):
            alternatief = ik([x, y, z], klaar_q())
            if alternatief is not None and limiet_fout(alternatief) is None:
                q_doel = alternatief
        if q_doel is None:
            log.error(
                f"GEWEIGERD ({naam}): punt niet bereikbaar met de gripper omlaag "
                f"(x={x:.3f}, y={y:.3f}, z={z:.3f})."
            )
            return False

        fout = limiet_fout(q_doel)
        if fout:
            log.error(f"GEWEIGERD ({naam}): zou {fout} geven.")
            return False

        sprong = math.degrees(float(np.max(np.abs(q_doel - q_nu))))
        if sprong > MAX_GEWRICHTSSPRONG_GRADEN:
            log.error(
                f"GEWEIGERD ({naam}): een gewricht moet {sprong:.0f} graden draaien "
                f"(max {MAX_GEWRICHTSSPRONG_GRADEN}). Zet de arm eerst in de klaarstand "
                "(dambord_setup.py doet dat)."
            )
            return False

        if not self._plan_en_voer_uit(q_doel, f"{naam} (x={x:.3f}, y={y:.3f}, z={z:.3f})"):
            return False

        if not self.gewrichten_ok():
            self.geblokkeerd = True
            return False
        return True


class Gripper:
    """Opent en sluit de G2 gripper op zowel de echte xArm als in simulatie."""

    def __init__(self, node):
        self.node = node
        self.actief = False
        self.modus = None
        if not cfg.GRIPPER:
            return

        self.grip_pos_client = node.create_client(GripperMove, "/xarm/set_gripper_position")
        if self.grip_pos_client.wait_for_service(timeout_sec=2.0):
            self.modus = "service"
            self.actief = True
            node.get_logger().info("Echte G2 Gripper gekoppeld via /xarm/set_gripper_position.")
            return

        self.grip_plan = node.create_client(PlanJoint, "/xarm_gripper_joint_plan")
        self.grip_exec = node.create_client(PlanExec, "/xarm_gripper_exec_plan")
        if self.grip_plan.wait_for_service(timeout_sec=3.0) and self.grip_exec.wait_for_service(timeout_sec=3.0):
            self.modus = "planner"
            self.actief = True
            node.get_logger().info("G2 Gripper gekoppeld via /xarm_gripper_joint_plan.")
            return

        self.traj_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/xarm_gripper_traj_controller/follow_joint_trajectory",
        )
        if self.traj_client.wait_for_server(timeout_sec=3.0):
            self.modus = "traj"
            self.actief = True
            node.get_logger().info("Gripper gekoppeld via /xarm_gripper_traj_controller.")
            return

        node.get_logger().warn("Geen gripper-service of controller gevonden.")

    def _ga_naar(self, positie, naam):
        log = self.node.get_logger()
        if not self.actief:
            return True

        if self.modus == "service":
            verhouding = max(0.0, min(1.0, positie / max(0.01, cfg.GRIPPER_DICHT)))
            echte_pos = float(850.0 * (1.0 - verhouding))
            log.info(f"Gripper {naam} (pos = {echte_pos:.0f})")
            req = GripperMove.Request()
            req.pos = echte_pos
            req.wait = True
            req.timeout = 5.0
            res = wacht_op(self.node, self.grip_pos_client.call_async(req), f"gripper {naam}", 10.0)
            return res is not None and getattr(res, "ret", 0) == 0

        if self.modus == "planner":
            log.info(f"G2 Gripper {naam} (target = {positie:.2f} rad)")
            req = PlanJoint.Request()
            req.target = [float(positie)]
            res_p = wacht_op(self.node, self.grip_plan.call_async(req), f"gripper plannen {naam}", 10.0)
            if res_p is None or not res_p.success:
                log.error(f"Gripper plannen {naam} mislukt.")
                return False
            ex = PlanExec.Request()
            ex.wait = True
            res_e = wacht_op(self.node, self.grip_exec.call_async(ex), f"gripper uitvoeren {naam}", 10.0)
            if res_e is None or not res_e.success:
                log.warn(f"Gripper {naam}: /xarm_gripper_exec_plan genegeerd (check: ros2 service list | grep grip)")
            return True

        log.info(f"Gripper {naam} (drive_joint = {positie:.2f} rad)")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["drive_joint"]
        punt = JointTrajectoryPoint()
        punt.positions = [float(positie)]
        punt.time_from_start = Duration(sec=0, nanosec=500_000_000)
        goal.trajectory.points = [punt]
        send_future = self.traj_client.send_goal_async(goal)
        goal_handle = wacht_op(self.node, send_future, f"gripper {naam} doel", 10.0)
        if goal_handle is None or not goal_handle.accepted:
            log.error(f"Gripper {naam}: doel geweigerd.")
            return False
        result = wacht_op(self.node, goal_handle.get_result_async(), f"gripper {naam} uitvoeren", 10.0)
        return result is not None

    def open(self):
        return self._ga_naar(cfg.GRIPPER_OPEN, "open")

    def dicht(self):
        return self._ga_naar(cfg.GRIPPER_DICHT, "dicht")


# ---------------------------------------------------------------------------
# Planning scene: schijven lezen, vastmaken en loslaten
# ---------------------------------------------------------------------------
def _quat_naar_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _pose_matrix(pose):
    m = np.eye(4)
    m[:3, :3] = _quat_naar_matrix(pose.orientation)
    m[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return m


def object_grenzen(obj):
    t_obj = _pose_matrix(obj.pose)
    punten = []
    for mesh, mp in zip(obj.meshes, obj.mesh_poses):
        t = t_obj @ _pose_matrix(mp)
        v = np.array([[p.x, p.y, p.z] for p in mesh.vertices])
        if len(v):
            punten.append(v @ t[:3, :3].T + t[:3, 3])
    for prim, pp in zip(obj.primitives, obj.primitive_poses):
        d = list(prim.dimensions)
        if prim.type == SolidPrimitive.BOX:
            half = (d[0] / 2, d[1] / 2, d[2] / 2)
        elif prim.type == SolidPrimitive.CYLINDER:
            half = (d[1], d[1], d[0] / 2)
        elif prim.type == SolidPrimitive.SPHERE:
            half = (d[0], d[0], d[0])
        else:
            continue
        hoeken = np.array([[sx * half[0], sy * half[1], sz * half[2]]
                           for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        t = t_obj @ _pose_matrix(pp)
        punten.append(hoeken @ t[:3, :3].T + t[:3, 3])
    if not punten:
        return None
    p = np.vstack(punten)
    return p.min(axis=0), p.max(axis=0)


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
        result = wacht_op(self.node, self.get.call_async(req), "/get_planning_scene", 20.0)
        return result.scene if result is not None else None

    def pas_toe(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        req = ApplyPlanningScene.Request()
        req.scene = scene
        result = wacht_op(self.node, self.apply.call_async(req), "/apply_planning_scene", 20.0)
        if result is None:
            return False
        if not result.success:
            self.node.get_logger().error("/apply_planning_scene gaf success=False.")
            return False
        return True


class Schijven:
    """Houdt bij welke schijf op welk vakje ligt, welke kleur hij heeft en wie aan zet is."""

    def __init__(self, scene, log):
        self.scene = scene
        self.log = log
        self.bezet = {}
        self.vast_id = None
        self.pick_z = None
        self.beurt = "wit"
        self.geslagen_teller = 0
        self.actief = False

        if not scene.beschikbaar:
            log.warn("Planning scene services niet gevonden: schijven worden niet gebruikt.")
            return

        for poging in range(1, 4):
            if self.ververs():
                self.actief = bool(self.bezet) or self.vast_id is not None
                if self.actief:
                    break
            log.warn(f"Poging {poging}/3 om schijven uit de scene te lezen mislukt; ik probeer opnieuw...")
            time.sleep(1.0)

        if not self.actief:
            log.warn(
                f"Geen schijven ('{cfg.SCHIJF_PREFIX}...') in de scene gevonden. "
                "Draai eerst dambord_setup.py en typ daarna 'l' om te herladen."
            )

    def ververs(self):
        scene = self.scene.lees(
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        if scene is None:
            self.log.error("Kon de planning scene niet lezen.")
            return False

        self.bezet = {}
        for obj in scene.world.collision_objects:
            if not obj.id.startswith(cfg.SCHIJF_PREFIX):
                continue
            grenzen = object_grenzen(obj)
            if grenzen is None:
                continue
            mn, mx = grenzen
            x, y = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
            vakje, afstand = cfg.dichtstbij(x, y)
            if afstand > MAX_AFSTAND_SCHIJF_VAKJE:
                continue
            if vakje in self.bezet:
                self.log.warn(f"Vakje {cfg.vakje_naar_naam(vakje)} heeft meer dan één schijf.")
                continue
            # Schijven liggen door de zwaartekracht altijd vlak op het bord
            self.bezet[vakje] = {
                "id": obj.id,
                "kleur": schijf_kleur(obj.id),
                "x": x,
                "y": y,
                "z_top": float(cfg.BORD_HOOGTE + cfg.SCHIJF_HOOGTE),
            }

        vast = [a.object.id for a in scene.robot_state.attached_collision_objects
                if a.object.id.startswith(cfg.SCHIJF_PREFIX)]
        self.vast_id = vast[0] if vast else None
        return True

    def _is_vast(self, object_id):
        scene = self.scene.lees(PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        if scene is None:
            return False
        return any(a.object.id == object_id
                   for a in scene.robot_state.attached_collision_objects)

    def _wereld_object(self, object_id):
        scene = self.scene.lees(
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        if scene is None:
            return None
        for obj in scene.world.collision_objects:
            if obj.id == object_id:
                return obj
        return None

    def vastmaken(self, object_id, vakje):
        naam = cfg.vakje_naar_naam(vakje)
        for poging in (0, 1):
            scene = PlanningScene()
            aco = AttachedCollisionObject()
            aco.link_name = EEF_FRAME
            aco.touch_links = list(cfg.TOUCH_LINKS)
            aco.object.header.frame_id = BASE_FRAME
            aco.object.id = object_id
            aco.object.operation = CollisionObject.ADD

            if poging == 1:
                obj = self._wereld_object(object_id)
                if obj is None:
                    break
                obj.operation = CollisionObject.ADD
                aco.object = obj
                weg = CollisionObject()
                weg.header.frame_id = BASE_FRAME
                weg.id = object_id
                weg.operation = CollisionObject.REMOVE
                scene.world.collision_objects = [weg]

            scene.robot_state.attached_collision_objects = [aco]
            if self.scene.pas_toe(scene) and self._is_vast(object_id):
                self.vast_id = object_id
                self.bezet.pop(vakje, None)
                self.log.info(f"Schijf {object_id} ({schijf_kleur(object_id)}) van {naam} vastgepakt.")
                return True

        self.log.error(f"Vastmaken van {object_id} in de scene mislukt.")
        return False

    def loslaten(self, label_doel):
        object_id = self.vast_id
        scene = PlanningScene()
        aco = AttachedCollisionObject()
        aco.link_name = EEF_FRAME
        aco.object.header.frame_id = BASE_FRAME
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [aco]

        if not self.scene.pas_toe(scene) or self._is_vast(object_id):
            self.log.error(f"Loslaten van {object_id} in de scene mislukt.")
            return False

        self.vast_id = None
        self.ververs()
        self.log.info(f"Schijf {object_id} losgelaten op {label_doel}.")
        return True

    def wissel_beurt(self):
        self.beurt = "zwart" if self.beurt == "wit" else "wit"

    def overzicht(self):
        if self.vast_id:
            vast = f"Gripper houdt {self.vast_id} ({schijf_kleur(self.vast_id)}) vast. "
        else:
            vast = f"Aan de beurt: {self.beurt.upper()}. "
        wit = [cfg.vakje_naar_naam(v) for v, info in sorted(self.bezet.items()) if info["kleur"] == "wit"]
        zwart = [cfg.vakje_naar_naam(v) for v, info in sorted(self.bezet.items()) if info["kleur"] == "zwart"]
        return f"{vast}\n  Wit   ({len(wit)}): {', '.join(wit)}\n  Zwart ({len(zwart)}): {', '.join(zwart)}"


# ---------------------------------------------------------------------------
# Bewegingen, slaan en hoofdprogramma
# ---------------------------------------------------------------------------
def _bereken_opneem_z(z_top):
    return max(ZONE_Z_MIN, z_top - cfg.GREEP_DIEPTE + OPNEEM_HOOGTE_EXTRA)


def bezoek_punt(arm, label, x, y, z_doel, voor_omlaag=None, tussenactie=None):
    z_hover = max(cfg.BORD_HOOGTE + HOVER_HOOGTE, z_doel + 0.05)

    log = arm.node.get_logger()
    log.info(f"{label}: x={x:.3f}, y={y:.3f}, z={z_doel:.3f}")

    if not arm.beweeg(x, y, z_hover, f"boven {label}"):
        return False
    if voor_omlaag is not None and not voor_omlaag():
        return False
    if not arm.beweeg(x, y, z_doel, f"omlaag {label}"):
        return False

    ok = tussenactie() if tussenactie else True
    time.sleep(PAUZE)

    omhoog = arm.beweeg(x, y, z_hover, f"omhoog {label}")
    return ok and omhoog


def schijf_beurt(arm, gripper, schijven, nummer, vrije_kleurkeuze=False, wissel_beurt_na=True):
    naam = cfg.vakje_naar_naam(nummer)

    if schijven.vast_id is None:
        if nummer not in schijven.bezet:
            print(f"Op {naam} ligt geen schijf. Typ 'l' om te zien waar ze liggen.")
            return False
        info = schijven.bezet[nummer]
        if not vrije_kleurkeuze and info["kleur"] != schijven.beurt:
            print(
                f"Op {naam} ligt een {info['kleur']}e schijf, maar {schijven.beurt.upper()} is aan de beurt! "
                f"(Typ 'w' om van kleur te wisselen.)"
            )
            return False

        z_pick = _bereken_opneem_z(info["z_top"])
        schijven.pick_z = z_pick
        return bezoek_punt(
            arm, naam, info["x"], info["y"], z_pick,
            voor_omlaag=gripper.open,
            tussenactie=lambda: gripper.dicht() and schijven.vastmaken(info["id"], nummer),
        )
    else:
        if nummer in schijven.bezet:
            print(f"Vakje {naam} is bezet door een {schijven.bezet[nummer]['kleur']}e schijf. Kies een leeg vakje.")
            return False
        x, y = cfg.vakje_positie(nummer)
        z_basis = schijven.pick_z
        if z_basis is None:
            z_basis = _bereken_opneem_z(cfg.BORD_HOOGTE + cfg.SCHIJF_HOOGTE)
        z_plaats = z_basis + LOSLAAT_HOOGTE_EXTRA

        geslaagd = bezoek_punt(
            arm, naam, x, y, z_plaats,
            tussenactie=lambda: gripper.open() and schijven.loslaten(naam),
        )
        if geslaagd and wissel_beurt_na:
            schijven.wissel_beurt()
            print(f"Zet voltooid! Nu is {schijven.beurt.upper()} aan de beurt.")
        return geslaagd


def vind_geslagen_schijven(schijven, van_nr, naar_nr, eigen_kleur):
    """
    Zoekt alle vijandelijke schijven die op de diagonaal tussen van_nr en naar_nr liggen.
    Werkt voor zowel een gewone slag (2 vakjes) als een dam-slag (meerdere vakjes).
    """
    x1, y1 = cfg.vakje_positie(van_nr)
    x2, y2 = cfg.vakje_positie(naar_nr)
    dx = x2 - x1
    dy = y2 - y1

    stappen_x = round(abs(dx) / cfg.VAK)
    stappen_y = round(abs(dy) / cfg.VAK)
    if stappen_x != stappen_y or stappen_x < 2:
        return []

    geslagen = []
    for stap in range(1, stappen_x):
        tx = x1 + (dx / stappen_x) * stap
        ty = y1 + (dy / stappen_y) * stap
        tussen_nr, afstand = cfg.dichtstbij(tx, ty)
        if afstand <= 0.015 and tussen_nr in schijven.bezet:
            if schijven.bezet[tussen_nr]["kleur"] != eigen_kleur:
                geslagen.append(tussen_nr)
    return geslagen


def verwijder_geslagen_schijf(arm, gripper, schijven, nummer):
    naam = cfg.vakje_naar_naam(nummer)
    if schijven.vast_id is not None:
        print("De gripper houdt al een schijf vast. Zet die eerst neer.")
        return False
    if nummer not in schijven.bezet:
        print(f"Op {naam} ligt geen schijf om te verwijderen.")
        return False

    info = schijven.bezet[nummer]
    z_pick = _bereken_opneem_z(info["z_top"])
    opgepakt = bezoek_punt(
        arm, f"slaan op {naam}", info["x"], info["y"], z_pick,
        voor_omlaag=gripper.open,
        tussenactie=lambda: gripper.dicht() and schijven.vastmaken(info["id"], nummer),
    )
    if not opgepakt:
        return False

    ax, ay = cfg.AFLEG_POSITIE
    ay_offset = ay + (schijven.geslagen_teller % 5) * 0.04
    schijven.geslagen_teller += 1
    z_plaats = z_pick + LOSLAAT_HOOGTE_EXTRA
    return bezoek_punt(
        arm, "aflegplek naast bord", ax, ay_offset, z_plaats,
        tussenactie=lambda: gripper.open() and schijven.loslaten("aflegplek"),
    )


def voer_zetreeks_uit(arm, gripper, schijven, vakjes):
    """
    Voert een enkele zet (bijv. B4-C5), een slagzet (B4-D6 / B4xD6)
    of een meerslag (B4-D6-F8) uit en legt geslagen schijven naast het bord.
    """
    log = arm.node.get_logger()
    alle_geslagen = []

    for i in range(len(vakjes) - 1):
        van_nr = vakjes[i]
        naar_nr = vakjes[i + 1]

        if van_nr not in schijven.bezet:
            print(f"Op {cfg.vakje_naar_naam(van_nr)} ligt geen schijf.")
            return False
        eigen_kleur = schijven.bezet[van_nr]["kleur"]
        if eigen_kleur != schijven.beurt:
            print(
                f"Op {cfg.vakje_naar_naam(van_nr)} ligt een {eigen_kleur}e schijf, "
                f"maar {schijven.beurt.upper()} is aan de beurt! (Typ 'w' om te wisselen.)"
            )
            return False
        if naar_nr in schijven.bezet:
            print(f"Vakje {cfg.vakje_naar_naam(naar_nr)} is al bezet.")
            return False

        geslagen = vind_geslagen_schijven(schijven, van_nr, naar_nr, eigen_kleur)
        if not schijf_beurt(arm, gripper, schijven, van_nr, wissel_beurt_na=False):
            return False
        if not schijf_beurt(arm, gripper, schijven, naar_nr, wissel_beurt_na=False):
            return False

        for g_nr in geslagen:
            g_naam = cfg.vakje_naar_naam(g_nr)
            log.info(f"Geslagen schijf op {g_naam} wegnemen en naast het bord leggen...")
            if not verwijder_geslagen_schijf(arm, gripper, schijven, g_nr):
                return False
            alle_geslagen.append(g_naam)

    schijven.wissel_beurt()
    if alle_geslagen:
        print(
            f"Slagzet voltooid (geslagen: {', '.join(alle_geslagen)})! "
            f"Nu is {schijven.beurt.upper()} aan de beurt."
        )
    else:
        print(f"Zet voltooid! Nu is {schijven.beurt.upper()} aan de beurt.")
    return True


def calibratie(arm):
    for nummer in (1, 5, 46, 50):
        x, y = cfg.vakje_positie(nummer)
        naam = cfg.vakje_naar_naam(nummer)
        arm.node.get_logger().info(f"Calibratie: boven {naam}")
        if not arm.beweeg(x, y, cfg.BORD_HOOGTE + HOVER_HOOGTE, f"calibratie {naam}"):
            return
        time.sleep(1.5)


def controleer_alle_vakjes(log):
    ok = True
    for nummer in range(1, 51):
        x, y = cfg.vakje_positie(nummer)
        for z in (cfg.BORD_HOOGTE + HOVER_HOOGTE, cfg.BORD_HOOGTE + PICK_HOOGTE, ZONE_Z_MIN):
            reden = controleer_zone(x, y, z)
            if reden:
                log.error(f"Vakje {cfg.vakje_naar_naam(nummer)} ligt buiten de zone: {reden}")
                ok = False
    return ok


def main():
    rclpy.init()
    node = rclpy.create_node("dambord_move")
    log = node.get_logger()

    try:
        if not verbind_met_echte_arm(log):
            return
        if not activeer_echte_motoren(node):
            return

        log.info(
            f"Bord: vakje = {cfg.VAK * 100:.1f} cm, "
            f"bovenkant op z = {cfg.BORD_HOOGTE:.3f} m"
        )
        log.info(
            f"Veilige zone: x [{ZONE_X_MIN:.2f}, {ZONE_X_MAX:.2f}], "
            f"y [{ZONE_Y_MIN:.2f}, {ZONE_Y_MAX:.2f}], "
            f"z [{ZONE_Z_MIN:.2f}, {ZONE_Z_MAX:.2f}] m"
        )
        for n in (1, 5, 46, 50):
            x, y = cfg.vakje_positie(n)
            log.info(f"  {cfg.vakje_naar_naam(n)}: x={x:.3f}, y={y:.3f}")

        if not controleer_alle_vakjes(log):
            log.error("Instellingen kloppen niet met de zone. Ik stop.")
            return

        arm = Arm(node)
        if not arm.controleer_kinematica():
            return
        if not arm.gewrichten_ok():
            antwoord = input(
                "\nDe arm staat buiten de gewrichtslimieten. "
                "Eerst omhoog naar de klaarstand? [j/n]: "
            ).strip().lower()
            if antwoord != "j" or not arm.eerst_omhoog_en_naar_klaarstand():
                log.error(
                    "Zet de arm eerst terug (bijvoorbeeld met dambord_setup.py) "
                    "en start opnieuw."
                )
                return
            log.info("Arm staat weer in de klaarstand.")

        gripper = Gripper(node)
        schijven = Schijven(Scene(node), log)
        if schijven.actief:
            print("\n" + schijven.overzicht())
            if schijven.vast_id is None:
                gripper.open()

        while True:
            status = (
                f"vast: {schijven.vast_id}"
                if schijven.vast_id
                else f"beurt: {schijven.beurt.upper()}"
            )
            invoer = input(
                f"\n[{status}] Zet (bijv. B4-C5 of slag B4-D6), 'x B4' = los slaan, "
                "'w' = wissel kleur, 'l' = overzicht, 'o'/'d' = gripper, 'c' = cal, 'q' = stop: "
            ).strip()

            cmd = invoer.lower()
            if cmd == "q":
                if schijven.vast_id:
                    print(f"Let op: {schijven.vast_id} zit nog vast aan de gripper.")
                break
            if cmd == "l":
                if schijven.ververs():
                    schijven.actief = bool(schijven.bezet) or schijven.vast_id is not None
                if schijven.actief:
                    print(schijven.overzicht())
                else:
                    print("Geen schijven in de scene gevonden. Draai eerst dambord_setup.py.")
                continue
            if cmd == "w":
                if schijven.vast_id:
                    print("Zet eerst de huidige schijf neer voordat je van beurt wisselt.")
                else:
                    schijven.wissel_beurt()
                    print(f"Beurt gewisseld: {schijven.beurt.upper()} is nu aan zet.")
                continue
            if cmd == "o":
                gripper.open()
                continue
            if cmd == "d":
                gripper.dicht()
                continue
            if cmd == "c":
                calibratie(arm)
                if arm.geblokkeerd:
                    break
                continue

            if cmd.startswith("x "):
                vak_tekst = invoer[2:].strip()
                nummer, fout = cfg.naam_naar_vakje(vak_tekst)
                if fout:
                    print(fout)
                elif schijven.actief:
                    verwijder_geslagen_schijf(arm, gripper, schijven, nummer)
                continue

            delen = [d for d in re.split(r"[-xX\s]+", invoer.strip()) if d]
            if len(delen) >= 2:
                vakjes = []
                ongeldig = None
                for d in delen:
                    nr, fout = cfg.naam_naar_vakje(d)
                    if fout:
                        ongeldig = fout
                        break
                    vakjes.append(nr)
                if ongeldig:
                    print(ongeldig)
                    continue
                if schijven.actief:
                    voer_zetreeks_uit(arm, gripper, schijven, vakjes)
                else:
                    for nr in vakjes:
                        x, y = cfg.vakje_positie(nr)
                        bezoek_punt(arm, cfg.vakje_naar_naam(nr), x, y, cfg.BORD_HOOGTE + PICK_HOOGTE)
                if arm.geblokkeerd:
                    break
                continue

            nummer, fout = cfg.naam_naar_vakje(invoer)
            if fout:
                print(fout)
                continue

            if schijven.actief:
                schijf_beurt(arm, gripper, schijven, nummer)
            else:
                x, y = cfg.vakje_positie(nummer)
                bezoek_punt(arm, cfg.vakje_naar_naam(nummer), x, y, cfg.BORD_HOOGTE + PICK_HOOGTE)

            if arm.geblokkeerd:
                log.error("Gestopt wegens een veiligheidsmelding.")
                break
    except SetupFout as e:
        log.error(str(e))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()