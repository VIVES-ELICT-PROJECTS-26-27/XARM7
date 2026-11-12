#!/usr/bin/env python3
"""
Beweeg de xArm7 naar de 50 speelbare (donkere) vakjes van een 10x10 dambord.

Het bord is 45 x 45 cm en ligt voor de robot, met de dichtstbijzijnde rand
op 18 cm van de robotbasis (x = 0.18 m), gecentreerd op y = 0.

Gezien vanaf de robotkant, kijkend naar het bord:
  - onderkant van het bord = kant het dichtst bij de robot (kleine x)
  - links = positieve y (links van de robot)
Vakje 1 = links onder, vakje 50 = rechts boven. De nummers lopen per rij
van links naar rechts, 5 vakjes per rij (1-5, 6-10, ..., 46-50).
Linksonder is een donker vakje.

Hoe de bewegingen veilig blijven:
  1. Het script rekent zelf de gewrichtshoeken uit (inverse kinematica), vertrekkend
     vanaf de HUIDIGE stand van de arm. Zo kiest de arm altijd de dichtstbijzijnde
     oplossing en klapt hij niet om naar een andere armstand.
  2. Elke beweging wordt VOORAF gecontroleerd: doelpunt binnen de zone (bord + 20 cm),
     gewrichten binnen de limieten, geen te grote sprong. Anders gebeurt er niets.
  3. De beweging zelf is een gewone gewrichtsbeweging (/xarm_joint_plan) naar die
     vooraf gecontroleerde stand. De grijper wijst altijd recht naar beneden.
  4. Na elke beweging worden de gewrichten nogmaals gecontroleerd.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from xarm_msgs.srv import PlanExec, PlanJoint

try:
    from tf2_ros import Buffer, TransformListener
except ImportError:  # TF-controle is optioneel
    Buffer = None

# ======================= INSTELLINGEN =======================
BORD_BREEDTE = 0.45     # 45 cm, bord is vierkant
BORD_X_NABIJ = 0.18     # afstand robotbasis -> dichtstbijzijnde rand van het bord (m)
BORD_Y_MIDDEN = 0.0     # y-positie van het midden van het bord (m)
BORD_HOOGTE = 0.02      # z van de bovenkant van het bord (m), meet dit na
BORDER = 0.0            # rand rond het speelveld (m), 0 = vakjes beslaan het hele bord
N = 10                  # 10x10 bord

# Pas aan als vakje 1 niet op de juiste hoek uitkomt
SPIEGEL_KOLOM = False   # spiegel links/rechts
SPIEGEL_RIJ = False     # spiegel onder/boven

HOVER_HOOGTE = 0.10     # hoogte boven de bovenkant van het bord (m)
PICK_HOOGTE = 0.02      # hoogte van de grijper boven het bord bij "oppakken" (m)
PAUZE = 1.0             # seconden wachten onderaan

# --- Veiligheidszone ---
ZONE_MARGE = 0.20           # 20 cm buiten het speelveld
ZONE_MIN_BOVEN_BORD = 0.01  # nooit lager dan 1 cm boven het bord
ZONE_MAX_BOVEN_BORD = 0.30  # nooit hoger dan 30 cm boven het bord
MIN_AFSTAND_BASIS = 0.15    # minimale horizontale afstand tot de robotbasis (m)

# --- Gewrichten ---
JOINT_STATES_TOPIC = "/joint_states"   # soms "/xarm/joint_states"
GEWRICHT_LIMIETEN_GRADEN = {           # gewrichtsnummer: maximaal ± graden
    1: 100,   # basisdraaiing: ruim genoeg voor het hele bord
    3: 180,   # bovenarm-rotatie: maximaal een halve slag per kant
    5: 180,   # onderarm-rotatie
    7: 180,   # pols: maximaal een halve slag per kant
}
MAX_GEWRICHTSSPRONG_GRADEN = 120   # grootste toegestane verandering per gewricht per beweging

# --- Kinematica-controle via TF ---
TCP_OFFSET_Z = 0.0      # afstand eindpunt voorbij flens (m); 0 = geen grijper in het model
BASE_FRAME = "link_base"
EEF_FRAME = "link_eef"
MAX_KINEMATICA_AFWIJKING = 0.01    # m
# ============================================================

ZONE_X_MIN = BORD_X_NABIJ - ZONE_MARGE
ZONE_X_MAX = BORD_X_NABIJ + BORD_BREEDTE + ZONE_MARGE
ZONE_Y_MIN = BORD_Y_MIDDEN - BORD_BREEDTE / 2 - ZONE_MARGE
ZONE_Y_MAX = BORD_Y_MIDDEN + BORD_BREEDTE / 2 + ZONE_MARGE
ZONE_Z_MIN = BORD_HOOGTE + ZONE_MIN_BOVEN_BORD
ZONE_Z_MAX = BORD_HOOGTE + ZONE_MAX_BOVEN_BORD


# ---------------------------------------------------------------------------
# Kinematica xArm7 (xarm_description/config/kinematics/default/xarm7_default_kinematics.yaml)
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
_LOWER = np.array([-2 * math.pi, -2.059, -2 * math.pi, -0.19198,
                   -2 * math.pi, -1.69297, -2 * math.pi])
_UPPER = np.array([2 * math.pi, 2.0944, 2 * math.pi, 3.927,
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


def ik(pos, q_start, iters=300, lam=0.02):
    """
    Inverse kinematica (damped least squares), vertrekkend vanaf q_start.
    Geeft de gewrichtshoeken voor positie 'pos' met de grijper omlaag, of None.
    """
    q = np.clip(np.array(q_start, dtype=float), _LOWER, _UPPER)
    for _ in range(iters):
        t, frames = fk(q)
        ep = np.asarray(pos) - t[:3, 3]
        eo = _rotvec(R_OMLAAG @ t[:3, :3].T)
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(eo) < 1e-3:
            return q
        jac = np.zeros((6, 7))
        for i in range(7):
            z = frames[i][:3, 2]
            jac[:3, i] = np.cross(z, t[:3, 3] - frames[i][:3, 3])
            jac[3:, i] = z
        jp = jac.T @ np.linalg.inv(jac @ jac.T + lam ** 2 * np.eye(6))
        dq = jp @ np.concatenate([ep, eo])
        m = np.max(np.abs(dq))
        if m > 0.2:
            dq *= 0.2 / m
        q = np.clip(q + dq, _LOWER, _UPPER)
    return None


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


class Bord:
    """Rekent vakjesnummers om naar x, y in het robot-basisframe."""

    def __init__(self):
        self.vak = (BORD_BREEDTE - 2 * BORDER) / N
        self.z_top = BORD_HOOGTE

    def positie(self, nummer):
        idx = nummer - 1
        rij = idx // 5                                  # 0 = onderste rij
        kolom = 2 * (idx % 5) + (1 if rij % 2 else 0)   # 0 = meest linkse kolom

        if SPIEGEL_KOLOM:
            kolom = N - 1 - kolom
        if SPIEGEL_RIJ:
            rij = N - 1 - rij

        # rij loopt van de robot af (x neemt toe), kolom loopt van links naar rechts
        # (y neemt af)
        x = BORD_X_NABIJ + BORDER + (rij + 0.5) * self.vak
        y = BORD_Y_MIDDEN + BORD_BREEDTE / 2 - BORDER - (kolom + 0.5) * self.vak
        return x, y


class SetupFout(Exception):
    pass


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
        node.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_cb, 10)
        self.geblokkeerd = False

        # Wachten op de huidige gewrichtsstand (nodig om veilig te kunnen plannen)
        eind = time.time() + 10.0
        while self.huidige_q() is None:
            if time.time() > eind:
                raise SetupFout(
                    f"Geen gewrichtsstand ontvangen op {JOINT_STATES_TOPIC}. "
                    "Controleer de topic met: ros2 topic list | grep joint_states"
                )
        log.info("Gewrichtsstand ontvangen.")

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

    def controleer_kinematica(self):
        """Vergelijkt mijn berekende eindpunt met dat van TF. False = afwijking."""
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
            log.warn(
                f"TF {BASE_FRAME} -> {EEF_FRAME} niet gevonden: "
                "kinematica-controle overgeslagen."
            )
            return True

        gemeten = np.array([tf.transform.translation.x,
                            tf.transform.translation.y,
                            tf.transform.translation.z])
        berekend = fk(self.huidige_q())[0][:3, 3]
        verschil = float(np.linalg.norm(gemeten - berekend))
        log.info(f"Kinematica-controle: afwijking {verschil * 100:.1f} cm")
        if verschil > MAX_KINEMATICA_AFWIJKING:
            log.error(
                f"Mijn berekening wijkt {verschil * 100:.1f} cm af van de robot "
                f"(TF: {np.round(gemeten, 3)}, berekend: {np.round(berekend, 3)}). "
                "Controleer TCP_OFFSET_Z (grijper) en de frames."
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

    def beweeg(self, x, y, z, naam):
        """Gewrichtsbeweging naar (x, y, z) met de grijper omlaag, na vooraf controleren."""
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
        if q_doel is None:
            log.error(
                f"GEWEIGERD ({naam}): punt niet bereikbaar met de grijper omlaag "
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
                f"(max {MAX_GEWRICHTSSPRONG_GRADEN}). Zet de arm eerst in een "
                "stand dichter bij het bord."
            )
            return False

        req = PlanJoint.Request()
        req.target = [float(v) for v in q_doel]
        future = self.joint_plan.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None or not future.result().success:
            log.error(f"Plannen mislukt: {naam} (x={x:.3f}, y={y:.3f}, z={z:.3f})")
            return False

        exec_req = PlanExec.Request()
        exec_req.wait = True
        future = self.execute.call_async(exec_req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None or not future.result().success:
            log.error(f"Uitvoeren mislukt: {naam}")
            return False

        if not self.gewrichten_ok():
            self.geblokkeerd = True
            return False
        return True


def bezoek_vakje(arm, bord, nummer, actie):
    """Boven het vakje, omlaag, pauze, omhoog."""
    x, y = bord.positie(nummer)
    z_hover = bord.z_top + HOVER_HOOGTE
    z_pick = bord.z_top + PICK_HOOGTE

    log = arm.node.get_logger()
    log.info(f"Vakje {nummer}: x={x:.3f}, y={y:.3f}")

    if not arm.beweeg(x, y, z_hover, f"boven vakje {nummer}"):
        return False
    if not arm.beweeg(x, y, z_pick, f"omlaag vakje {nummer}"):
        return False

    log.info(f"Damschijf {actie} op vakje {nummer}...")
    time.sleep(PAUZE)

    return arm.beweeg(x, y, z_hover, f"omhoog vakje {nummer}")


def calibratie(arm, bord):
    """Beweegt boven de hoekvakjes zodat je in RViz kan controleren of alles klopt."""
    for nummer in (1, 5, 46, 50):
        x, y = bord.positie(nummer)
        arm.node.get_logger().info(f"Calibratie: boven vakje {nummer}")
        if not arm.beweeg(x, y, bord.z_top + HOVER_HOOGTE, f"calibratie {nummer}"):
            return
        time.sleep(1.5)


def controleer_alle_vakjes(bord, log):
    """Controleert bij het opstarten dat alle 50 vakjes binnen de zone liggen."""
    ok = True
    for nummer in range(1, 51):
        x, y = bord.positie(nummer)
        for z in (bord.z_top + HOVER_HOOGTE, bord.z_top + PICK_HOOGTE):
            reden = controleer_zone(x, y, z)
            if reden:
                log.error(f"Vakje {nummer} ligt buiten de zone: {reden}")
                ok = False
    return ok


def main():
    rclpy.init()
    node = rclpy.create_node("dambord_move")
    log = node.get_logger()

    try:
        bord = Bord()
        log.info(
            f"Bord: vakje = {bord.vak * 100:.1f} cm, "
            f"bovenkant op z = {bord.z_top:.3f} m"
        )
        log.info(
            f"Veilige zone: x [{ZONE_X_MIN:.2f}, {ZONE_X_MAX:.2f}], "
            f"y [{ZONE_Y_MIN:.2f}, {ZONE_Y_MAX:.2f}], "
            f"z [{ZONE_Z_MIN:.2f}, {ZONE_Z_MAX:.2f}] m"
        )
        for n in (1, 5, 46, 50):
            x, y = bord.positie(n)
            log.info(f"  vakje {n}: x={x:.3f}, y={y:.3f}")

        if not controleer_alle_vakjes(bord, log):
            log.error("Instellingen kloppen niet met de zone. Ik stop.")
            return

        arm = Arm(node)

        if not arm.controleer_kinematica():
            return
        if not arm.gewrichten_ok():
            log.error("De arm staat al buiten de gewrichtslimieten. Zet hem eerst terug.")
            return

        heeft_schijf = False  # wisselt af tussen oppakken en neerzetten

        while True:
            invoer = input(
                "\nVakje (1-50), 'c' = calibratie, 'q' = stoppen: "
            ).strip().lower()

            if invoer == "q":
                break
            if invoer == "c":
                calibratie(arm, bord)
                if arm.geblokkeerd:
                    break
                continue

            try:
                nummer = int(invoer)
            except ValueError:
                print("Geef een getal van 1 tot 50 in.")
                continue
            if not 1 <= nummer <= 50:
                print("Het nummer moet tussen 1 en 50 liggen.")
                continue

            actie = "neergezet" if heeft_schijf else "opgepakt"
            if bezoek_vakje(arm, bord, nummer, actie):
                heeft_schijf = not heeft_schijf
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
