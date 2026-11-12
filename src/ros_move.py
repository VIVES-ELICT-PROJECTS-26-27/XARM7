#!/usr/bin/env python3
"""Directe aansturing van de fysieke xArm via IP-adres.

Vakken geef je op als coördinaat op het 10x10 dambord: kolom a-j en rij 1-10 (bijv. e5, a1, j10).
Volgens de damregels kan de arm alleen naar de speelbare (donkere) velden: a1 is donker,
daarna wisselt het per veld en per rij (50 speelbare velden).
Tijdens het bewegen worden de echte jointhoeken naar ROS gepubliceerd (RViz toont de arm mee).
"""

import math
import re
import sys
import threading
import time
from xarm.wrapper import XArmAPI

# ROS 2 (rclpy) of ROS 1 (rospy): wat beschikbaar is wordt automatisch gebruikt
try:
    import rclpy
    from sensor_msgs.msg import JointState
    ROS_VERSION = 2
except ImportError:
    try:
        import rospy
        from sensor_msgs.msg import JointState
        ROS_VERSION = 1
    except ImportError:
        ROS_VERSION = 0

# ================= CONFIGURATIE =================
ROBOT_IP = "192.168.1.237"

# Bordparameters
BOARD_DIM_MM = 450.0                  # Bord is 45 x 45 cm
BOARD_SQUARES_PER_SIDE = 10
SQUARE_SIZE_MM = BOARD_DIM_MM / BOARD_SQUARES_PER_SIDE  # 45.0 mm per veld

# Afstand van basis tot bordrand
BASE_TO_BOARD_EDGE_X_MM = 175.0       # 17.5 cm vanaf de robotbasis

# Coördinaten van het centrum van Vak 1:
# X: 175 mm + half vakje (22.5 mm) = 197.5 mm
BOARD_BASE_X_MM = 0.0
BOARD_BASE_Y_MM = 0.0
Z_TRAVEL_HEIGHT_MM = 150.0            # Vaste verplaatsingshoogte

# Gereedschapsoriëntatie (Roll, Pitch, Yaw): loodrecht naar beneden
ROLL_DEG = 180.0
PITCH_DEG = 0.0
YAW_DEG = 0.0

SPEED_VAL = 40.0
ACC_VAL = 300.0

# Vakcoördinaten: letter = kolom (a..j), cijfer = rij (1..10).
# a1 = midden van het oude vak 1 (het nulpunt). Rij 1 ligt het dichtst bij de robot,
# rijen lopen langs +X (weg van de robot); kolommen lopen vanaf a langs -Y.
# Klopt een richting niet met jouw bord? Zet de bijbehorende waarde op -1 / 1.
RANK_DIR_X = 1       # +1: rij 10 ligt verder van de robot dan rij 1
FILE_DIR_Y = -1      # -1: kolom j ligt op -Y ten opzichte van kolom a

# Alleen speelbare (donkere) velden toestaan, zoals bij dammen. a1 is donker: een veld is
# donker als (kolomnummer + rijnummer) even is, geteld vanaf 0 (a=0, rij 1=0).
ONLY_DARK_SQUARES = True

# ROS-visualisatie (RViz): echte jointhoeken van de arm publiceren
ROS_ENABLED = True                    # False = geen ROS gebruiken
JOINT_STATE_TOPIC = "/joint_states"   # topic waar robot_state_publisher / RViz naar luistert
JOINT_STATE_RATE_HZ = 30.0
JOINT_NAME_PREFIX = ""                # bijv. "xarm_" als je URDF dat gebruikt
# ================================================


FILES = "abcdefghijklmnopqrstuvwxyz"[:BOARD_SQUARES_PER_SIDE]    # a..j


def parse_square(text: str) -> tuple[int, int]:
    """'e5' -> (kolom-index 4, rij-index 4). Geeft ValueError bij een ongeldig vak."""
    m = re.fullmatch(r"([a-z])\s*(\d{1,2})", text.strip().lower())
    msg = (f"Ongeldig vak. Gebruik een letter a-{FILES[-1]} en een cijfer 1-{BOARD_SQUARES_PER_SIDE}, "
           f"bijvoorbeeld e5.")
    if not m:
        raise ValueError(msg)
    letter, rank = m.group(1), int(m.group(2))
    if letter not in FILES or not 1 <= rank <= BOARD_SQUARES_PER_SIDE:
        raise ValueError(msg)
    return FILES.index(letter), rank - 1


def is_playable(col: int, row: int) -> bool:
    """Donker (speelbaar) veld? a1 (col 0, row 0) is donker."""
    return (col + row) % 2 == 0


def draughts_number(col: int, row: int) -> int:
    """Officieel vaknummer 1..50 van een speelbaar veld (rij 1 = 1..5, rij 2 = 6..10, ...)."""
    return row * (BOARD_SQUARES_PER_SIDE // 2) + col // 2 + 1


def square_to_board_coords(col: int, row: int) -> tuple[float, float]:
    """
    Nulpunt = midden van a1.
    col 0..9 = letter a..j (langs Y), row 0..9 = cijfer 1..10 (langs X, weg van de robot).
    """
    target_x = BOARD_BASE_X_MM + RANK_DIR_X * row * SQUARE_SIZE_MM
    target_y = BOARD_BASE_Y_MM + FILE_DIR_Y * col * SQUARE_SIZE_MM
    return target_x, target_y


class RosJointPublisher:
    """Publiceert de echte jointhoeken (arm.angles, graden) als sensor_msgs/JointState
    in radialen, in een aparte thread zodat input() en de beweging er geen last van hebben."""

    def __init__(self, xarm):
        self.xarm = xarm
        self.n_joints = int(xarm.axis)
        self.names = [f"{JOINT_NAME_PREFIX}joint{i}" for i in range(1, self.n_joints + 1)]
        self._stop = threading.Event()
        self._thread = None
        self._node = None
        self._pub = None

    def start(self) -> bool:
        if ROS_VERSION == 0:
            print("ROS: rclpy/rospy niet gevonden (omgeving niet gesourced?). "
                  "Doorgaan zonder ROS-visualisatie.")
            return False
        try:
            if ROS_VERSION == 2:
                try:
                    # Geen eigen SIGINT-handler van rclpy: Ctrl+C blijft in dit script werken
                    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
                except (TypeError, AttributeError):
                    rclpy.init()
                self._node = rclpy.create_node("xarm_joint_state_publisher")
                self._pub = self._node.create_publisher(JointState, JOINT_STATE_TOPIC, 10)
            else:
                rospy.init_node("xarm_joint_state_publisher", anonymous=True,
                                disable_signals=True)
                self._pub = rospy.Publisher(JOINT_STATE_TOPIC, JointState, queue_size=10)
        except Exception as err:
            print(f"ROS: starten mislukt ({err}). Doorgaan zonder ROS-visualisatie.")
            return False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"ROS {ROS_VERSION}: jointhoeken worden gepubliceerd op {JOINT_STATE_TOPIC}")
        return True

    def _loop(self) -> None:
        period = 1.0 / JOINT_STATE_RATE_HZ
        while not self._stop.is_set():
            try:
                angles_deg = self.xarm.angles
                if angles_deg and len(angles_deg) >= self.n_joints:
                    msg = JointState()
                    if ROS_VERSION == 2:
                        msg.header.stamp = self._node.get_clock().now().to_msg()
                    else:
                        msg.header.stamp = rospy.Time.now()
                    msg.name = self.names
                    msg.position = [math.radians(a) for a in angles_deg[:self.n_joints]]
                    self._pub.publish(msg)
            except Exception:
                pass
            time.sleep(period)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if ROS_VERSION == 2 and self._node is not None:
            try:
                self._node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


def check(name: str, code: int) -> None:
    """Stop als een initialisatiestap faalt."""
    if code != 0:
        print(f"{name} mislukt (code {code}). "
              f"Fout: {arm.error_code}, waarschuwing: {arm.warn_code}")
        arm.disconnect()
        sys.exit(1)


def recover() -> None:
    """Reset fout/waarschuwing en zet de arm weer klaar voor beweging."""
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)


def main():
    global arm
    print(f"Verbinden met fysieke xArm op {ROBOT_IP}...")
    arm = XArmAPI(ROBOT_IP)
    time.sleep(1.0)

    if not arm.connected:
        print(f"Kan geen verbinding maken met {ROBOT_IP}. Controleer de kabel en het IP.")
        sys.exit(1)

    print(f"Firmware: {arm.version} | assen: {arm.axis}")
    print(f"Start: fout={arm.error_code}, waarschuwing={arm.warn_code}, "
          f"status={arm.state}, mode={arm.mode}")

    # Robot activeren en fouten resetten
    check("clean_warn", arm.clean_warn())
    check("clean_error", arm.clean_error())
    check("motion_enable", arm.motion_enable(enable=True))
    check("set_mode", arm.set_mode(0))     # 0 = Cartesische / Positiebesturing
    check("set_state", arm.set_state(0))   # 0 = Klaar voor beweging
    time.sleep(1.0)

    print(f"Huidige positie: {arm.get_position()}")
    print("Verbonden met de robot. Motoren zijn ingeschakeld.")

    ros_pub = None
    if ROS_ENABLED:
        ros_pub = RosJointPublisher(arm)
        ros_pub.start()

    current_square = None

    try:
        while True:
            try:
                user_input = input(f"\nNaar welk vak? (speelbaar veld, bijv. e5; q om te stoppen): ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                break

            if user_input in {"q", "quit", "stop"}:
                break

            try:
                col, row = parse_square(user_input)
                if ONLY_DARK_SQUARES and not is_playable(col, row):
                    speelbaar = ", ".join(f"{FILES[c]}{row + 1}" for c in range(len(FILES))
                                          if is_playable(c, row))
                    raise ValueError(f"{FILES[col]}{row + 1} is een licht veld en niet speelbaar. "
                                     f"Speelbaar in rij {row + 1}: {speelbaar}.")
                target_x, target_y = square_to_board_coords(col, row)
            except ValueError as err:
                print(err)
                continue

            target_square = f"{FILES[col]}{row + 1}"
            nr_txt = f" (nr {draughts_number(col, row)})" if is_playable(col, row) else ""

            if target_square == current_square:
                print(f"De arm staat al boven vak {target_square}.")
                continue

            target_z = Z_TRAVEL_HEIGHT_MM

            print(
                f"Beweeg naar vak {target_square}{nr_txt} -> "
                f"X: {target_x:.1f} mm, Y: {target_y:.1f} mm, Z: {target_z:.1f} mm"
            )

            # Limietcontrole
            try:
                lim_code, out_of_limit = arm.is_tcp_limit(
                    [target_x, target_y, target_z, ROLL_DEG, PITCH_DEG, YAW_DEG]
                )
                if lim_code == 0 and out_of_limit:
                    print("Dit punt ligt buiten het TCP-bereik van de arm, beweging overgeslagen.")
                    continue
            except Exception:
                pass

            if arm.error_code != 0 or arm.state in (4, 5):
                print(f"Arm in foutstatus (fout={arm.error_code}), herstellen...")
                recover()

            code = arm.set_position(
                x=target_x,
                y=target_y,
                z=target_z,
                roll=ROLL_DEG,
                pitch=PITCH_DEG,
                yaw=YAW_DEG,
                speed=SPEED_VAL,
                mvacc=ACC_VAL,
                wait=True
            )

            if code == 0 and arm.error_code == 0:
                current_square = target_square
                print(f"Arm staat nu boven vak {target_square}.")
            else:
                print(f"Beweging mislukt. Returncode: {code}, "
                      f"foutcode: {arm.error_code}, waarschuwing: {arm.warn_code}")
                recover()

    finally:
        print("Verbinding verbreken...")
        if ros_pub is not None:
            ros_pub.stop()
        arm.disconnect()


if __name__ == "__main__":
    main()