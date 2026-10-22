#!/usr/bin/env python3
"""Directe aansturing van de fysieke xArm via IP-adres."""

import sys
import time
from xarm.wrapper import XArmAPI

# ================= CONFIGURATIE =================
ROBOT_IP = "192.168.1.237"

# Bordparameters
BOARD_DIM_MM = 450.0                  # Bord is 45 x 45 cm
BOARD_SQUARES_PER_SIDE = 10
SQUARE_SIZE_MM = BOARD_DIM_MM / BOARD_SQUARES_PER_SIDE  # 45.0 mm per veld
BOARD_SQUARE_COUNT = 50

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
# ================================================


def square_to_board_coords(square: int) -> tuple[float, float]:
    """
    Nulpunt = midden van vak 1.
    Binnen een rij (1-5, 6-10, ...) loopt de nummering langs Y (van vak 1 naar de overkant, -Y).
    Elke volgende rij schuift een vak verder van de robot af (+X).
    """
    if not 1 <= square <= BOARD_SQUARE_COUNT:
        raise ValueError(f"Vul een vaknummer in van 1 t/m {BOARD_SQUARE_COUNT}.")

    row = (square - 1) // 5          # 0..9, loopt langs X (weg van de robot)
    col_idx = (square - 1) % 5       # 0..4, positie in de rij

    # Dambord: donkere velden wisselen per rij
    col = col_idx * 2 + (row % 2)    # 0..9, loopt langs Y

    target_x = BOARD_BASE_X_MM + row * SQUARE_SIZE_MM
    target_y = BOARD_BASE_Y_MM - col * SQUARE_SIZE_MM

    return target_x, target_y


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

    current_square = None

    try:
        while True:
            try:
                user_input = input("\nNaar welk vak? (1-50, q om te stoppen): ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                break

            if user_input in {"q", "quit", "stop"}:
                break

            if not user_input.isdigit():
                print("Vul een getal in tussen 1 en 50.")
                continue

            target_square = int(user_input)

            if target_square == current_square:
                print(f"De arm staat al boven vak {target_square}.")
                continue

            try:
                target_x, target_y = square_to_board_coords(target_square)
            except ValueError as err:
                print(err)
                continue

            target_z = Z_TRAVEL_HEIGHT_MM

            print(
                f"Beweeg naar vak {target_square} -> "
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
        arm.disconnect()


if __name__ == "__main__":
    main()