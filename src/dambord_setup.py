#!/usr/bin/env python3
"""
Setup voor zowel de echte xArm7 als offline ROS/RViz:
1. Wijst desgewenst de 4 zijden van het dambord aan om het bord recht te leggen.
2. Plaatst het dambord en de 40 schijven in MoveIt/RViz (wit: A1-J4, zwart: A7-J10).
"""

import os
import sys

try:
    import rclpy
except ImportError:
    _cmd = (
        "source /opt/ros/*/setup.bash 2>/dev/null; "
        "source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null; "
        f"exec {sys.executable} {' '.join(sys.argv)}"
    )
    os.execvp("bash", ["bash", "-c", _cmd])

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents

from dambord_move import (
    Arm,
    BASE_FRAME,
    BORD_BREEDTE,
    BORD_HOOGTE,
    BORD_RAND,
    BORD_X_NABIJ,
    BORD_Y_MIDDEN,
    EEF_FRAME,
    Gripper,
    Scene,
    SetupFout,
    activeer_echte_motoren,
    ik,
    klaar_q,
    limiet_fout,
    plaats_bord_en_schijven,
    verbind_met_arm,
)

WIJS_HOOGTE_BOVEN_BORD = 0.005  # 5 mm boven de bovenkant van het bord
HOVER_HOOGTE = 0.08            # 8 cm boven het bord tussen de zijden

X_VOOR = BORD_X_NABIJ - BORD_RAND
X_ACHTER = BORD_X_NABIJ + BORD_BREEDTE + BORD_RAND
X_MIDDEN = (X_VOOR + X_ACHTER) / 2.0

Y_RECHTS = BORD_Y_MIDDEN - (BORD_BREEDTE / 2.0 + BORD_RAND)
Y_LINKS = BORD_Y_MIDDEN + (BORD_BREEDTE / 2.0 + BORD_RAND)
Y_MIDDEN = BORD_Y_MIDDEN


def _zoek_bereikbare_hoogte(x, y, z_start, q_ref):
    for extra in (0.0, 0.01, 0.02, 0.04, 0.06):
        z_test = z_start + extra
        q = ik([x, y, z_test], q_ref)
        if q is None or limiet_fout(q) is not None:
            q = ik([x, y, z_test], klaar_q())
        if q is not None and limiet_fout(q) is None:
            return z_test
    return None


def _ruim_oude_objecten_op(scene):
    if not scene.beschikbaar:
        return
    huidige = scene.lees(
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    )
    if huidige is None:
        return

    weg_wereld = []
    for obj in huidige.world.collision_objects:
        c = CollisionObject()
        c.header.frame_id = BASE_FRAME
        c.id = obj.id
        c.operation = CollisionObject.REMOVE
        weg_wereld.append(c)

    weg_vast = []
    for att in huidige.robot_state.attached_collision_objects:
        a = AttachedCollisionObject()
        a.link_name = EEF_FRAME
        a.object.header.frame_id = BASE_FRAME
        a.object.id = att.object.id
        a.object.operation = CollisionObject.REMOVE
        weg_vast.append(a)

    if weg_wereld or weg_vast:
        ps = PlanningScene()
        ps.world.collision_objects = weg_wereld
        ps.robot_state.attached_collision_objects = weg_vast
        scene.pas_toe(ps)


def lijn_zijden_uit(arm, gripper, log, is_echt):
    standaard = "j" if is_echt else "n"
    keuze = input(
        f"\nDe 4 zijden van het bord aanwijzen om het bord recht te leggen? "
        f"[j = ja / n = overslaan] (standaard: {standaard}): "
    ).strip().lower()
    if not keuze:
        keuze = standaard
    if keuze != "j":
        return

    gripper.dicht()
    zijden = [
        ("Voorzijde (robotkant)", X_VOOR, Y_MIDDEN),
        ("Rechterzijde (-Y)", X_MIDDEN, Y_RECHTS),
        ("Achterzijde (overkant)", X_ACHTER, Y_MIDDEN),
        ("Linkerzijde (+Y)", X_MIDDEN, Y_LINKS),
    ]

    z_basis = BORD_HOOGTE + WIJS_HOOGTE_BOVEN_BORD
    z_hover = max(BORD_HOOGTE + HOVER_HOOGTE, z_basis + 0.05)

    while True:
        gestopt = False
        for idx, (naam, x, y) in enumerate(zijden, start=1):
            z_wijs = _zoek_bereikbare_hoogte(x, y, z_basis, arm.huidige_q())
            if z_wijs is None:
                log.warn(f"{naam} buiten bereik; overgeslagen.")
                continue

            if not arm.beweeg(x, y, max(z_hover, z_wijs + 0.04), f"boven {naam}"):
                gestopt = True
                break
            if not arm.beweeg(x, y, z_wijs, f"aanwijzen {naam}"):
                gestopt = True
                break

            actie = input(
                f"[{idx}/4] {naam} (x={x:.3f}, y={y:.3f}) -> "
                "[Enter] = volgende zijde, 's' = stoppen met uitlijnen: "
            ).strip().lower()
            arm.beweeg(x, y, max(z_hover, z_wijs + 0.04), f"omhoog {naam}")
            if actie == "s":
                gestopt = True
                break

        if gestopt:
            break
        if input("\nNog een controleronde langs de 4 zijden? [j/n] (standaard: n): ").strip().lower() != "j":
            break

    arm.eerst_omhoog_en_naar_klaarstand()
    gripper.open()


def main():
    rclpy.init()
    node = rclpy.create_node("dambord_setup")
    log = node.get_logger()

    try:
        ok, is_echt = verbind_met_arm(log)
        if not ok:
            return
        if is_echt and not activeer_echte_motoren(node):
            return

        scene = Scene(node)
        if not scene.beschikbaar:
            log.error("Planning scene services niet gevonden.")
            return

        _ruim_oude_objecten_op(scene)

        arm = Arm(node)
        if not arm.controleer_kinematica():
            return
        if not arm.eerst_omhoog_en_naar_klaarstand():
            log.error("Arm naar de beginpositie brengen mislukt.")
            return

        gripper = Gripper(node)
        lijn_zijden_uit(arm, gripper, log, is_echt)
        plaats_bord_en_schijven(scene, log)
    except SetupFout as e:
        log.error(str(e))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
