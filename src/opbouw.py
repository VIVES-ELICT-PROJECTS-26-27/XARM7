#!/usr/bin/env python3
"""Zet het dambord (STL-mesh) en de dammen (cilinders) in de MoveIt planning scene.

Gebruik:  python3 scene_opzetten.py
Het STL-bestand wordt automatisch gezocht (zie MESH_PADEN en ZOEKMAPPEN).
Een eigen pad meegeven kan met --mesh PAD.
Geen extra pakketten nodig: de STL wordt met een eigen lezer ingelezen (binair of ascii).

Bestaande objecten met dezelfde naam worden vervangen. Draai dit script vóór dammen2.py.
Bij het starten wordt gevraagd hoeveel dammen er komen: 1 (alleen A1), 20 (de eerste 20)
of 40 (alle). Direct meegeven kan met --aantal 1|20|40.
Kies RESET (of start met --reset) om het bord en alle dammen weer uit de scene te verwijderen.
De beginopstelling (BEGINVAKKEN) moet gelijk zijn aan die in dammen2.py.
"""

import argparse
import glob
import math
import os
import struct
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Point

WORLD_FRAME = "world"

# --- Dambord (waarden uit RViz: Change object pose/scale) ---
BORD_ID = "dambord_volledig.stl_0"
BORD_LINK = "world"                     # link waar het bord aan vastgemaakt wordt (zoals "Attach" in RViz)
BORD_POSITIE = (0.18, 0.22, 0.00)       # x, y, z in meter
BORD_ROTATIE = (0.0, 0.0, -1.57)        # roll, pitch, yaw in radialen
MESH_SCHAAL = 0.001                     # STL is in millimeter getekend -> naar meter (1.0 = al meter)
MESH_NAAM = "dambord_volledig.stl"
# Eerst deze vaste paden proberen (in de container); daarna wordt in ZOEKMAPPEN gezocht.
MESH_PADEN = [
    "/home/ubuntu/src/scene_objects/dambord_volledig.stl",
    "/home/ubuntu/src/dambord_volledig.stl",
    "/home/ubuntu/ros2_ws/scene_objects/dambord_volledig.stl",
]
ZOEKMAPPEN = ["/home/ubuntu", "/root", "/mnt", "/workspace"]

# --- Dammen (cilinders) ---
# naam in de scene -> vak waar hij in de beginopstelling staat
# Moet gelijk zijn aan BEGINVAKKEN in dammen2.py (de volgorde bepaalt de namen Cylinder_0, 1, 2, ...).
EERSTE_20 = ["A1", "C1", "E1", "G1", "I1",
             "A3", "B2", "B4", "C3", "D2", "D4", "E3", "F2", "F4", "G3", "H2", "H4", "I3", "J2", "J4"]
TWEEDE_20 = ["A7", "C7", "E7", "G7", "I7",
             "B8", "D8", "F8", "H8", "J8",
             "A9", "C9", "E9", "G9", "I9",
             "B10", "D10", "F10", "H10", "J10"]
BEGINVAKKEN = EERSTE_20 + TWEEDE_20
SCHIJVEN = {f"Cylinder_{i}": vak for i, vak in enumerate(BEGINVAKKEN)}   # alle 40 mogelijke dammen
SCHIJF_Z = 0.02                         # z van het middelpunt van elke dam in meter
CILINDER_ROTATIE = (0.0, 0.0, 0.0)
CILINDER_HOOGTE = 0.01                  # hoogte in meter
CILINDER_DIAMETER = 0.03                # diameter in meter
CILINDER_STRAAL = CILINDER_DIAMETER / 2.0   # MoveIt wil de straal (0.015 m)

# Bordindeling: moet gelijk zijn aan die in dammen2.py
A1_MM = (210.0, 196.0)
J10_MM = (602.5, -202.5)
COLUMNS = "ABCDEFGHIJ"
NUM_ROWS = 10
LETTERS_ALONG_Y = True


def rpy_naar_quaternion(roll, pitch, yaw):
    """Roll/pitch/yaw (rad, ROS-conventie) naar quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def maak_pose(positie, rotatie):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = positie
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = rpy_naar_quaternion(*rotatie)
    return pose


def zoek_mesh(opgegeven):
    """Geef het pad naar de STL: het opgegeven pad, een vast pad, of het eerste gevonden bestand."""
    if opgegeven:
        return opgegeven
    for pad in MESH_PADEN:
        if os.path.isfile(pad):
            return pad
    for map_ in ZOEKMAPPEN:
        gevonden = glob.glob(os.path.join(map_, "**", MESH_NAAM), recursive=True)
        if gevonden:
            return gevonden[0]
    return None


def lees_stl(pad):
    """Lees een binaire of ascii STL zonder extra pakketten.
    Geeft een lijst driehoeken terug, elk een tuple van 3 hoekpunten (x, y, z)."""
    with open(pad, "rb") as bestand:
        data = bestand.read()

    driehoeken = []
    if len(data) >= 84:
        aantal = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * aantal:   # binaire STL: 50 bytes per driehoek
            for i in range(aantal):
                waarden = struct.unpack_from("<12f", data, 84 + 50 * i)
                driehoeken.append((waarden[3:6], waarden[6:9], waarden[9:12]))
            return driehoeken

    # ascii STL
    punten = []
    for regel in data.decode("utf-8", errors="ignore").splitlines():
        delen = regel.split()
        if len(delen) == 4 and delen[0].lower() == "vertex":
            punten.append(tuple(float(w) for w in delen[1:]))
    for i in range(0, len(punten) - 2, 3):
        driehoeken.append((punten[i], punten[i + 1], punten[i + 2]))
    return driehoeken


def laad_mesh(pad, schaal):
    """Lees een STL-bestand en geef een shape_msgs/Mesh terug (gedeelde hoekpunten samengevoegd)."""
    mesh = Mesh()
    index_van_punt = {}
    for driehoek in lees_stl(pad):
        indices = []
        for punt in driehoek:
            sleutel = tuple(round(w, 6) for w in punt)
            if sleutel not in index_van_punt:
                index_van_punt[sleutel] = len(mesh.vertices)
                mesh.vertices.append(Point(x=float(punt[0]) * schaal,
                                           y=float(punt[1]) * schaal,
                                           z=float(punt[2]) * schaal))
            indices.append(index_van_punt[sleutel])
        if len(set(indices)) == 3:   # degenerate driehoeken overslaan
            mesh.triangles.append(MeshTriangle(vertex_indices=indices))
    return mesh


def bord_object(mesh):
    voorwerp = CollisionObject()
    voorwerp.id = BORD_ID
    voorwerp.header.frame_id = WORLD_FRAME
    voorwerp.pose = maak_pose(BORD_POSITIE, BORD_ROTATIE)
    voorwerp.meshes = [mesh]
    voorwerp.mesh_poses = [Pose()]   # mesh zit op de oorsprong van het object
    voorwerp.mesh_poses[0].orientation.w = 1.0
    voorwerp.operation = CollisionObject.ADD
    return voorwerp


def bord_vast_object(mesh):
    """Het bord als vastgemaakt object aan BORD_LINK (zoals Attach in RViz met link 'world')."""
    vast = AttachedCollisionObject()
    vast.link_name = BORD_LINK
    vast.touch_links = [BORD_LINK]
    vast.object = bord_object(mesh)
    return vast


def vak_naar_xy(vak):
    """Zet een vak als 'C1' om naar (x, y) in meter (zelfde berekening als in dammen2.py)."""
    letter, nummer = vak[0].upper(), int(vak[1:])
    f_kolom = COLUMNS.index(letter) / (len(COLUMNS) - 1)
    f_rij = (nummer - 1) / (NUM_ROWS - 1)
    (x0, y0), (x1, y1) = A1_MM, J10_MM
    if LETTERS_ALONG_Y:
        x = x0 + (x1 - x0) * f_rij
        y = y0 + (y1 - y0) * f_kolom
    else:
        x = x0 + (x1 - x0) * f_kolom
        y = y0 + (y1 - y0) * f_rij
    return x / 1000.0, y / 1000.0


def cilinder_object(naam, positie):
    vorm = SolidPrimitive()
    vorm.type = SolidPrimitive.CYLINDER
    vorm.dimensions = [CILINDER_HOOGTE, CILINDER_STRAAL]   # [hoogte, straal]

    voorwerp = CollisionObject()
    voorwerp.id = naam
    voorwerp.header.frame_id = WORLD_FRAME
    voorwerp.pose = maak_pose(positie, CILINDER_ROTATIE)
    voorwerp.primitives = [vorm]
    voorwerp.primitive_poses = [Pose()]
    voorwerp.primitive_poses[0].orientation.w = 1.0
    voorwerp.operation = CollisionObject.ADD
    return voorwerp


def namen_in_scene(node):
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    verzoek = GetPlanningScene.Request()
    verzoek.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    toekomst = client.call_async(verzoek)
    rclpy.spin_until_future_complete(node, toekomst)
    return [o.id for o in toekomst.result().scene.world.collision_objects]


def bord_losmaken_van_robot(node, log):
    """Is het bord al eerder vastgemaakt (bv. met de hand via Attach in RViz), dan wordt die
    oude versie eerst losgemaakt; daarna wordt het bord opnieuw vastgemaakt aan BORD_LINK."""
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    verzoek = GetPlanningScene.Request()
    verzoek.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    toekomst = client.call_async(verzoek)
    rclpy.spin_until_future_complete(node, toekomst)

    vastgemaakt = [a for a in toekomst.result().scene.robot_state.attached_collision_objects
                   if a.object.id == BORD_ID]
    if not vastgemaakt:
        return

    for a in vastgemaakt:
        log.info(f"'{BORD_ID}' was al vastgemaakt aan '{a.link_name}': wordt opnieuw vastgemaakt.")
    losmaken = []
    for a in vastgemaakt:
        item = AttachedCollisionObject()
        item.link_name = a.link_name
        item.object.id = BORD_ID
        item.object.operation = CollisionObject.REMOVE
        losmaken.append(item)

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = losmaken

    apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    apply_client.wait_for_service()
    apply_verzoek = ApplyPlanningScene.Request()
    apply_verzoek.scene = scene
    apply_toekomst = apply_client.call_async(apply_verzoek)
    rclpy.spin_until_future_complete(node, apply_toekomst)


RESET = "RESET"   # waarde die ask_aantal() teruggeeft als alles verwijderd moet worden


def ask_aantal(preset=None):
    """Vraag hoeveel dammen er geplaatst worden: 1 (alleen A1), 20 (de eerste 20) of 40 (alle).
    Geeft RESET terug als het bord en alle dammen verwijderd moeten worden."""
    while True:
        raw = preset if preset is not None else input(
            "Hoeveel dammen plaatsen?  1 = alleen A1,  20 = de eerste 20,  40 = alle 40,  "
            "RESET = bord en alle dammen verwijderen: ")
        preset = None  # bij een fout daarna interactief opnieuw vragen
        tekst = raw.strip().upper()
        if tekst in ("RESET", "R"):
            return RESET
        if tekst in ("1", "20", "40"):
            return int(tekst)
        print("Kies 1, 20, 40 of RESET.")


def pas_scene_toe(node, scene):
    """Stuur een scene-wijziging naar MoveIt. De 'success' van de service is niet altijd
    betrouwbaar, dus de aanroeper controleert zelf het resultaat."""
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    client.wait_for_service()
    verzoek = ApplyPlanningScene.Request()
    verzoek.scene = scene
    toekomst = client.call_async(verzoek)
    rclpy.spin_until_future_complete(node, toekomst)
    return toekomst.result() is not None and toekomst.result().success


def ruim_alles_op(node, log):
    """RESET: verwijder het bord en alle dammen uit de scene (ook als ze aan een link vastzitten)."""
    ids = [BORD_ID] + list(SCHIJVEN)

    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    verzoek = GetPlanningScene.Request()
    verzoek.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    toekomst = client.call_async(verzoek)
    rclpy.spin_until_future_complete(node, toekomst)
    vastgemaakt = [a for a in toekomst.result().scene.robot_state.attached_collision_objects
                   if a.object.id in ids]

    # 1. Vastgemaakte objecten eerst losmaken (ze komen dan terug in de wereld)
    if vastgemaakt:
        losmaken = []
        for a in vastgemaakt:
            log.info(f"'{a.object.id}' hangt aan '{a.link_name}': wordt losgemaakt.")
            item = AttachedCollisionObject()
            item.link_name = a.link_name
            item.object.id = a.object.id
            item.object.operation = CollisionObject.REMOVE
            losmaken.append(item)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = losmaken
        pas_scene_toe(node, scene)

    # 2. Alles uit de wereld verwijderen
    verwijderen = []
    for naam in ids:
        oud = CollisionObject()
        oud.id = naam
        oud.operation = CollisionObject.REMOVE
        verwijderen.append(oud)
    scene2 = PlanningScene()
    scene2.is_diff = True
    scene2.world.collision_objects = verwijderen
    pas_scene_toe(node, scene2)

    # 3. Controle
    time.sleep(0.5)
    resterend = [naam for naam in namen_in_scene(node) if naam in ids]
    if resterend:
        log.error(f"Nog in de scene: {resterend}")
    else:
        log.info("Reset klaar: het bord en alle dammen zijn verwijderd.")


def toon_bord(node, log):
    """Lees het bord terug uit de planning scene en toon frame en positie (controle)."""
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    verzoek = GetPlanningScene.Request()
    verzoek.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                     | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    toekomst = client.call_async(verzoek)
    rclpy.spin_until_future_complete(node, toekomst)
    scene = toekomst.result().scene

    for a in scene.robot_state.attached_collision_objects:
        if a.object.id == BORD_ID:
            p = a.object.pose.position
            log.info(f"Bord vastgemaakt aan link '{a.link_name}' "
                     f"(frame '{a.object.header.frame_id}'), "
                     f"positie ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
            return
    log.error("Bord is niet vastgemaakt aan een link.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh", default=None, help="pad naar dambord_volledig.stl (optioneel)")
    parser.add_argument("--aantal", default=None,
                        help="aantal dammen: 1, 20, 40 of reset (anders wordt het gevraagd)")
    parser.add_argument("--reset", action="store_true",
                        help="verwijder het bord en alle dammen uit de scene")
    args = parser.parse_args()

    aantal = RESET if args.reset else ask_aantal(args.aantal)

    rclpy.init()
    node = rclpy.create_node("scene_opzetten")
    log = node.get_logger()

    if aantal == RESET:
        ruim_alles_op(node, log)
        rclpy.shutdown()
        return

    mesh_pad = zoek_mesh(args.mesh)
    if mesh_pad is None:
        log.error(f"'{MESH_NAAM}' niet gevonden in de container. Kopieer het bestand erheen "
                  f"(docker cp) of geef het pad mee met --mesh.")
        rclpy.shutdown()
        return
    log.info(f"Mesh gevonden: {mesh_pad}")

    try:
        mesh = laad_mesh(mesh_pad, MESH_SCHAAL)
    except Exception as fout:
        log.error(f"Mesh inlezen mislukt ({mesh_pad}): {fout}")
        rclpy.shutdown()
        return
    log.info(f"Mesh ingelezen: {len(mesh.vertices)} hoekpunten, {len(mesh.triangles)} driehoeken.")
    breedte = [max(getattr(p, as_) for p in mesh.vertices) - min(getattr(p, as_) for p in mesh.vertices)
               for as_ in ("x", "y", "z")]
    log.info(f"Afmetingen van het bord: {breedte[0]:.3f} x {breedte[1]:.3f} x {breedte[2]:.3f} m "
             f"(schaal {MESH_SCHAAL})")

    bord_losmaken_van_robot(node, log)   # eerdere (handmatige) attach opruimen

    # Het bord gaat uit de wereld en wordt vastgemaakt aan link BORD_LINK ("Attach" in RViz)
    weghalen = CollisionObject()
    weghalen.id = BORD_ID
    weghalen.operation = CollisionObject.REMOVE
    voorwerpen = [weghalen]
    gekozen = list(SCHIJVEN.items())[:aantal]
    for naam, vak in SCHIJVEN.items():
        if (naam, vak) not in gekozen:   # dammen van een eerdere run die nu niet meer nodig zijn
            oud = CollisionObject()
            oud.id = naam
            oud.operation = CollisionObject.REMOVE
            voorwerpen.append(oud)
    for naam, vak in gekozen:
        x, y = vak_naar_xy(vak)
        log.info(f"Dam '{naam}' op vak {vak}: x={x * 1000:.1f} mm, y={y * 1000:.1f} mm, "
                 f"z={SCHIJF_Z * 1000:.0f} mm")
        voorwerpen.append(cilinder_object(naam, (x, y, SCHIJF_Z)))
    log.info(f"{len(gekozen)} dam(men) worden geplaatst.")

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = voorwerpen
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [bord_vast_object(mesh)]

    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    client.wait_for_service()
    verzoek = ApplyPlanningScene.Request()
    verzoek.scene = scene
    toekomst = client.call_async(verzoek)
    rclpy.spin_until_future_complete(node, toekomst)

    # 'success' van de service is niet altijd betrouwbaar: controleer de scene zelf
    time.sleep(0.5)
    namen = namen_in_scene(node)
    for naam, _vak in gekozen:
        if naam in namen:
            log.info(f"'{naam}' staat in de planning scene.")
        else:
            log.error(f"'{naam}' ontbreekt in de planning scene.")
    toon_bord(node, log)

    rclpy.shutdown()


if __name__ == "__main__":
    main()