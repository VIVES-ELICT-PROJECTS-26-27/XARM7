#!/usr/bin/env python3
"""Dambord opbouwen + dammen spelen met de xArm, alles in één script.

Verloop:
  1. Scene opbouwen: vraagt hoeveel dammen (1 = alleen A1, 20 = de eerste 20, 40 = alle),
     zet het dambord (STL, vastgemaakt aan 'world') en de gekozen dammen in de planning scene.
  2. Dammen: telkens een zet vragen als 'B4,C5' (van vak, naar vak):
       - schijf oppakken, 30 mm optillen, rechtstreeks naar het doelvak, zakken, loslaten
       - arm terug naar de home-positie
     START = alle dammen terug op hun beginvak, S = stoppen.
  3. Bij S: arm naar home, daarna worden het bord en alle dammen uit de scene verwijderd.

Gebruik:  python3 dammen_totaal.py
Opties:   --aantal 1|20|40   aantal dammen direct meegeven
          --zet B4,C5        eerste zet direct meegeven
          --mesh PAD         eigen pad naar dambord_volledig.stl (anders wordt hij gezocht)
          --reset            alleen het bord en alle dammen uit de scene verwijderen en stoppen
"""

import argparse
import glob
import math
import os
import re
import struct
import time

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from xarm_msgs.srv import PlanSingleStraight, PlanPose, PlanJoint, PlanExec

WORLD_FRAME = "world"  # frame van de planning scene

# =====================================================================================
# Instellingen: scene
# =====================================================================================

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
# naam in de scene -> beginvak (de volgorde bepaalt de namen Cylinder_0, 1, 2, ...)
EERSTE_20 = ["A1", "C1", "E1", "G1", "I1",
             "A3", "B2", "B4", "C3", "D2", "D4", "E3", "F2", "F4", "G3", "H2", "H4", "I3", "J2", "J4"]
TWEEDE_20 = ["A7", "C7", "E7", "G7", "I7",
             "B8", "D8", "F8", "H8", "J8",
             "A9", "C9", "E9", "G9", "I9",
             "B10", "D10", "F10", "H10", "J10"]
BEGINVAKKEN = EERSTE_20 + TWEEDE_20
SCHIJVEN = {f"Cylinder_{i}": vak for i, vak in enumerate(BEGINVAKKEN)}   # alle 40 mogelijke dammen
SCHIJF_IDS = list(SCHIJVEN)             # namen van alle mogelijke schijven in de scene
SCHIJF_Z = 0.02                         # z van het middelpunt van elke dam in meter
CILINDER_ROTATIE = (0.0, 0.0, 0.0)
CILINDER_HOOGTE = 0.01                  # hoogte in meter
CILINDER_DIAMETER = 0.03                # diameter in meter
CILINDER_STRAAL = CILINDER_DIAMETER / 2.0   # MoveIt wil de straal (0.015 m)

# --- Bordindeling: vakken A1 t/m J10 (10 x 10) ---
# Middelpunten van twee hoekvakken (mm, in het base-frame van de robot).
A1_MM = (210.0, 196.0)
J10_MM = (602.5, -202.5)
COLUMNS = "ABCDEFGHIJ"   # letters
NUM_ROWS = 10            # cijfers 1..10
# True : letters (A..J) lopen langs de Y-as, cijfers (1..10) langs de X-as
#        (A links, J rechts gezien vanaf de robot; rij 1 het dichtst bij de robot).
# False: letters lopen langs de X-as, cijfers langs de Y-as.
LETTERS_ALONG_Y = True

# =====================================================================================
# Instellingen: dammen spelen
# =====================================================================================

PARKEER_VAKKEN = ["A5", "C5", "E5", "G5", "I5"]   # tijdelijke plekken om een knoop bij START te ontwarren
START = "START"                             # waarde die ask_zet() teruggeeft bij het commando start
SCHIJF_TOLERANTIE = 0.02                    # max. afstand (m) tussen een schijf en het opgegeven vak
BOX_ID = SCHIJF_IDS[0]  # de schijf die nu wordt verplaatst; wordt per zet gekozen in verplaats()
CLEARANCE = 0.002      # afstand boven de bovenkant van het voorwerp (m)
APPROACH_EXTRA = 0.15  # tussenpunt ligt zoveel hoger dan het doel (m)
TCP_OFFSET = 0.0       # extra offset voor tool/gripper (m), indien nodig
TRAVEL_Y = None        # None = kortste weg (rechte lijn); of bv. 0.1 om onderweg eerst via Y = 0,1 m te gaan
OPTILLEN = 0.030       # de schijf wordt zoveel opgetild tijdens het verplaatsen (m)

# Home-positie: None = alle gewrichten op 0 (xArm-nulstand), of eigen waarden in radialen,
# bv. [0.0, -0.5, -0.5, 0.0, 1.0, 0.0]
HOME_JOINTS = None

EEF_LINK = "link_eef"                # link waar het voorwerp aan komt te hangen
TOUCH_LINKS = ["link_eef", "link6"]  # links die het voorwerp mogen raken (xarm7 -> link7)


# =====================================================================================
# Bordvakken
# =====================================================================================

def square_to_xy(square):
    """Zet een vak als 'F4' om naar (x, y) in meter. Geeft None bij ongeldig vak."""
    m = re.fullmatch(r"([A-Za-z])(\d{1,2})", square.strip())
    if not m:
        return None
    letter, number = m.group(1).upper(), int(m.group(2))
    if letter not in COLUMNS or not 1 <= number <= NUM_ROWS:
        return None

    f_col = COLUMNS.index(letter) / (len(COLUMNS) - 1)   # 0..1 langs de letters
    f_row = (number - 1) / (NUM_ROWS - 1)                # 0..1 langs de cijfers
    (x0, y0), (x1, y1) = A1_MM, J10_MM
    if LETTERS_ALONG_Y:
        x = x0 + (x1 - x0) * f_row
        y = y0 + (y1 - y0) * f_col
    else:
        x = x0 + (x1 - x0) * f_col
        y = y0 + (y1 - y0) * f_row
    return x / 1000.0, y / 1000.0   # mm -> m


# =====================================================================================
# Deel 1: scene opbouwen / opruimen
# =====================================================================================

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

    losmaken = []
    for a in vastgemaakt:
        log.info(f"'{BORD_ID}' was al vastgemaakt aan '{a.link_name}': wordt opnieuw vastgemaakt.")
        item = AttachedCollisionObject()
        item.link_name = a.link_name
        item.object.id = BORD_ID
        item.object.operation = CollisionObject.REMOVE
        losmaken.append(item)

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = losmaken
    pas_scene_toe(node, scene)


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


def ask_aantal(preset=None):
    """Vraag hoeveel dammen er geplaatst worden: 1 (alleen A1), 20 (de eerste 20) of 40 (alle)."""
    while True:
        raw = preset if preset is not None else input(
            "Hoeveel dammen plaatsen?  1 = alleen A1,  20 = de eerste 20,  40 = alle 40: ")
        preset = None  # bij een fout daarna interactief opnieuw vragen
        tekst = raw.strip()
        if tekst in ("1", "20", "40"):
            return int(tekst)
        print("Kies 1, 20 of 40.")


def bouw_scene(node, log, aantal, mesh_opgegeven):
    """Zet het bord (vast aan BORD_LINK) en de eerste 'aantal' dammen in de scene. True bij succes."""
    mesh_pad = zoek_mesh(mesh_opgegeven)
    if mesh_pad is None:
        log.error(f"'{MESH_NAAM}' niet gevonden in de container. Kopieer het bestand erheen "
                  f"(docker cp) of geef het pad mee met --mesh.")
        return False
    log.info(f"Mesh gevonden: {mesh_pad}")

    try:
        mesh = laad_mesh(mesh_pad, MESH_SCHAAL)
    except Exception as fout:
        log.error(f"Mesh inlezen mislukt ({mesh_pad}): {fout}")
        return False
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
        x, y = square_to_xy(vak)
        log.info(f"Dam '{naam}' op vak {vak}: x={x * 1000:.1f} mm, y={y * 1000:.1f} mm, "
                 f"z={SCHIJF_Z * 1000:.0f} mm")
        voorwerpen.append(cilinder_object(naam, (x, y, SCHIJF_Z)))
    log.info(f"{len(gekozen)} dam(men) worden geplaatst.")

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = voorwerpen
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [bord_vast_object(mesh)]
    pas_scene_toe(node, scene)

    # 'success' van de service is niet altijd betrouwbaar: controleer de scene zelf
    time.sleep(0.5)
    namen = namen_in_scene(node)
    alles_ok = True
    for naam, _vak in gekozen:
        if naam in namen:
            log.info(f"'{naam}' staat in de planning scene.")
        else:
            log.error(f"'{naam}' ontbreekt in de planning scene.")
            alles_ok = False
    toon_bord(node, log)
    return alles_ok


def ruim_alles_op(node, log):
    """Verwijder het bord en alle dammen uit de scene (ook als ze aan een link vastzitten)."""
    ids = [BORD_ID] + SCHIJF_IDS

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
        log.info("Opgeruimd: het bord en alle dammen zijn verwijderd.")


# =====================================================================================
# Deel 2: dammen spelen
# =====================================================================================

def ask_zet(preset=None):
    """Vraag een zet als 'B4,C5' (van vak, naar vak).

    Geeft ((vak, x, y), (vak, x, y)) terug (x/y in meter), START bij het commando start,
    of None als de gebruiker S invoert (stoppen). 'preset' komt van --zet."""
    while True:
        raw = preset if preset is not None else input(
            "Zet (van,naar), bv. B4,C5   [START = alles terugzetten, "
            "S = stoppen en bord/dammen verwijderen]: ")
        preset = None  # bij een fout daarna interactief opnieuw vragen

        tekst = raw.strip().upper()
        if tekst == "S":
            return None
        if tekst == "START":
            return START

        delen = [d for d in re.split(r"[,;\s>-]+", tekst) if d]
        if len(delen) != 2:
            print("Geef twee vakken gescheiden door een komma, bv. B4,C5.")
            continue
        van_xy = square_to_xy(delen[0])
        naar_xy = square_to_xy(delen[1])
        if van_xy is None or naar_xy is None:
            print(f"Ongeldig vak in '{raw}'. Gebruik per vak een letter {COLUMNS[0]}-{COLUMNS[-1]} "
                  f"en een cijfer 1-{NUM_ROWS}, bv. B4,C5.")
            continue
        return (delen[0], van_xy[0], van_xy[1]), (delen[1], naar_xy[0], naar_xy[1])


def read_current_joints(node, timeout=3.0):
    """Lees de huidige gewrichtsstand (joint1..jointN) uit /joint_states, of None."""
    msgs = []
    sub = node.create_subscription(JointState, "/joint_states", msgs.append, 10)
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while not msgs and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if not msgs:
        return None

    pairs = []
    for name, pos in zip(msgs[-1].name, msgs[-1].position):
        m = re.fullmatch(r"joint(\d+)", name)
        if m:
            pairs.append((int(m.group(1)), pos))
    return [pos for _, pos in sorted(pairs)] or None


def prim_height(prim):
    """Hoogte (z) van een box of rechtopstaande cilinder, of None voor andere vormen."""
    if prim.type == SolidPrimitive.BOX:
        return prim.dimensions[SolidPrimitive.BOX_Z]
    if prim.type == SolidPrimitive.CYLINDER:
        return prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT]
    return None


def get_box(node):
    """Geeft (primitive, x, y, top_z) van het voorwerp uit de planning scene, of None."""
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    for obj in future.result().scene.world.collision_objects:
        if obj.id != BOX_ID:
            continue
        for prim, prim_pose in zip(obj.primitives, obj.primitive_poses):
            if prim_height(prim) is None:   # alleen box of cilinder
                continue
            x = obj.pose.position.x + prim_pose.position.x
            y = obj.pose.position.y + prim_pose.position.y
            z = obj.pose.position.z + prim_pose.position.z
            height = prim_height(prim)
            return prim, x, y, z + height / 2.0
    return None


def make_pose(x, y, z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 1.0   # tool naar beneden
    pose.orientation.w = 0.0
    return pose


def plan_and_execute(node, plan_client, request, execute_client):
    future = plan_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is None or not future.result().success:
        return False
    exec_req = PlanExec.Request()
    exec_req.wait = True
    future = execute_client.call_async(exec_req)
    rclpy.spin_until_future_complete(node, future)
    return future.result() is not None


def straight_to(node, straight, execute, pose):
    req = PlanSingleStraight.Request()
    req.target = pose
    return plan_and_execute(node, straight, req, execute)


def go_to(node, log, straight, pose_plan, execute, x, y, tool_z):
    """Ga met de tool (naar beneden) naar x, y, tool_z: eerst rechte lijn, anders via tussenpunt."""
    target = make_pose(x, y, tool_z)
    if straight_to(node, straight, execute, target):
        return True

    log.warn("Rechte lijn mislukt, via tussenpunt hoger boven het doel...")
    preq = PlanPose.Request()
    preq.target = make_pose(x, y, tool_z + APPROACH_EXTRA)
    if not plan_and_execute(node, pose_plan, preq, execute):
        log.error("Planning naar het tussenpunt mislukt (buiten bereik of botsing).")
        return False
    if not straight_to(node, straight, execute, target):
        log.error("Rechte lijn naar beneden mislukt.")
        return False
    return True


def apply_scene(node, scene):
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    client.wait_for_service()
    req = ApplyPlanningScene.Request()
    req.scene = scene
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result() is not None and future.result().success


def get_other_object_ids(node):
    """Namen van alle andere objecten in de scene (bv. het damboard), zonder het voorwerp."""
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    req = GetPlanningScene.Request()
    req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                 | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    scene = future.result().scene
    namen = [o.id for o in scene.world.collision_objects]
    # ook objecten die aan een link vastzitten (bv. het bord, vastgemaakt aan 'world')
    namen += [a.object.id for a in scene.robot_state.attached_collision_objects]
    return [naam for naam in namen if naam != BOX_ID]


def is_attached(node):
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    attached = future.result().scene.robot_state.attached_collision_objects
    return any(a.object.id == BOX_ID for a in attached)


def attach_box(node, prim, height):
    """Verwijder het voorwerp uit de wereld en hang het aan de end-effector."""
    # De arm staat nu met de tool naar beneden CLEARANCE boven het voorwerp:
    # het middelpunt ligt op tool-z = CLEARANCE + TCP_OFFSET + halve hoogte.
    rel = Pose()
    rel.position.z = CLEARANCE + TCP_OFFSET + height / 2.0
    rel.orientation.x = 1.0
    rel.orientation.w = 0.0

    attached = AttachedCollisionObject()
    attached.link_name = EEF_LINK
    # Het voorwerp mag de robotlinks én de andere objecten (het damboard) raken, anders
    # ziet MoveIt een botsing zodra het eraan hangt en mislukt de planning.
    attached.touch_links = TOUCH_LINKS + get_other_object_ids(node)
    attached.object.id = BOX_ID
    attached.object.header.frame_id = EEF_LINK
    attached.object.primitives = [prim]
    attached.object.primitive_poses = [rel]
    attached.object.operation = CollisionObject.ADD

    remove = CollisionObject()
    remove.id = BOX_ID
    remove.operation = CollisionObject.REMOVE

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [remove]
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]

    # De 'success' van de service is hier niet betrouwbaar (geeft soms False terwijl
    # het attachen wel lukt), dus controleer zelf of het voorwerp echt geattacht is.
    apply_scene(node, scene)
    return is_attached(node)


def detach_and_place(node, prim, x, y, z):
    """Detach het voorwerp en zet het in de wereld met het middelpunt op x, y, z (recht, geen rotatie)."""
    attached = AttachedCollisionObject()
    attached.link_name = EEF_LINK
    attached.object.id = BOX_ID
    attached.object.operation = CollisionObject.REMOVE

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]
    apply_scene(node, scene)
    if is_attached(node):
        return False

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    placed = CollisionObject()
    placed.id = BOX_ID
    placed.header.frame_id = WORLD_FRAME
    placed.primitives = [prim]
    placed.primitive_poses = [pose]
    placed.operation = CollisionObject.ADD

    scene2 = PlanningScene()
    scene2.is_diff = True
    scene2.world.collision_objects = [placed]
    apply_scene(node, scene2)
    return True


def schijf_posities(node):
    """Geeft {naam: (x, y)} van alle schijven die los in de wereld staan."""
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    verzoek = GetPlanningScene.Request()
    verzoek.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    toekomst = client.call_async(verzoek)
    rclpy.spin_until_future_complete(node, toekomst)

    posities = {}
    for voorwerp in toekomst.result().scene.world.collision_objects:
        if voorwerp.id not in SCHIJF_IDS:
            continue
        schijf_x = voorwerp.pose.position.x
        schijf_y = voorwerp.pose.position.y
        if voorwerp.primitive_poses:   # positie van de vorm komt erbovenop
            schijf_x += voorwerp.primitive_poses[0].position.x
            schijf_y += voorwerp.primitive_poses[0].position.y
        posities[voorwerp.id] = (schijf_x, schijf_y)
    return posities


def zoek_schijf(node, x, y):
    """Naam van de schijf die het dichtst bij (x, y) staat (binnen SCHIJF_TOLERANTIE), of None."""
    beste_naam, beste_afstand = None, SCHIJF_TOLERANTIE
    for naam, (schijf_x, schijf_y) in schijf_posities(node).items():
        afstand = math.hypot(schijf_x - x, schijf_y - y)
        if afstand <= beste_afstand:
            beste_naam, beste_afstand = naam, afstand
    return beste_naam


def controleer_zet(node, van_x, van_y, eind_x, eind_y):
    """Geeft None als de zet kan, anders een tekst met de reden waarom niet."""
    naam = zoek_schijf(node, van_x, van_y)
    if naam is None:
        return "Er staat geen schijf op het startvak."
    bezet = zoek_schijf(node, eind_x, eind_y)
    if bezet is not None and bezet != naam:
        return f"Het doelvak is bezet door '{bezet}'."
    return None


def verplaats(node, log, straight, pose_plan, execute, van_vak, van_x, van_y,
              vak, eind_x, eind_y):
    """Schijf op van_vak oppakken, optillen, naar vak brengen, zakken en loslaten.

    True  = gelukt.
    None  = zet niet uitgevoerd, er is niets vastgemaakt (waarschuwing; je kunt doorgaan).
    False = echte fout (de schijf kan nog aan de arm vastzitten)."""
    global BOX_ID

    # 0. Welke schijf staat op van_vak? En is het doelvak vrij?
    naam = zoek_schijf(node, van_x, van_y)
    if naam is None:
        log.warn(f"Geen schijf gevonden op vak {van_vak}.")
        return None
    BOX_ID = naam
    bezet = zoek_schijf(node, eind_x, eind_y)
    if bezet is not None and bezet != BOX_ID:
        log.warn(f"Vak {vak} is bezet door '{bezet}'.")
        return None
    log.info(f"Schijf '{BOX_ID}' op vak {van_vak} -> vak {vak}: "
             f"x={eind_x * 1000:.1f} mm, y={eind_y * 1000:.1f} mm")

    # 1. Voorwerp uitlezen (na een eerdere ronde staat het op het vorige vak)
    voorwerp = get_box(node)
    if voorwerp is None:
        log.warn(f"'{BOX_ID}' niet gevonden in de planning scene (Publish Current Scene?).")
        return None
    prim, x, y, top_z = voorwerp
    hoogte = prim_height(prim)
    pak_z = top_z + CLEARANCE + TCP_OFFSET
    midden_z = top_z - hoogte / 2.0   # huidige z van het middelpunt
    log.info(f"Voorwerp '{BOX_ID}': x={x:.3f}, y={y:.3f}, bovenkant z={top_z:.3f} "
             f"-> doel z={pak_z:.3f}")

    # 2. Naar het voorwerp (vanuit elke startpositie)
    log.info("Naar het voorwerp...")
    if not go_to(node, log, straight, pose_plan, execute, x, y, pak_z):
        log.warn("Kan de schijf niet bereiken; niets vastgemaakt.")
        return None
    log.info("Arm staat bij het voorwerp.")

    # 3. Pas nu attachen
    if not attach_box(node, prim, hoogte):
        log.error("Attach mislukt: voorwerp niet als geattacht gevonden in de planning scene.")
        return False
    log.info(f"'{BOX_ID}' hangt nu aan '{EEF_LINK}'.")

    # 4. Eerst OPTILLEN, zodat de schijf niet tegen andere schijven botst, dan naar het vak
    #    op verhoogde hoogte en daar weer zakken. De weg is rechtstreeks (kortste weg);
    #    met TRAVEL_Y kun je een omweg via een vaste Y afdwingen.
    reis_z = pak_z + OPTILLEN
    log.info(f"Optillen ({OPTILLEN * 1000:.0f} mm)...")
    if not straight_to(node, straight, execute, make_pose(x, y, reis_z)):
        log.error("Recht omhoog optillen mislukt. Voorwerp blijft geattacht.")
        return False
    if TRAVEL_Y is not None:   # optionele omweg via een vaste Y
        log.info(f"Eerst via y={TRAVEL_Y:.3f}...")
        if not go_to(node, log, straight, pose_plan, execute, eind_x, TRAVEL_Y, reis_z):
            log.error("Tussenpunt niet bereikt. Voorwerp blijft geattacht.")
            return False
    log.info(f"Rechte lijn naar vak {vak}...")
    if not go_to(node, log, straight, pose_plan, execute, eind_x, eind_y, reis_z):
        log.error("Eindpositie niet bereikt. Voorwerp blijft geattacht.")
        return False
    log.info("Zakken...")
    if not straight_to(node, straight, execute, make_pose(eind_x, eind_y, pak_z)):
        log.error("Recht omlaag zakken mislukt. Voorwerp blijft geattacht.")
        return False
    log.info("Arm staat op de eindpositie.")

    # 5. Detachen en op het vak (zelfde z als eerst) in de wereld zetten
    if not detach_and_place(node, prim, eind_x, eind_y, midden_z):
        log.error("Detachen mislukt.")
        return False
    log.info(f"'{BOX_ID}' staat op vak {vak} (x={eind_x * 1000:.0f} mm, "
             f"y={eind_y * 1000:.0f} mm, z={midden_z * 1000:.0f} mm).")
    return True


def ga_naar_home(node, log, execute):
    """Arm naar de home-positie (alle gewrichten 0, tenzij HOME_JOINTS is ingesteld)."""
    thuis = HOME_JOINTS
    if thuis is None:
        huidig = read_current_joints(node)
        if huidig is None:
            log.warn("Kan /joint_states niet lezen: arm blijft staan.")
            return False
        thuis = [0.0] * len(huidig)

    joint = node.create_client(PlanJoint, "/xarm_joint_plan")
    joint.wait_for_service()
    jreq = PlanJoint.Request()
    jreq.target = [float(j) for j in thuis]
    log.info("Terug naar de home-positie...")
    if plan_and_execute(node, joint, jreq, execute):
        log.info("Arm staat in de home-positie.")
        return True
    log.error("Planning naar de home-positie mislukt.")
    return False


def zet_terug(node, log, straight, pose_plan, execute):
    """START: zet alle schijven terug op hun beginvak (SCHIJVEN). True bij succes.

    Een schijf gaat pas naar zijn beginvak als dat vrij is. Staan de schijven in een knoop
    (elk beginvak is bezet door een andere schijf), dan wordt er eerst één op een vrij
    PARKEER_VAK gezet."""
    for _ in range(3 * len(SCHIJVEN)):
        posities = schijf_posities(node)
        if not posities:
            log.error("Geen schijven gevonden in de scene.")
            return False

        # Schijven die niet op hun beginvak staan (alleen de dammen die in de scene staan:
        # je kunt er 1, 20 of 40 hebben geplaatst)
        fout = []
        for naam, begin_vak in SCHIJVEN.items():
            if naam not in posities:
                continue
            begin_x, begin_y = square_to_xy(begin_vak)
            huidig_x, huidig_y = posities[naam]
            if math.hypot(huidig_x - begin_x, huidig_y - begin_y) > SCHIJF_TOLERANTIE:
                fout.append((naam, huidig_x, huidig_y, begin_vak, begin_x, begin_y))
        if not fout:
            log.info("Alle schijven staan op hun beginvak.")
            return True

        # Kies een schijf waarvan het beginvak vrij is
        keuze = None
        for punt in fout:
            vrij = not any(math.hypot(sx - punt[4], sy - punt[5]) <= SCHIJF_TOLERANTIE
                           for sx, sy in posities.values())
            if vrij:
                keuze = punt
                break

        if keuze is not None:
            naam, huidig_x, huidig_y, doel_vak, doel_x, doel_y = keuze
        else:
            # Knoop: parkeer de eerste foute schijf op een vrij tussenvak
            naam, huidig_x, huidig_y = fout[0][:3]
            for parkeer_vak in PARKEER_VAKKEN:
                park_x, park_y = square_to_xy(parkeer_vak)
                if not any(math.hypot(sx - park_x, sy - park_y) <= SCHIJF_TOLERANTIE
                           for sx, sy in posities.values()):
                    doel_vak, doel_x, doel_y = parkeer_vak, park_x, park_y
                    break
            else:
                log.error("Geen vrij parkeervak gevonden.")
                return False
            log.info(f"Alle beginvakken bezet: '{naam}' wordt tijdelijk op {doel_vak} geparkeerd.")

        huidige_plek = f"({huidig_x * 1000:.0f}, {huidig_y * 1000:.0f}) mm"
        if not verplaats(node, log, straight, pose_plan, execute,
                         huidige_plek, huidig_x, huidig_y, doel_vak, doel_x, doel_y):
            return False

    log.error("Terugzetten niet afgerond na het maximum aantal zetten.")
    return False


# =====================================================================================
# Hoofdprogramma
# =====================================================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh", default=None, help="pad naar dambord_volledig.stl (optioneel)")
    parser.add_argument("--aantal", default=None,
                        help="aantal dammen: 1, 20 of 40 (anders wordt het gevraagd)")
    parser.add_argument("--zet", default=None, metavar="VAN,NAAR",
                        help="eerste zet direct meegeven (bv. --zet B4,C5)")
    parser.add_argument("--reset", action="store_true",
                        help="alleen het bord en alle dammen uit de scene verwijderen en stoppen")
    args, _ = parser.parse_known_args()

    # Eerst vragen hoeveel dammen (nog vóór ROS start)
    aantal = None if args.reset else ask_aantal(args.aantal)

    rclpy.init()
    node = rclpy.create_node("dammen_totaal")
    log = node.get_logger()

    if args.reset:
        ruim_alles_op(node, log)
        rclpy.shutdown()
        return

    # --- Deel 1: scene opbouwen ---
    if not bouw_scene(node, log, aantal, args.mesh):
        log.error("Scene opbouwen mislukt: gestopt.")
        rclpy.shutdown()
        return

    straight = node.create_client(PlanSingleStraight, "/xarm_straight_plan")
    pose_plan = node.create_client(PlanPose, "/xarm_pose_plan")
    execute = node.create_client(PlanExec, "/xarm_exec_plan")
    straight.wait_for_service()
    pose_plan.wait_for_service()
    execute.wait_for_service()

    # --- Deel 2: dammen spelen, tot S wordt ingevoerd ---
    voorinstelling = args.zet
    while True:
        zet = ask_zet(voorinstelling)
        voorinstelling = None   # --zet geldt alleen voor de eerste zet
        if zet is None:
            break

        # START: alle schijven terug naar hun beginvak, daarna naar home
        if zet == START:
            log.info("START: alle schijven terug naar hun beginvak...")
            if not zet_terug(node, log, straight, pose_plan, execute):
                log.error("Fout: gestopt zonder op te ruimen (controleer de scene).")
                rclpy.shutdown()
                return
            if not ga_naar_home(node, log, execute):
                rclpy.shutdown()
                return
            continue

        van, naar = zet
        van_vak, van_x, van_y = van
        vak, eind_x, eind_y = naar
        # Eerst controleren: bij een ongeldige zet alleen een waarschuwing, niet afsluiten
        reden = controleer_zet(node, van_x, van_y, eind_x, eind_y)
        if reden is not None:
            log.warn(f"Zet {van_vak} -> {vak} niet mogelijk: {reden} Kies een andere zet.")
            continue

        resultaat = verplaats(node, log, straight, pose_plan, execute,
                              van_vak, van_x, van_y, vak, eind_x, eind_y)
        if resultaat is False:   # echte fout: de schijf kan nog vastzitten
            log.error("Fout: gestopt zonder op te ruimen (controleer de scene).")
            rclpy.shutdown()
            return
        if resultaat is None:
            log.warn(f"Zet {van_vak} -> {vak} niet uitgevoerd. Kies een andere zet.")
        # Na elke zet terug naar de home-positie
        if not ga_naar_home(node, log, execute):
            rclpy.shutdown()
            return

    # --- S: arm naar home, daarna bord en alle dammen verwijderen ---
    log.info("S: stoppen en opruimen...")
    ga_naar_home(node, log, execute)
    ruim_alles_op(node, log)
    log.info("Gestopt.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()