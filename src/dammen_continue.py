#!/usr/bin/env python3
"""Schijf (box of cilinder) kiezen via het vak waar hij staat en naar een ander vak verplaatsen,
telkens opnieuw, tot S wordt ingevoerd.

Per ronde:
  1. Voorwerp uitlezen uit de planning scene (staat na een ronde op het vorige vak)
  2. Arm naar net boven het voorwerp (vanuit elke startpositie)
  3. Voorwerp attachen aan de end-effector (pas ná het bereiken ervan)
  4. Schijf OPTILLEN, rechtstreeks naar het gekozen vak (kortste weg), daar weer zakken
  5. Voorwerp detachen en op het vak in de wereld zetten
  6. Arm terug naar de home-positie (na elke zet)
Daarna wordt een nieuwe zet gevraagd. Typ START (+ Enter) om alle schijven terug te zetten
op hun beginvak, of S (+ Enter) om te stoppen.

Eerste zet direct meegeven kan ook:  python3 script.py --zet B4,C5
"""

import argparse
import math
import re

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from xarm_msgs.srv import PlanSingleStraight, PlanPose, PlanJoint, PlanExec

# Beginopstelling: naam van de schijf -> vak waar hij bij START naartoe terugkeert.
# Moet gelijk zijn aan BEGINVAKKEN in scene_opzetten.py.
EERSTE_20 = ["A1", "C1", "E1", "G1", "I1",
             "A3", "B2", "B4", "C3", "D2", "D4", "E3", "F2", "F4", "G3", "H2", "H4", "I3", "J2", "J4"]
TWEEDE_20 = ["A7", "C7", "E7", "G7", "I7",
             "B8", "D8", "F8", "H8", "J8",
             "A9", "C9", "E9", "G9", "I9",
             "B10", "D10", "F10", "H10", "J10"]
BEGINVAKKEN = EERSTE_20 + TWEEDE_20   # alle 40; START gebruikt alleen de dammen die in de scene staan
ORIGINELE_VAKKEN = {f"Cylinder_{i}": vak for i, vak in enumerate(BEGINVAKKEN)}
SCHIJF_IDS = list(ORIGINELE_VAKKEN)         # namen van alle schijven in de scene (RViz)
PARKEER_VAKKEN = ["A5", "C5", "E5", "G5", "I5"]   # tijdelijke plekken om een knoop bij START te ontwarren
START = "START"                             # waarde die ask_square() teruggeeft bij het commando start
SCHIJF_TOLERANTIE = 0.02                    # max. afstand (m) tussen een schijf en het opgegeven vak
BOX_ID = SCHIJF_IDS[0]  # de schijf die nu wordt verplaatst; wordt per zet gekozen in verplaats()
CLEARANCE = 0.002      # afstand boven de bovenkant van het voorwerp (m)
APPROACH_EXTRA = 0.15  # tussenpunt ligt zoveel hoger dan het doel (m)
TCP_OFFSET = 0.0       # extra offset voor tool/gripper (m), indien nodig
TRAVEL_Y = None        # None = kortste weg (rechte lijn); of bv. 0.1 om onderweg eerst via Y = 0,1 m te gaan
OPTILLEN = 0.030       # de schijf wordt zoveel opgetild tijdens het verplaatsen (m); een schijf is
                       # 10 mm hoog, dus dit laat ongeveer 20 mm ruimte boven andere schijven

# Home-positie: None = alle gewrichten op 0 (xArm-nulstand), of eigen waarden in radialen,
# bv. [0.0, -0.5, -0.5, 0.0, 1.0, 0.0]
HOME_JOINTS = None

WORLD_FRAME = "world"  # frame van de planning scene

EEF_LINK = "link_eef"                # link waar het voorwerp aan komt te hangen
TOUCH_LINKS = ["link_eef", "link6"]  # links die het voorwerp mogen raken (xarm7 -> link7)


# --- Schaakbord: vakken A1 t/m J10 (10 x 10) ---
# Middelpunten van twee hoekvakken (mm, in het base-frame van de robot).
A1_MM = (210.0, 196.0)
J10_MM = (602.5, -202.5)
COLUMNS = "ABCDEFGHIJ"   # letters
NUM_ROWS = 10            # cijfers 1..10
# True : letters (A..J) lopen langs de Y-as, cijfers (1..10) langs de X-as
#        (A links, J rechts gezien vanaf de robot; rij 1 het dichtst bij de robot).
# False: letters lopen langs de X-as, cijfers langs de Y-as.
LETTERS_ALONG_Y = True


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


def ask_zet(preset=None):
    """Vraag een zet als 'B4,C5' (van vak, naar vak).

    Geeft ((vak, x, y), (vak, x, y)) terug (x/y in meter), START bij het commando start,
    of None als de gebruiker S invoert (stoppen). 'preset' komt van --zet."""
    while True:
        raw = preset if preset is not None else input(
            "Zet (van,naar), bv. B4,C5   [START = alles terugzetten, S = stoppen]: ")
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
    """Stappen 1-5: schijf op van_vak oppakken, naar vak brengen, loslaten.

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
    # Alleen een rechte lijn omhoog (geen omweg via een gewrichtsplan), zodat de beweging
    # recht omhoog gaat voordat de schijf zijwaarts beweegt.
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
    """Stap 6: arm naar de home-positie (alle gewrichten 0, tenzij HOME_JOINTS is ingesteld)."""
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
    """START: zet alle schijven terug op hun beginvak (ORIGINELE_VAKKEN). True bij succes.

    Een schijf gaat pas naar zijn beginvak als dat vrij is. Staan de schijven in een knoop
    (elk beginvak is bezet door een andere schijf), dan wordt er eerst één op een vrij
    PARKEER_VAK gezet."""
    for _ in range(3 * len(ORIGINELE_VAKKEN)):
        posities = schijf_posities(node)
        if not posities:
            log.error("Geen schijven gevonden in de scene (draai eerst scene_opzetten.py).")
            return False

        # Schijven die niet op hun beginvak staan (alleen de dammen die in de scene staan:
        # je kunt er 1, 20 of 40 hebben geplaatst)
        fout = []
        for naam, begin_vak in ORIGINELE_VAKKEN.items():
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zet", default=None, metavar="VAN,NAAR",
                        help="eerste zet direct meegeven (bv. --zet B4,C5)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = rclpy.create_node("pick_box_and_go_home")
    log = node.get_logger()

    straight = node.create_client(PlanSingleStraight, "/xarm_straight_plan")
    pose_plan = node.create_client(PlanPose, "/xarm_pose_plan")
    execute = node.create_client(PlanExec, "/xarm_exec_plan")
    straight.wait_for_service()
    pose_plan.wait_for_service()
    execute.wait_for_service()

    # Blijf vragen welke schijf waarheen moet, tot S wordt ingevoerd
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
            rclpy.shutdown()
            return
        if resultaat is None:
            log.warn(f"Zet {van_vak} -> {vak} niet uitgevoerd. Kies een andere zet.")
        # 6. Na elke zet terug naar de home-positie
        if not ga_naar_home(node, log, execute):
            rclpy.shutdown()
            return

    log.info("Gestopt.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()