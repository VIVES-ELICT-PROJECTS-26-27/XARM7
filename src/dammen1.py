#!/usr/bin/env python3
"""Volledige reeks: naar de box gaan, attachen, naar een zelfgekozen X/Y/Z brengen en loslaten.

  0. Eindpositie vragen als bordvak (bv. F4), omgerekend naar X/Y;
     Z blijft de huidige Z (de hoogte waarop de box nu staat)
  1. Box uitlezen uit de planning scene (positie + afmetingen)
  2. Arm naar net boven de box (vanuit elke startpositie)
  3. Box attachen aan de end-effector (pas ná het bereiken van de box)
  4. Arm met de box naar de eindpositie; onderweg blijft Y rond TRAVEL_Y
  5. Box detachen en op de eindpositie in de wereld zetten
  6. Arm terug naar de home-positie

Eindpositie direct meegeven kan ook:  python3 script.py --end F4
"""

import argparse
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

BOX_ID = "Cylinder_0"       # naam van het object in RViz (box of cilinder), bv. "Cylinder_0"
CLEARANCE = 0.002      # afstand boven de bovenkant van de box (m)
APPROACH_EXTRA = 0.15  # tussenpunt ligt zoveel hoger dan het doel (m)
TCP_OFFSET = 0.0       # extra offset voor tool/gripper (m), indien nodig
TRAVEL_Y = 0.1         # tijdens het bewegen met de box blijft Y rond deze waarde (m)

# Home-positie: None = alle gewrichten op 0 (xArm-nulstand), of eigen waarden in radialen,
# bv. [0.0, -0.5, -0.5, 0.0, 1.0, 0.0]
HOME_JOINTS = None

WORLD_FRAME = "world"  # frame van de planning scene

EEF_LINK = "link_eef"                # link waar de box aan komt te hangen
TOUCH_LINKS = ["link_eef", "link6"]  # links die de box mogen raken (xarm7 -> link7)


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


def ask_square(preset=None):
    """Vraag een bordvak (bv. F4) in de terminal. Geeft (square, x, y) terug, x/y in meter.
    'preset' komt van --end."""
    while True:
        raw = preset if preset is not None else input(
            f"Eindpositie (vak {COLUMNS[0]}1 t/m {COLUMNS[-1]}{NUM_ROWS}, bv. F4): ")
        preset = None  # bij een fout daarna interactief opnieuw vragen

        xy = square_to_xy(raw)
        if xy is None:
            print(f"Ongeldig vak '{raw}'. Gebruik een letter {COLUMNS[0]}-{COLUMNS[-1]} "
                  f"en een cijfer 1-{NUM_ROWS}, bv. F4.")
            continue
        return raw.strip().upper(), xy[0], xy[1]


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
    """Geeft (primitive, x, y, top_z) van de box uit de planning scene, of None."""
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
    """Namen van alle andere objecten in de scene (bv. het damboard), zonder de box."""
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    objects = future.result().scene.world.collision_objects
    return [o.id for o in objects if o.id != BOX_ID]


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
    """Verwijder de box uit de wereld en hang hem aan de end-effector."""
    # De arm staat nu met de tool naar beneden CLEARANCE boven de box:
    # het box-middelpunt ligt op tool-z = CLEARANCE + TCP_OFFSET + halve hoogte.
    rel = Pose()
    rel.position.z = CLEARANCE + TCP_OFFSET + height / 2.0
    rel.orientation.x = 1.0
    rel.orientation.w = 0.0

    attached = AttachedCollisionObject()
    attached.link_name = EEF_LINK
    # De box mag de robotlinks én de andere objecten (het damboard) raken, anders ziet
    # MoveIt een botsing zodra de box eraan hangt en mislukt de planning.
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
    # het attachen wel lukt), dus controleer zelf of de box echt geattacht is.
    apply_scene(node, scene)
    return is_attached(node)


def detach_and_place(node, prim, x, y, z):
    """Detach de box en zet hem in de wereld met het middelpunt op x, y, z (recht, geen rotatie)."""
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--end", default=None, metavar="VAK",
                        help="eindpositie als bordvak (bv. --end F4)")
    args, _ = parser.parse_known_args()

    # 0. Eindpositie kiezen in de terminal (of via --end)
    square, end_x, end_y = ask_square(args.end)

    rclpy.init()
    node = rclpy.create_node("pick_box_and_go_home")
    log = node.get_logger()
    log.info(f"Eindpositie vak {square}: x={end_x * 1000:.1f} mm, y={end_y * 1000:.1f} mm "
             f"(z blijft de huidige z)")

    # 1. Box uitlezen
    box = get_box(node)
    if box is None:
        log.error(f"'{BOX_ID}' niet gevonden als box of cilinder in de planning scene (Publish Current Scene?).")
        rclpy.shutdown()
        return
    prim, x, y, top_z = box
    height = prim_height(prim)
    pick_z = top_z + CLEARANCE + TCP_OFFSET
    box_z = top_z - height / 2.0   # huidige z van het box-middelpunt
    log.info(f"Box '{BOX_ID}': x={x:.3f}, y={y:.3f}, bovenkant z={top_z:.3f} -> doel z={pick_z:.3f}")

    straight = node.create_client(PlanSingleStraight, "/xarm_straight_plan")
    pose_plan = node.create_client(PlanPose, "/xarm_pose_plan")
    execute = node.create_client(PlanExec, "/xarm_exec_plan")
    straight.wait_for_service()
    pose_plan.wait_for_service()
    execute.wait_for_service()

    # 2. Naar de box (vanuit elke startpositie)
    log.info("Naar de box...")
    if not go_to(node, log, straight, pose_plan, execute, x, y, pick_z):
        log.error("Gestopt, box niet geattacht.")
        rclpy.shutdown()
        return
    log.info("Arm staat bij de box.")

    # 3. Pas nu attachen
    if not attach_box(node, prim, height):
        log.error("Attach mislukt: box niet als geattacht gevonden in de planning scene.")
        rclpy.shutdown()
        return
    log.info(f"'{BOX_ID}' hangt nu aan '{EEF_LINK}'.")

    # 4. Naar de eindpositie met dezelfde tool-hoogte (huidige Z blijft behouden).
    #    Onderweg blijft Y rond TRAVEL_Y: eerst naar (end_x, TRAVEL_Y), dan naar (end_x, end_y).
    log.info(f"Naar de eindpositie, via y={TRAVEL_Y:.3f}...")
    if not go_to(node, log, straight, pose_plan, execute, end_x, TRAVEL_Y, pick_z):
        log.error("Tussenpunt niet bereikt. Box blijft geattacht.")
        rclpy.shutdown()
        return
    if abs(end_y - TRAVEL_Y) > 1e-3:
        if not go_to(node, log, straight, pose_plan, execute, end_x, end_y, pick_z):
            log.error("Eindpositie niet bereikt. Box blijft geattacht.")
            rclpy.shutdown()
            return
    log.info("Arm staat op de eindpositie.")

    # 5. Box detachen en op de eindpositie (zelfde z als eerst (0.01)) in de wereld zetten
    if not detach_and_place(node, prim, end_x, end_y, box_z):
        log.error("Detachen mislukt.")
        rclpy.shutdown()
        return
    log.info(f"'{BOX_ID}' is gedetacht en staat op x={end_x * 1000:.0f} mm, "
             f"y={end_y * 1000:.0f} mm, z={box_z * 1000:.0f} mm.")

    # 6. Terug naar de home-positie (alle gewrichten 0, tenzij HOME_JOINTS is ingesteld)
    home = HOME_JOINTS
    if home is None:
        current = read_current_joints(node)
        if current is None:
            log.warn("Kan /joint_states niet lezen: arm blijft op de eindpositie staan.")
            rclpy.shutdown()
            return
        home = [0.0] * len(current)

    joint = node.create_client(PlanJoint, "/xarm_joint_plan")
    joint.wait_for_service()
    jreq = PlanJoint.Request()
    jreq.target = [float(j) for j in home]
    log.info("Terug naar de home-positie...")
    if plan_and_execute(node, joint, jreq, execute):
        log.info("Klaar: arm staat in de home-positie.")
    else:
        log.error("Planning naar de home-positie mislukt.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()