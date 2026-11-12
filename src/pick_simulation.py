#!/usr/bin/env python3
"""
Simuleert een 'oppak'-beweging met de xArm:
  1. naar positie boven het object
  2. recht omlaag (alsof hij iets pakt)
  3. korte pauze (grijper dicht)
  4. recht weer omhoog
"""

import time

import rclpy
from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanSingleStraight, PlanExec

# --- Instellingen (in meters) ---
X = 0.4
Y = 0.0
Z_BOVEN = 0.3     # veilige hoogte
Z_ONDER = 0.15    # hoogte waar het "object" ligt
PAUZE = 1.0       # seconden wachten onderaan (grijper dicht)


def maak_pose(x, y, z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    # Zelfde oriëntatie als in je originele script (grijper wijst naar beneden)
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    return pose


def beweeg_naar(node, plan, execute, pose, naam):
    """Plan een rechte lijn naar 'pose' en voer die uit. Geeft True bij succes."""
    node.get_logger().info(f"Plannen: {naam}")

    req = PlanSingleStraight.Request()
    req.target = pose

    future = plan.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None or not future.result().success:
        node.get_logger().error(f"Plannen mislukt: {naam}")
        return False

    node.get_logger().info(f"Uitvoeren: {naam}")
    exec_req = PlanExec.Request()
    exec_req.wait = True

    future = execute.call_async(exec_req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None or not future.result().success:
        node.get_logger().error(f"Uitvoeren mislukt: {naam}")
        return False

    return True


def main():
    rclpy.init()
    node = rclpy.create_node("pick_simulation")

    plan = node.create_client(PlanSingleStraight, "/xarm_straight_plan")
    execute = node.create_client(PlanExec, "/xarm_exec_plan")

    plan.wait_for_service()
    execute.wait_for_service()

    boven = maak_pose(X, Y, Z_BOVEN)
    onder = maak_pose(X, Y, Z_ONDER)

    try:
        # 1. Naar positie boven het object
        if not beweeg_naar(node, plan, execute, boven, "boven object"):
            return

        # 2. Recht omlaag
        if not beweeg_naar(node, plan, execute, onder, "omlaag"):
            return

        # 3. "Oppakken" - hier kan later een echte grijper-aanroep komen
        node.get_logger().info("Object oppakken (grijper dicht)...")
        time.sleep(PAUZE)

        # 4. Recht weer omhoog
        if not beweeg_naar(node, plan, execute, boven, "omhoog"):
            return

        node.get_logger().info("Klaar!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
