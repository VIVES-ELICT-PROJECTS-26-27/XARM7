#!/usr/bin/env python3
"""Move an xArm to numbered squares on an international draughts board via ROS 2 MoveIt simulation."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanSingleStraight, PlanExec


# Bord- en offsetparameters
SQUARE_SIZE_MM = 43.0
BOARD_SQUARE_COUNT = 50
BOARD_SIDE_SQUARE_COUNT = 10
Z_HEIGHT_MM = 100.0

# Omrekenen naar meters voor ROS 2 / MoveIt
SQUARE_SIZE_M = SQUARE_SIZE_MM / 1000.0   # 0.043 m
Z_HEIGHT_M = Z_HEIGHT_MM / 1000.0         # 0.100 m

# --- POSITIE VAN HET NULPUNT (VAK 1) T.O.V. DE ROBOTBASIS ---
# 20 cm naar voren op de X-as:
BOARD_BASE_X_M = 0.210

# 20.5 cm op de Y-as.
# Omdat Y vanaf vak 1 alleen maar negatiever (-Y, naar rechts) wordt,
# moet vak 1 aan de linkerkant (+0.205) of in het midden (0.0) starten.
BOARD_BASE_Y_M = 0.190 


def square_to_board_offset(square: int) -> tuple[float, float]:
    """
    Bereken de X/Y verplaatsing t.o.v. vak 1 op basis van het bordrooster.
    Vak 1 = (0, 0)
    """
    if not 1 <= square <= BOARD_SQUARE_COUNT:
        raise ValueError(f"Vul een vaknummer van 1 t/m {BOARD_SQUARE_COUNT} in.")

    # Rijnummer van 0 t/m 9 (rij 0 = vak 1..5, rij 9 = vak 46..50)
    row = (square - 1) // 5
    col_idx = (square - 1) % 5

    # Op een dambord wisselt het startpatroon per rij:
    # Als vak 1 begint op kolom 0:
    # - In even rijen (0, 2, 4, 6, 8): kolommen 0, 2, 4, 6, 8 -> col = col_idx * 2
    # - In oneven rijen (1, 3, 5, 7, 9): kolommen 1, 3, 5, 7, 9 -> col = col_idx * 2 + 1
    # Controleer of jouw bord zo ligt, of precies omgekeerd (1 - row % 2):
    col = col_idx * 2 + (row % 2)

    # Bereken verplaatsing:
    # X loopt vooruit (0 t/m 9 stappen van 43 mm)
    # Y loopt naar rechts (negatief)
    offset_x = row * SQUARE_SIZE_M
    offset_y = -col * SQUARE_SIZE_M

    return offset_x, offset_y


class DraughtsMoveItNode(Node):
    def __init__(self):
        super().__init__('draughts_moveit_sim')

        self.plan_client = self.create_client(PlanSingleStraight, '/xarm_straight_plan')
        self.exec_client = self.create_client(PlanExec, '/xarm_exec_plan')

        self.get_logger().info("Wachten op MoveIt services...")
        self.plan_client.wait_for_service()
        self.exec_client.wait_for_service()
        self.get_logger().info("Verbonden met /xarm_straight_plan en /xarm_exec_plan.")

        self.current_square = None

    def plan_and_execute(self, x: float, y: float, z: float) -> bool:
        """Plan en executeer een rechtlijnig pad naar een X, Y, Z doelpositie."""
        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z

        # Vaste oriëntatie: gereedschap loodrecht naar beneden gericht
        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        # 1. Plan opstellen
        plan_req = PlanSingleStraight.Request()
        plan_req.target = target
        future_plan = self.plan_client.call_async(plan_req)
        rclpy.spin_until_future_complete(self, future_plan)

        res_plan = future_plan.result()
        if res_plan is None or not res_plan.success:
            return False

        # 2. Plan uitvoeren
        exec_req = PlanExec.Request()
        exec_req.wait = True
        future_exec = self.exec_client.call_async(exec_req)
        rclpy.spin_until_future_complete(self, future_exec)

        res_exec = future_exec.result()
        return res_exec is not None and res_exec.success

    def run_cli(self):
        while rclpy.ok():
            try:
                value = input("Naar welk vak? (1-50, q om te stoppen): ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                break

            if value in {"q", "quit", "stop"}:
                break

            try:
                target_square = int(value)
                offset_x_m, offset_y_m = square_to_board_offset(target_square)
            except ValueError as error:
                print(error)
                continue

            if self.current_square == target_square:
                print(f"De arm staat al boven vak {target_square}.")
                continue

            # Absolute positie van het doelvak berekenen
            target_x = BOARD_BASE_X_M + offset_x_m
            target_y = BOARD_BASE_Y_M + offset_y_m
            target_z = Z_HEIGHT_M

            self.get_logger().info(
                f"Plannen naar vak {target_square}: X={target_x:.3f}m, Y={target_y:.3f}m, Z={target_z:.3f}m "
                f"(offset: dx={offset_x_m:.3f}m, dy={offset_y_m:.3f}m)"
            )

            success = self.plan_and_execute(target_x, target_y, target_z)
            if success:
                self.current_square = target_square
                print(f"Arm staat nu boven vak {target_square} (z={Z_HEIGHT_MM:.0f} mm).")
            else:
                self.get_logger().error(
                    f"Kon geen geldig traject plannen/uitvoeren naar vak {target_square} (buiten bereik of botsing)."
                )


def main(args=None):
    rclpy.init(args=args)
    node = DraughtsMoveItNode()
    try:
        node.run_cli()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()