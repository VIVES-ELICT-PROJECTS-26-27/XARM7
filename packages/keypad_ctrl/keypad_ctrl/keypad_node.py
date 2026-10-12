#!/usr/bin/env python3
"""Jog the xArm end effector with the numeric keypad."""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListHardwareComponents
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


class Terminal:
    """Read individual keys from the controlling terminal."""

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("keypad control requires an interactive terminal")
        self._fd = sys.stdin.fileno()
        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)

    def read(self, timeout):
        if not select.select([self._fd], [], [], timeout)[0]:
            return None
        return os.read(self._fd, 1).decode(errors="ignore")


class KeypadController(Node):
    KEYS = {
        "8": (1.0, 0.0, 0.0),
        "2": (-1.0, 0.0, 0.0),
        "4": (0.0, 1.0, 0.0),
        "6": (0.0, -1.0, 0.0),
        "-": (0.0, 0.0, 1.0),
        "+": (0.0, 0.0, -1.0),
        "=": (0.0, 0.0, -1.0),
    }

    def __init__(self):
        super().__init__("keypad_ctrl")
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("hold_time", 0.15)
        self.declare_parameter("command_frame", "link_base")
        self.declare_parameter("twist_topic", "/servo_server/delta_twist_cmds")
        self.declare_parameter("start_service", "/servo_server/start_servo")
        self.declare_parameter("controller_manager", "/controller_manager/list_hardware_components")
        self.declare_parameter("trajectory_action", "/xarm7_traj_controller/follow_joint_trajectory")
        self.declare_parameter("joint_names", [f"joint{number}" for number in range(1, 8)])
        self.declare_parameter("ready_position", [0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0])

        self._linear_speed = self.get_parameter("linear_speed").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._hold_time = self.get_parameter("hold_time").value
        self._command_frame = self.get_parameter("command_frame").value
        self._start_service = self.get_parameter("start_service").value
        self._controller_manager = self.get_parameter("controller_manager").value
        self._trajectory_action = self.get_parameter("trajectory_action").value
        self._joint_names = list(self.get_parameter("joint_names").value)
        self._ready_position = list(self.get_parameter("ready_position").value)

        twist_topic = self.get_parameter("twist_topic").value
        self._publisher = self.create_publisher(TwistStamped, twist_topic, 10)
        self._motion = (0.0, 0.0, 0.0)
        self._motion_until = 0.0

    def run(self):
        if self._is_simulation():
            self._move_to_ready_position()
        self._start_servo()
        print("8/2 forward/back  4/6 left/right  -/+= up/down  q quits")

        period = 1.0 / self._publish_rate
        try:
            with Terminal() as keyboard:
                while rclpy.ok():
                    self._handle_key(keyboard.read(period))
                    self._publish_motion()
        except KeyboardInterrupt:
            pass
        finally:
            self._publish((0.0, 0.0, 0.0))

    def _is_simulation(self):
        client = self.create_client(ListHardwareComponents, self._controller_manager)
        client.wait_for_service()
        future = client.call_async(ListHardwareComponents.Request())
        rclpy.spin_until_future_complete(self, future)
        return any("FakeSystemHardware" in item.class_type for item in future.result().component)

    def _move_to_ready_position(self):
        client = ActionClient(self, FollowJointTrajectory, self._trajectory_action)
        client.wait_for_server()

        point = JointTrajectoryPoint(positions=self._ready_position)
        point.time_from_start.sec = 2
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [point]

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not result.accepted:
            raise RuntimeError("ready-position trajectory was rejected")

        future = result.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("failed to reach the ready position")

    def _start_servo(self):
        client = self.create_client(Trigger, self._start_service)
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"service {self._start_service} is unavailable")

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message or "MoveIt Servo failed to start")

    def _handle_key(self, key):
        if key in ("q", "Q", "\x1b"):
            raise KeyboardInterrupt
        if key in self.KEYS:
            self._motion = self.KEYS[key]
            self._motion_until = time.monotonic() + self._hold_time

    def _publish_motion(self):
        motion = self._motion if time.monotonic() < self._motion_until else (0.0, 0.0, 0.0)
        self._publish(tuple(axis * self._linear_speed for axis in motion))

    def _publish(self, velocity):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._command_frame
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = velocity
        self._publisher.publish(message)


def main():
    rclpy.init()
    controller = KeypadController()
    try:
        controller.run()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
