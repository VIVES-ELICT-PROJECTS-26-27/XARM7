#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
if [ -f /home/ubuntu/overlay_ws/install/setup.bash ]; then
    source /home/ubuntu/overlay_ws/install/setup.bash
fi

unset RMW_IMPLEMENTATION
unset CYCLONEDDS_URI

rm -rf ~/.ros/log/* 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 2

case "${1:-planner}" in

    planner)
        if [ -n "${2:-}" ]; then
            exec ros2 launch xarm_planner xarm7_planner_realmove.launch.py \
                robot_ip:="$2"
        fi
        exec ros2 launch xarm_planner xarm7_planner_fake.launch.py
        ;;

    servo)
        if [ -n "${2:-}" ]; then
            exec ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py \
                dof:=7 robot_ip:="$2"
        fi
        exec ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py dof:=7
        ;;

    *)
        exec "$@"
        ;;
esac