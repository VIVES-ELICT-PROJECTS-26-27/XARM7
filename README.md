# xArm7 ROS 2 Development Environment

A Docker-based development environment for the xArm7 robotic arm using ROS 2 Humble and MoveIt 2. The container can be launched either in servo mode or in planner mode. For both modes you can either control a simulation or the real robot. 

## Demo

### Planner
https://github.com/user-attachments/assets/cb43e891-b189-4561-9a3b-d466995449d2

> **Drag the interactive marker to set a goal pose, then click "Plan & Execute"**

### Servo

Moveit Servo facilitates realtime control of the robot arm. The servo accepts invidual joint velocities, desired velocity of the end effector or the desired pose of the end effector.   
Read the explanation on the [moveit page](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html).

## Prerequisites

* Ubuntu/Linux or Windows 10/11 with WSL2
* Docker
* For WSL2: WSLg (included in Windows 11, or Windows 10 build 21364+)

### Installation

```bash
# Build the container

docker compose build
```

### Optional: Add your own ROS packages
Packages under `packages/` are an overlay workspace for your own packages. Build them once inside the container, and again after you add a package. The result is kept for the next container:

```bash
docker exec -it hn-xarm7-sim-xarm7-ctl-run-<id> bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
cd /home/ubuntu/overlay_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the container
### Options
1. Automatic Path Planning
2. Servo controller for Direct Control
3. Use your own commands on an existing or new container

## Option 1: Planner
```
docker compose run --rm xarm7-ctl planner
```

Without an ip address after the planner parameter creates a simulator. It launches:

* **RViz2** - Visualization and motion planning interface

* **MoveIt 2** - Motion planning framework

* **ros2_control** - Robot controller interface

To connect to a physical xArm7:

```bash
docker compose run --rm xarm7-ctl planner <robot_ip>
```

For example:

```bash
docker compose run --rm xarm7-ctl planner 192.168.1.237
```

The computer running Docker must be able to reach the robot over the network.

```bash
ping 192.168.1.237
```

## Option 2: Servo
```
docker compose run --rm xarm7-ctl servo
```

Without an IP address, this starts a simulation. It launches:

* **RViz2** - Visualization

* **MoveIt Servo** - Streams Cartesian velocity to the arm

* **ros2_control** - Robot controller interface

To connect to a physical xArm7:

```bash
docker compose run --rm xarm7-ctl servo <robot_ip>
```

For example:

```bash
docker compose run --rm xarm7-ctl servo 192.168.1.237
```

The computer running Docker must be able to reach the robot over the network.

```bash
ping 192.168.1.237
```
In a second shell inside that container, hold a key to jog the arm:

```bash
docker exec -it hn-xarm7-sim-xarm7-ctl-run-<id> bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
python3 /home/ubuntu/src/keypad_ctrl.py
```

| Key | Motion |
|-----|--------|
| `8` / `2` | forward / backward |
| `4` / `6` | left / right |
| `-` / `+` | up / down |

`q` quits. Releasing a key stops that motion.

## Option 3: Your own bash shell
To start a ROS-configured Bash shell in a **new** container:

```bash
docker compose run --rm xarm7-ctl bash
```

You can also open a new shell in an **already running** container:

```bash
docker exec -it hn-xarm7-sim-xarm7-ctl-run-<id> bash

# Source ROS inside the new shell
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Optionally source your custom packages
source /home/ubuntu/overlay_ws/install/setup.bash

# Launch your python script (Example)
python3 /home/ubuntu/src/simple_move.py

# Optionally launch a custom ROS node
ros2 run keypad_ctrl keypad_ctrl
```

## Moving the Robot with the Planner

### Method 1: RViz Interactive Markers (Recommended)

1. In RViz, look for the **interactive marker** (colored sphere/arrows) at the robot's end effector

2. **Drag the marker** to set a target pose

3. Click **"Plan & Execute"** in the MotionPlanning panel (left side)

### Method 2: MoveIt Motion Planning Panel

1. In RViz's **MotionPlanning** panel:

   * Go to the **"Planning"** tab

   * Set **Goal State** to a predefined pose or "random valid"

   * Click **"Plan"** to preview the trajectory

   * Click **"Execute"** to move the robot

### Method 3: Command Line (ros2 action)

Open a terminal inside the container:

```bash
docker exec -it hn-xarm7-sim-xarm7-ctl-run-<id> bash
```

Then send a joint trajectory goal:

```bash
# Source the workspace
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Move to a specific joint configuration (7 joints, in radians)
ros2 action send_goal /xarm7_traj_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
      points: [
        { positions: [0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0], time_from_start: { sec: 3 } }
      ]
    }
  }"
```

### Method 4: Python Script

Python scripts can use ROS 2 and the xArm planner services to plan and execute robot motions.

## Useful Commands

```bash
# Enter an already running container
docker exec -it hn-xarm7-sim-xarm7-ctl-run-<id> bash

# List available topics
ros2 topic list

# View joint states
ros2 topic echo /joint_states

# List available actions
ros2 action list

# Check controller status
ros2 control list_controllers
```

## File Structure

```text
XARM7/

├── docker-compose.yml   # Container configuration
├── Dockerfile           # Image build instructions
├── cyclonedds.xml       # DDS configuration (optional)
├── entrypoint.sh        # Startup script
├── packages/            # ROS packages added to the overlay workspace
├── src/                 # User scripts / development code
└── README.md            # This file
```

## Troubleshooting

### RViz doesn't display / black screen
Check that the display variable is set:
```bash
echo $DISPLAY
```
The container must have access to the same X display as the host.
If RViz reports that it cannot connect to the display, allow local Docker containers to access the X server:

```bash
xhost +local:docker
```
The permission can later be revoked with:
```bash
xhost -local:docker
```
If necessary, software rendering can also be tested with:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Nodes crash with SIGABRT
* Check Docker has enough memory (Settings → Resources → 4GB+)
* Ensure `ipc: host` is set in docker-compose.yml

### Cannot connect to robot
Check that the robot is reachable from inside the container:

```bash
ping <robot_ip>
```

Check that ROS receives the physical robot state:

```bash
ros2 topic echo /joint_states
```

When using the real robot configuration, RViz should reflect the joint state reported by the physical robot.

## Stopping
If the container was started with `docker compose run --rm`, stop it with `Ctrl+C`.
To stop containers started by the Compose project:

```bash
docker compose down
```

Run this command from the directory containing `docker-compose.yml`.

## License
This project uses:
* [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) - xArm ROS 2 packages
* [MoveIt 2](https://moveit.ros.org/) - Motion planning framework
* [ROS 2 Humble](https://docs.ros.org/en/humble/) - Robot Operating System

