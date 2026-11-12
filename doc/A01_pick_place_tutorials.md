# MoveIt pick-and-place tutorials

These tutorials demonstrate three levels of working with objects in MoveIt:

1. **Planning Scene ROS API** focuses only on scene bookkeeping: add an object,
   attach it to the gripper, detach it, and remove it. The arm does not move.
2. **Move Group Interface** introduces motion planning. The arm plans around an
   obstacle and later moves while carrying an attached object.
3. **MoveIt Task Constructor** builds a complete pick-and-place operation from
   stages such as approach, grasp, attach, lift, transport, place, detach, and
   retreat.

All three use MoveIt's simulated Panda arm, not the xArm7. Each demo needs two
host terminals connected to the same Docker container.

## One-time setup

From the repository root, build the image and the tutorial packages:

```bash
docker compose build
docker compose run --rm xarm7-ctl bash -lc \
  'cd /home/ubuntu/overlay_ws && colcon build --symlink-install'
```

The second command stores the overlay build in the `overlay_ws` Docker volume.
Its temporary container exits and is removed automatically when the build
finishes. Rebuild the overlay after changing tutorial C++ code or adding
packages.

## 1. Planning Scene ROS API

This is the smallest example. It adds a box to the planning scene, attaches it
to the Panda hand, detaches it, and removes it. It illustrates planning-scene
state changes without moving the arm.

1. Open terminal 1 in the cloned repository root and start MoveIt and RViz:

   ```bash
   docker compose run --rm xarm7-ctl \
     ros2 launch moveit2_tutorials move_group.launch.py
   ```

2. Wait until RViz has loaded.

3. Open terminal 2 in the same repository root and enter the running `xarm7-ctl`
   service container:

   ```bash
   docker compose exec xarm7-ctl bash
   ```

4. In the container shell in terminal 2, source the workspaces and launch the
   tutorial:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ubuntu/ros2_ws/install/setup.bash
   source /home/ubuntu/overlay_ws/install/setup.bash
   ros2 launch moveit2_tutorials planning_scene_ros_api_tutorial.launch.py
   ```

5. Advance the demo with **Next** in RViz's **RvizVisualToolsGui** panel. You
   can also select the RViz **Key Tool** and press `0` while RViz has focus.

## 2. Move Group Interface

This broader MoveGroupInterface example demonstrates pose, joint, constrained,
and Cartesian motion. It then adds a collision box, plans around it, attaches
an object to the wrist, moves the arm while carrying the object, and finally
detaches and removes the objects.

1. Open terminal 1 in the cloned repository root and start MoveIt and RViz:

   ```bash
   docker compose run --rm xarm7-ctl \
     ros2 launch moveit2_tutorials move_group.launch.py
   ```

2. Wait until RViz has loaded.

3. Open terminal 2 in the same repository root and enter the running `xarm7-ctl`
   service container:

   ```bash
   docker compose exec xarm7-ctl bash
   ```

4. In the container shell in terminal 2, source the workspaces and launch the
   tutorial:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ubuntu/ros2_ws/install/setup.bash
   source /home/ubuntu/overlay_ws/install/setup.bash
   ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py
   ```

5. Advance each stage with **Next** in **RvizVisualToolsGui**, or select the
   RViz **Key Tool** and press `0`.

## 3. MoveIt Task Constructor pick and place

This is the most complete example. It plans and executes a staged task that
approaches the object, opens and closes the gripper, attaches and lifts the
object, transports it, places and detaches it, retreats, and returns home.

1. Open terminal 1 in the cloned repository root and start the MTC-enabled
   MoveIt backend and RViz:

   ```bash
   docker compose run --rm xarm7-ctl \
     ros2 launch moveit2_tutorials mtc_demo.launch.py
   ```

2. Wait until RViz has loaded.

3. Open terminal 2 in the same repository root and enter the running `xarm7-ctl`
   service container:

   ```bash
   docker compose exec xarm7-ctl bash
   ```

4. In the container shell in terminal 2, source the workspaces and launch the
   pick-and-place task:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ubuntu/ros2_ws/install/setup.bash
   source /home/ubuntu/overlay_ws/install/setup.bash
   ros2 launch moveit2_tutorials pick_place_demo.launch.py
   ```

5. The node plans the task and executes its first solution. The RViz **Motion
   Planning Tasks** panel displays the individual stages and candidate
   solutions.

## Stopping a demo

Stop the tutorial in terminal 2 with `Ctrl+C`, then stop the backend in terminal
1 with `Ctrl+C`. The `--rm` option removes the container after the backend
exits.
