# Franka Gazebo
A project integrating Franka ROS 2 with the Gazebo simulator.

## Launch RVIZ + Gazebo

Launch an example which spawns RVIZ and Gazebo showing the robot:

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py
```

If you want to display another robot, you can define the arm_id:

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py arm_id:=fp3
```

If you want to start the simulation including the franka_hand:

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py load_gripper:=true franka_hand:='franka_hand'
```

## Interesting Notes

If you experience that Gazebo can't find your model files, try to include the workspace. E.g.

```bash
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/workspaces/src/
```
