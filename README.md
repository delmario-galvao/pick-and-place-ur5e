# pick-and-place-ur5e
6D grasping strategies using UR5e and Robotiq 2F-85 in ROS/Gazebo with MoveIt: a comparative study between lateral and vertical grasps.

# UR5e Pick and Place – Comparative Grasping Strategies

This repository contains the simulation files and scripts for the final project of the *Robotic Systems* course, which focused on robotic object manipulation using 6D grasping strategies.

## Project Overview

Robotic object manipulation holds great relevance in industrial and domestic environments. In this context, this work compares **lateral** and **vertical** grasping strategies using the **UR5e manipulator** and the **Robotiq 2F-85 gripper**, simulated in **ROS Noetic**, **MoveIt**, and **Gazebo**.

The motion planning was performed using Jacobian pseudoinverse-based inverse kinematics. An Intel RealSense D435 RGB-D camera was also attached to the manipulator to enable future integration of object detection using computer vision.

## Key Results

| Strategy       | Path Length | Duration | Success Rate |
|----------------|-------------|----------|---------------|
| Lateral Grasp  | 6.54 m      | 43 s     | 60%           |
| Vertical Grasp | 1.66 m      | 35 s     | 100%          |

The vertical grasp proved to be more efficient and less complex in terms of trajectory and collision avoidance, especially due to the object’s position in the scene.

## Repository Structure

ur5e_mtc_ws/
├── src/
│ ├── ur5e_robotiq_85_mtc/ # Custom robot and gripper config
│ ├── gazebo_ros_link_attacher/ # Attachment plugin for grasping
│ ├── ur5_gripper_control/ # Gripper joint control interface
├── launch/ # Simulation launch files
├── config/ # MoveIt config files
└── README.md


## How to Run

1. **Dependencies**
   - ROS Noetic
   - MoveIt
   - Gazebo
   - `gazebo_ros_link_attacher` plugin
   - RealSense camera package (optional)

2. **Build the workspace**

```bash
cd ur5e_mtc_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```bash

3. **Launch Gazebo along with support for MoveIt! and RViz:**

roslaunch ur5e_robotiq_85_mtc_pkg setup.launch start_rviz:=true

4. **run the lateral pick and place pipeline:**

rosrun ur5e_robotiq_85_mtc_pkg side_pick.py

5. **Or run the topdown pick and place pipeline:**

rosrun ur5e_robotiq_85_mtc_pkg topdown_pick.py

## Future Work

Force analysis during grasping

Object detection via YOLO with RealSense RGB-D data

Obstacle avoidance using OctoMap

Real-robot deployment and testing

Authors:

Delmário dos Santos Gomes Galvão \
Federal University of Bahia (UFBA) \
ORCID: 0000-0002-0738-2473

Luciano dos Santos Gomes \
Federal University of Bahia (UFBA) \
Email: luciano.gomes@ufba.br
