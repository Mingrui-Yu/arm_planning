# Studying Sampling-Based Robotic Arm Planning

## Introduction

When I was trying to study the robotic arm planning, I found that most of the courses / tutorials / slides focus on sampling-based planning algorithms such as basic PRMs and RRTs in general environments and use 2-D planning problems and results for illustration. Few materials discuss about the specific settings of the planning for high-DoF robotic arms. Thus, I briefly review the literature of robotic arm planning, and re-implement some representative and easy-to-understand algorithms for learning and practice. Each of the algorithms is published on top journals or conferences and has been cited more than (around) 100 times. 

These algorithms include:
* RRT
* RRTConnect (Bi-directional RRT)
* JTRRT
* IKRRT
* ATACE
* CBiRRT

In this repository, I write the source code for the main planning procedures, and use MoveIt! for the planning scene setup, robotic kinematics solving, and collision checking, which makes it more convenient to install the dependencies.

Please note that this repository is not a library for general usage, but may be helpful for learners to understand the details of the arm planners. 


## Dependencies
These dependencies should be properly installed ahead.
* ROS noetic
* MoveIt!
* Eigen3

## Installation

Clone the repositry:
```bash
$ git clone https://github.com/Mingrui-Yu/arm_planning.git
```

Clone the third-party packages:
```bash
$ cd arm_planning/ws_catkin/src

$ git clone https://github.com/THU-DA-Robotics/universal_robot.git -b calibration_devel
$ git clone https://github.com/THU-DA-Robotics/Universal_Robots_ROS_Driver.git
$ git clone https://github.com/THU-DA-Robotics/robotiq.git -b noetic-devel
$ git clone https://github.com/THU-DA-Robotics/dual_ur.git -b noetic_devel
```

Automatically install other dependencies:
```bash
$ cd arm_planning/ws_catkin
$ rosdep install --from-paths src --ignore-src -y
```

Build the catkin workspaces:
```bash
$ cd arm_planning/ws_catkin
$ catkin_make -j4
```

## Usage

In the single-arm planning, I use a 7-DoF Franka Emilka Panda arm.
In the dual-arm, I use two 6-DoF UR5 arms installed on a base holder (the robot in [our lab](https://github.com/THU-DA-Robotics)).

### Single-arm planning with joint-space goals

The planning goals are specified as the goal arm joint positions. The RRT and RRTConnect are tested.

First,
```bash
$ roslaunch arm_planning panda_sim_prepare.launch
```
Then,
```bash
$ rosrun arm_planning test_2
```

<p align="center">
  <img width="50%" src="docs/single_arm_joint_goal.gif">
</p>


### Single-arm planning with task-space goals

The planning goals are specified as the goal poses of the tool center point (TCP). The JTRRT and IKRRT are tested.

First, 
```bash
$ roslaunch arm_planning panda_sim_prepare.launch
```
Then,
```bash
$ rosrun arm_planning test_3
```

<p align="center">
  <img width="50%" src="docs/single_arm_task_goal.gif">
</p>

### Single-arm planning with task-space goals and constraints
The planning goals are specified as the goal poses of the tool center point (TCP). Moreover, the poses of the TCP are also constrained during the paths (e.g., Z-axis down). The ATACE and CBiRRT are tested.

First, 
```bash
$ roslaunch arm_planning panda_sim_prepare.launch
```
Then,
```bash
$ rosrun arm_planning test_4
```

<p align="center">
  <img width="50%" src="docs/single_arm_task_constraint.gif">
</p>


### Dual-arm planning with closed-chain constraints
During the path, the two arms should bimanually grasp the object, which is so-called a closed-chain constraint. The planning goals can be specified as the goal joint positions (more efficient) or goal TCP poses (less efficient). The CBiRRT is tested.

First, 
```bash
$ roslaunch dual_arm_planning dual_ur_sim_prepare.launch
```
If using joint-space goals:
```bash
$ rosrun dual_arm_planning dual_arm_planning_test_3
```
Or if using task-space goals:
```bash
$ rosrun dual_arm_planning dual_arm_planning_test_4
```

<p align="center">
  <img width="50%" src="docs/dual_arm_constraint.gif">
</p>