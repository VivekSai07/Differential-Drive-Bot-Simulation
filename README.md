# Differential Drive Robot Simulation

This repository contains files and instructions to simulate a differential drive robot with laser mapping capabilities in Gazebo using ROS. The robot was designed in Fusion 360 and exported to URDF using the [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) tool. The simulation includes teleoperation control and laser sensor integration for mapping in RViz.

## Table of Contents
- [Introduction](#introduction)
- [Sketch](#sketch)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Output](#output)
- [Acknowledgments](#acknowledgments)

## Introduction
This project aims to provide a comprehensive simulation environment for a differential drive robot. The robot model is designed in Fusion 360 and converted to URDF. It is controlled using ROS and simulated in Gazebo, featuring a laser sensor for mapping purposes.

## Sketch
![Differential Drive Robot](https://github.com/VivekSai07/Differential-Drive-Bot-Simulation/blob/main/Sketch.jpg)

## Features
- Differential drive robot simulation in Gazebo.
- Teleoperation control using `teleop_twist_keyboard`.
- Laser sensor integration for mapping with `gmapping`.
- Visualizing the mapping process in RViz.

## Installation
### Prerequisites
- ROS (tested with Noetic)
- Gazebo
- RViz
- `teleop_twist_keyboard` package

### Steps
1. Clone the repository:
    ```bash
    cd catkin_ws/src
    git clone https://github.com/VivekSai07/Differential-Drive-Bot-Simulation.git
    cp TwoWheelRobot_description catkin_ws/src/
    ```

2. Build the workspace:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3. Install necessary ROS packages:
    ```bash
    sudo apt-get install ros-noetic-teleop-twist-keyboard ros-noetic-openslam-gmappin
    ```

## Usage
1. Launch the Gazebo simulation:
    ```bash
    roslaunch differential_drive_bot gazebo.launch
    ```

2. In a new terminal, start the teleoperation node:
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

3. In another terminal, start the `gmapping` node for mapping:
    ```bash
    rosrun TwoWheelRobot_description mapping.launch
    ```

4. Visualize the map in RViz:
    ```bash
    rviz
    ```

## Output
![Differential Drive Robot](https://github.com/VivekSai07/Differential-Drive-Bot-Simulation/blob/main/diffdrive.jpg)
![Differential Drive Robot](https://github.com/VivekSai07/Differential-Drive-Bot-Simulation/blob/main/diffdriveinworld.jpg)

## Acknowledgments
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) by syuntoku14
- ROS community and contributors
- Gazebo developers
