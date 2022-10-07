# Robotics Project - a.y. 2021-2022
_Robotics 2022 project at Politecnico di Milano._

## Description
The project is divided in two parts, each part detailed specifications are described in the provided presentations. The project has been implemented in the C++ language and developed using ROS Melodic on Ubuntu 18.04.

The instructions to run the project are provided in the 'instructions.txt' files present in the 'project' folders.

## The robot
### Omnidirectional robot
<img src="https://user-images.githubusercontent.com/61839160/194611799-1fa060d2-9e18-4bf9-adc7-d50c443839e7.png" width="400">

- **Mecanum wheels**
  - 4 wheels with rollers at 45°
- **Encoders on each wheel**
  - RPM (very noisy)
  - Ticks (more accurate)
- **Geometric parameters:**
  - Wheel radius (𝑟)
  - Wheel position along x (±𝑙)
  - Wheel position along y (±𝑤)

## Project 1
#### I. Compute odometry using appropriate kinematics
- Compute robot linear and angular velocities v, ⍵ from wheel encoders
- Compute odometry using both Euler and Runge-Kutta integration
  - ROS parameter for initial pose
- Calibrate (fine-tune) robot parameters to match ground truth
#### II. Compute wheel control speeds from v, ⍵
#### III. Add a service to reset the odometry to a specified pose (x,y,θ)
#### IV. Use dynamic reconfigure to select between integration method

## Project 2
#### I. Write launch files to create the map
#### II. Write launch file to perform amcl based localization
#### III. Write service to save an image with the map and the trajectory of the robot

## Group Members
- [__Marco Lacava__](https://github.com/LacavaMarco)
- [__Lorenzo Aicardi__](https://github.com/LorenzoAicardi)
