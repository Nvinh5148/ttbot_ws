# ttbot_ws

A ROS workspace for an autonomous robot using **Ackermann steering**.  
This project focuses on building a car-like robotic platform with modules for:  
- Steering & speed control  
- Sensor processing  
- Localization  
- Mapping  
- Navigation  
- System bringup & configuration

---

## 📦 Packages Overview

### `ttbot_bringup`
Launch files and system configurations for running the robot or simulation.  
Includes:
- Controller launch
- Sensor processing launch
- Navigation launch
- Shared parameters (YAML)

---

### `ttbot_controller`
Main control module for the Ackermann robot.  
Features:
- Ackermann steering angle control  
- Rear-wheel speed control  
- Motor/servo interface  
- Accepts commands from `/cmd_vel` or `/ackermann_cmd`  
- Publishes TF (base_link → odom)

---

### `ttbot_description`
URDF/Xacro description of the robot model:
- Body and chassis  
- Ackermann wheel configuration  
- LiDAR, IMU, and other sensors  
- Joint limits  
- Collision & visual geometry

Used for RViz visualization and Gazebo simulation.

---

### `ttbot_localization`
Localization and odometry fusion:
- Encoder + IMU processing  
- EKF/UKF (if using robot_localization)  
- Odometry publishing  
- TF tree: map → odom → base_link

---

## 🚗 Features
- ✓ Ackermann steering control  
- ✓ Speed control for drive wheels  
- ✓ Sensor processing (IMU, encoder, lidar depending on configuration)  
- ✓ Localization using odometry + filtering  
- ✓ Basic navigation support  
- ✓ Full URDF robot model  
- ✓ Clean bringup and ROS workspace structure  

---

## 📁 Workspace Structure (simplified)

