# Autonomous Navigation Using LiDAR-Based Technology

## Overview
This project aims to develop a cost-effective autonomous navigation system using LiDAR technology. The focus is on designing an indigenous solution for applications in agriculture, disaster response, environmental monitoring, and construction. The system emphasizes affordability, educational value, and modularity.

---

## Problem Statement
Existing autonomous navigation solutions rely on expensive hardware and proprietary systems, limiting accessibility and customization. This project addresses these challenges by:
- Using off-the-shelf components and open-source technologies.
- Enabling customization of algorithms for specific applications.
- Making advanced robotics accessible for small industries and educational purposes.

---

## Objectives
1. Develop a robust autonomous navigation system.  
2. Integrate LiDAR sensors for real-time mapping and obstacle detection.  
3. Implement path planning algorithms for autonomous movement.  
4. Optimize SLAM and localization techniques for dynamic environments.  
5. Transition from simulation to real-world deployment.  

---

## Project Achievements
1. **Differential Drive Robot Simulation**: Modeled and implemented in Gazebo.  
2. **ROS2 Navigation Stack Integration**: Enables SLAM, localization, and path planning.  
3. **SLAM Implementation**: Utilized for mapping and localization in simulation.  
4. **Path Planning**: Leveraged Dijkstra's Algorithm and TEB Local Planner.  
5. **Obstacle Avoidance**: Enabled collision-free movement in simulation.  

---

## Hardware Requirements
1. **Chassis**: Differential drive platform.  
2. **LiDAR**: RP Lidar A1/A2 or similar.  
3. **Microcontroller**: Raspberry Pi 4 and Arduino.  
4. **Power Supply**: Rechargeable LiPo battery (e.g., 11.1V, 3S).  
5. **Motor Driver**: Motor Driver used is BTS7960 although the project supports (L298N, POLOLU_MC33926, POLOLU_VNH5019, ROBOGAIA Motor encoder shields as well)

---

## Software Tools
- **Gazebo**: Simulation environment.  
- **ROS2 Nav2**: Navigation stack for SLAM, localization, and path planning.  
- **Open-Source SLAM Algorithm**: `slam_toolbox` integrated with Nav2.  

---

## Testing Plan
1. **Simulation**:  
   - Evaluate `slam_toolbox` in Gazebo for mapping and localization.  
   - Test path planning in dynamic environments.  

2. **Hardware**:  
   - Conduct real-world tests for mapping, localization, and obstacle avoidance.  
   - Validate performance metrics such as localization drift and path efficiency.  

---

## Real-World Applications
1. **Disaster Response**: Navigate debris and collapsed structures.  
2. **Agriculture**: Automate navigation in fields.  
3. **Construction**: Inspect and transport materials.  
4. **Environmental Monitoring**: Traverse forests and wetlands for data collection.  

---

## Project Timeline
1. **Months 1–2**: Simulation and algorithm development.  
2. **Month 3**: Hardware assembly and integration.  
3. **Months 4–5**: Real-world testing and optimization.  
4. **Month 6**: Final validation and documentation.  

---

## Commands
### To flash the Arduino with the serial communication and motor driver code
   - Open the ROSArduinoBridge folder and open the ROSArduinoBridge.ino in ArduinoIDE
Uncomment the line of motor driver accordingly in the headers section. (L298N and BTS7960 share similar driver code so no need to comment them)
   - Flash the arduino using the IDE
   - Use the motor_driver.h file as reference to connect the circuit accordingly
### To check the motor driver and its encoder count
   - Connect the arduino to your computer 
   - Open your serial monitor to the connected arduino and check the encoder counts using `e` and enter
   - It should show 0 0 if the motor has not moved
   - Try using `m 200 200` to see both the motors move.
   - Now reset the encoder counts using `r` and enter
   - Attach a wheel to the motor shaft and manually move it 360 degrees
   - Check the encoder count again. That's the value of encoder_counts_per_rev for configuring our files

### Making changes according to the bot
   - Go to the articubot_one/config/my_controllers.yaml file and edit the `wheel_separation` and `wheel_radius` parameters according to your bot specifications
   - Do the same in diffdrive_arduino/bringup/config/diffbot_controllers.yaml file
   - Connect your arduino to Rpi and check its serial port using 
   ```
   ls -l /dev/ttyUSB*
   ```
   or
   ```
   ls -l /dev/ttyACM*
   ```
   - Go to the ros2_control/diffbot.ros2_control.xacro file and change the `<param nam='device'>` and `<param name='enc_counts_per_rev'>` tag according to the serial port the arduino is connected to and wheel encoder counts noted above. Also change params according to your need.
   - Go to the articubot_one/description/ros2_control.xacro and change the same params.
   - Go to robot_core.urdf.xacro and enter your bot specifications accordingly
   - Go to articubot_one/config/mapper_params_online_async.yaml file and change your `map_file_name` to the folder name and file name to your desired file and folder location. (Use this in every future save of the map) 
   - Connect your lidar and repeat the same steps as with arduino to find its serial port
   and change it in articubot_one/src/launch/rplidar.launch.py accordinhly

### To launch the bot
   1. Go to the project worspace on Pi and source it
   2. Launch the core robot transforms and hardware interface
   ```
   ros2 launch articubot_one launch_robot.launch.py
   ```
   3. In another interface on pi run the following to start publishing your Lidar Data
   ```
   ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=<Your serial port here> \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser_frame \
  -p angle_compensate:=true \
  -p scan_mode:=Standard
  ```
  (Currently the rplidar.launch.py is unable to do the same for some reason. It will be fixed in a future update)
   4. On the home workstation source the workspace and run
   ```
   ros2 launch articubot_one joystick.launch.py
   ```
   Hold down the right trigger and slowly move the bot using the left joystick. (Use small flicks to avoid sudden jerks)
   5. Launch the slam_toolbox to start mapping
   ```
   ros2 launch slam_toolbox online_async_launch.py
   ``` 
   4. Open rviz2 to view the map (assuming you are in root of workspace)
   ```
   rviz2 -d src/articubot_one/config/map.rviz
   ```
   You should see a map forming around the bot
   5. Slowly move the bot around the area to create the map. Avoid moving object at this stage to prevent weird obstacles on the map. When you are satisfied with the map. Go to rviz2 -> Panels -> Add new panel -> SlamToolboxPlugin. Give your map the name you edited in the earlier step in serialize map textbox and press `Serialize Map` button to save the map
   6. Close rviz2 and slam_toolbox running in the window. Instead launch
   ```
   ros2 launch articubot_one online_async.launch.py
   ```
   This will launch slam_toolbox in localization mode
   7. Open rviz2 again and you should see the saved map load up once again. Move the bot around a bit to let it recalibrate its postion in the map
   8. Run
   ```
   ros2 launch articubot_one navigation.launch.py\
   ```
   9. In Rviz2 go to Map -> topic -> Change to global_costmap. Also change color scheme to costmap
   10. Assign goal pose using goal pose button and then clicking and dragging on the map to assign a goal pose. If all went well the robot should start planning and following the goal.