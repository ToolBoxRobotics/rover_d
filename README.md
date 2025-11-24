# rover_d


#
# Full Rover Workspace 

1. ``rover_description``
   - URDF/XACRO
   - Six-wheel chassis
   - Steering servos
   - Arm (5 DOF)
   - Sensors & TF tree
   - Meshes (I can use simple primitives or placeholders)

2. ``rover_bringup``
   - master_hw.launch
   - rosserial nodes
   - IMU/GPS/INA219 drivers
   - TCA9548A multiplexer driver
   - watchdog
   - diagnostics node

3. ```rover_base```
   - Ackermann kinematics
   - wheel controller
   - steering controller
   - differential map for 6-wheel skid/ackermann hybrid
   - Arduino interface library
   - motor driver (DRI0002) logic

4. ``rover_arm``
   - IK solver
   - joint controller node
   - A4988 stepper driver interface
   - calibration (limit switch) logic

5. ``rover_navigation``
   - EKF (robot_localization)
   - costmap config
   - global/local planner
   - Nav2-style plugins (ported for ROS1)
   - map server
   - AMCL / SLAM Toolbox (your choice)

6. ``rover_simulation``
   - Gazebo world
   - Gazebo plugins for motors
   - simulated sensors (IMU, GPS, Kinect)
   - simulated 6-wheel steering + control

7. ``rover_perception``   - Kinect RGB-D driver
   - OpenCV pipeline
   - obstacle detection
   - visual odometry / feature tracker

8. ``rover_moveit``
   - MoveIt setup assistant config
   - SRDF, kinematics.yaml
   - Move group launch

9. Extras
    - ``rover_web`` interface (React build directory)
    - ``scripts/flash.sh``, ``build.sh``, systemd units
    - Example mission behaviors (behavior trees)


#### Structure:
```text
catkin_ws_full/
  CMakeLists.txt              # top-level for catkin_workspace()
  src/
    rover_description/
      CMakeLists.txt
      package.xml
      urdf/
        rover.urdf.xacro      # 6-wheel chassis + 5-DOF arm + sensors
      launch/
        display.launch

    rover_base/
      CMakeLists.txt
      package.xml
      src/
        rover_kinematics_ackermann.py
      scripts/
        imu_node.py           # MPU6050 via TCA9548A
        ina219_node.py        # INA219 via TCA9548A
        cmd_vel_watchdog.py   # safety stop on /cmd_vel timeout
        diagnostics_node.py   # publishes /diagnostics

    rover_bringup/
      CMakeLists.txt
      package.xml
      launch/
        master_hw.launch      # full hardware bringup (Pi)
        state_estimation.launch
        teleop.launch

    rover_navigation/
      CMakeLists.txt
      package.xml
      config/
        ekf_odom.yaml
        ekf_map.yaml
        global_costmap.yaml
        local_costmap.yaml
        teb_local_planner.yaml
        amcl.yaml
      launch/
        navigation.launch     # map_server + amcl + move_base (TEB) + BT
      scripts/
        bt_autonomy.py        # Nav2-style mission behavior tree

    rover_perception/
      CMakeLists.txt
      package.xml
      launch/
        depth_pipeline.launch # depth → point cloud
        depth_to_scan.launch  # depth → LaserScan /scan
      scripts/
        vision_node.py        # OpenCV placeholder

    rover_arm/
      CMakeLists.txt
      package.xml
      src/
        arm_ik_node.py        # KDL-based IK for 5-DOF arm

    rover_simulation/
      CMakeLists.txt
      package.xml
      worlds/
        rover_world.world     # simple Gazebo world
      launch/
        sim.launch            # spawn rover in Gazebo

    rover_moveit/
      CMakeLists.txt
      package.xml
      README.md               # stub; ready for MoveIt Setup Assistant

```
