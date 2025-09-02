# Robot Core


## How to use

1. Enter
  ```sh
  docker-compose run --rm robot_core bash
  # or
  docker exec -it robot_core bash
  ```
2. Launch: ros2 launch robot_launch teleop.launch.py
3. Test: ros2 launch robot_launch test_teleop.launch.py


## External data

- camera_publisher.py
  - Publishes: /camera/image_raw (sensor_msgs/Image).
  - Responsibility: stream video (to dashboard).
- imu_publisher.py (optional)
  - Publishes: /imu/data (sensor_msgs/Imu).
- lidar_publisher.py (optional)
  - Publishes: /scan (sensor_msgs/LaserScan).
- odometry_publisher.py
  - Publishes: /odom (nav_msgs/Odometry).
  - Derived from wheel encoders or simulation.

## Nodes

### status_publisher.py
Info: CPU, battery, network latency, uptime

- Publishes:
    - /robot/status
    - /odom

### cmd_vel_subscriber.py
Converts generic teleop input into actuator commands.

- Subscribes:
  - /cmd_vel (geometry_msgs/Twist) → velocity commands (from web joystick).
- Publishes:
    - /wheel_commands (custom WheelCmd.msg or directly to motor driver topic).

### emergency_stop.py
Fail-safe.

- Service:
  - /emergency_stop (std_srvs/Trigger): On trigger: zeros velocity and sends stop command.

### watchdog.py
Monitors: last received /cmd_vel.

If timeout (e.g. >1 s): sends stop command.
Responsibility: safety in case of connection loss.


