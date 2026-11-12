# vesc

## Purpose

ROS 2 drivers and message packages for VESC motor controllers. The nested
repository contains the low-level serial driver, VESC messages, and the
Ackermann conversion and odometry nodes.

## Use

The F1TENTH stack launches the relevant nodes through f1tenth_stack bringup:

~~~bash
ros2 launch f1tenth_stack bringup_launch.py
~~~

For direct driver testing:

~~~bash
ros2 launch vesc_driver vesc_driver_node.launch.py
~~~

The VESC must be connected and the configured serial device must be accessible.

## Configuration

The main F1TENTH configuration is:

~~~text
f1tenth_stack/config/vesc.yaml
~~~

Important settings include:

- port
- speed_to_erpm_gain and speed_to_erpm_offset
- steering_angle_to_servo_gain and steering_angle_to_servo_offset
- wheelbase
- servo and speed limits
- publish_tf
- publish_imu

The current odometry uses a kinematic bicycle model: speed comes from VESC
telemetry and yaw rate is calculated from steering angle and wheelbase.

## Interfaces

The driver publishes VESC telemetry, sensor data, and IMU messages. The
Ackermann node consumes vehicle commands, and vesc_to_odom publishes /odom and
optionally odom to base_link TF.

## Theory and further reading

The driver converts Ackermann commands into VESC motor/servo commands and
estimates planar motion from speed and steering. See the
[VESC firmware project](https://vesc-project.com/) and the
[F1TENTH VESC documentation](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html).
