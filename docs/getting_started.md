# Getting Started

The Galil ROS 2 Test Repo for RSP currently supports three useful command paths:
* `position_controller` - position command test controller
* `real_velocity_controller` - real_velocity command test controller
* `sine_real_velocity_controller` - a controller giving sine-wave real_velocity command

All units are in meters. When using real hardware, the recommended largest velocity is `0.0025 m/s`.

## Prerequisites

Galil gclib install instructions:
* https://www.galil.com/sw/pub/all/doc/global/install/linux/ubuntu/

Galil Python wrapper docs:
* https://www.galil.com/sw/pub/all/doc/gclib/html/python.html

Nlopt? :
* 

## Build

```bash
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
