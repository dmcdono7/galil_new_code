# Getting Started

The Galil ROS 2 Test Repo for RSP currently supports three useful command paths:
* `position_controller` - position command test controller
* `real_velocity_controller` - real_velocity command test controller
* `sine_real_velocity_controller` - a controller giving sine-wave real_velocity command
* `mp_needle_trajectory_controller` - controls the robotic insertion using Model Predictive Control based on estimated values from an FBG sensor

All units are in meters. When using real hardware, the recommended largest velocity is `0.0025 m/s`.

## Prerequisites

Galil gclib install instructions:
* https://www.galil.com/sw/pub/all/doc/global/install/linux/ubuntu/

Galil Python wrapper docs:
* https://www.galil.com/sw/pub/all/doc/gclib/html/python.html

Nlopt installation instructions:
This gets tricky because the CPP header (nlopt.hpp) is not installed with the rest of the package. The extra installation steps are described below.
* First, install the package library:
```bash
sudo apt install libnlopt-dev
```
* Second, download the latest version of the NLopt library from here (it is under 'Download and Installation' library):
  https://nlopt.readthedocs.io/en/latest/
* Then, follow the instructions here (just the first two code blocks in the first two sections) to build it with cmake:
  https://nlopt.readthedocs.io/en/latest/NLopt_Installation/
After that you should be all set!
## Build

```bash
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
