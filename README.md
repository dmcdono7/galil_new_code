# Galil ROS 2 Test Repo for RSP

Currently supports three useful command paths:

- `position_controller` position command test controller
- `real_velocity_controller` real_velocity command test controller
- `sine_real_velocity_controller` a controller giving sine-wave real_velocity command

All units are in meters. When using real hardware, the recommanded largest velocity is 0.0025 m/s.

## Prerequisites

Galil gclib install instructions:

- https://www.galil.com/sw/pub/all/doc/global/install/linux/ubuntu/

Galil Python wrapper docs:

- https://www.galil.com/sw/pub/all/doc/gclib/html/python.html

## Build

```bash
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Real Velocity Prerequisite

Before using any controller that claims `joint/real_velocity`, fill the per-joint Galil init parameters in:

- [galil_description/urdf/galil.ros2_control.xacro]

Required params per joint:

- `real_velocity_kp`: Kp param
- `real_velocity_ki`: Ki param
- `real_velocity_kd`: Kd param
- `real_velocity_it`: filter (1=no filter, smaller = smoother but less sensitive)
- `real_velocity_ac`: max acceleration
- `real_velocity_dc`: max decceleration

## Launch Files

### `mariana_view.launch.py`
From Simon
Starts:

- `joint_state_broadcaster` active
- `position_controller` active
- `velocity_controller` inactive
- `real_velocity_controller` inactive
- `sine_real_velocity_controller` inactive

Run it with:

```bash
ros2 launch galil_driver mariana_view.launch.py
```

### `RSP.launch.py`

Starts:

- `joint_state_broadcaster` active
- `real_velocity_controller` active
- `position_controller` inactive
- `velocity_controller` inactive
- `sine_real_velocity_controller` inactive

Run it with:

```bash
ros2 launch galil_driver RSP.launch.py
```

## Gazebo Simulation

Run the same launch file in sim mode:

```bash
ros2 launch galil_driver RSP.launch.py use_simulation:=true
```

In simulation, Gazebo owns `/controller_manager` through the `ign_ros2_control` plugin. 

The routing is different from hardware due to Gazebo cannot recognize our custom real_velocity interface:

- [galil_description/urdf/galil.ros2_control.xacro](galil_description/urdf/galil.ros2_control.xacro) hides the custom `real_velocity` command interface in sim, because Gazebo only understands standard simulated command interfaces such as `position`, `velocity`, and `effort`
- the same xacro loads [galil_driver/config/galil_controllers_sim.yaml](galil_driver/config/galil_controllers_sim.yaml) for the Gazebo controller manager
- `galil_controllers_sim.yaml` keeps the controller name `real_velocity_controller`, but changes its `interface_name` to `velocity`
- the purpose of this re-routing is to allow existing code to publish to `/real_velocity_controller/commands` in sim without changing the hardware controller YAML

Example of send a sim velocity command (different than real hardware because in real, once a command is sent to Galil, galil will keep that command executed. But in sim, it needs to be sent repeatedly to keep executing):

```bash
ros2 topic pub -r 20 /real_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.001, 0.0, 0.0, 0.0]}"
```

Stop motion:

```bash
ros2 topic pub -1 /real_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
```

NOTE: In sim, command values are normal Gazebo joint velocities, not Galil jog counts. For the prismatic joints, start with small values near the URDF limits.

Switch from `real_velocity_controller` to the sine controller:

```bash
ros2 control switch_controllers --controller-manager /controller_manager --deactivate real_velocity_controller --activate sine_real_velocity_controller --strict
```
If you want to try the esitimated velocity controller from Simon:
`velocity_controller` is defined in the sim YAML, but `RSP.launch.py` does not spawn it in sim by default. Load it inactive before switching to it:

```bash
ros2 run controller_manager spawner velocity_controller --controller-manager /controller_manager --inactive
ros2 control switch_controllers --controller-manager /controller_manager --deactivate real_velocity_controller --activate velocity_controller --strict
```

## Inspection Commands:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

## Joint Order

Controller command arrays in this order:

- index `0`: `insertion_joint`
- index `1`: `horizontal_joint`
- index `2`: `vertical_joint`
- index `3`: `needle_drive_joint`

## Hardware Interfaces

Each joint currently exports these **state interfaces**:

- `position`: read from Galil `TP`
- `velocity`: read from Galil `TV`
- `effort`: read from Galil `TT`. 

Each joint currently exports these **command interfaces**:

- `position`: standard position command path using Galil `PA` and `BG`
- `velocity`: Simon's workaround path using Position Tracking (`PT`) plus incremental `PA`
- `real_velocity`: true jog-style velocity path using `JG`, `BG`, and axis-specific `ST`

Controller mapping:

- `position_controller` -> `position`
- `velocity_controller` -> `velocity`
- `real_velocity_controller` -> `real_velocity`
- `sine_real_velocity_controller` -> `real_velocity`

## Position Control


```bash
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
```

## Simon's Estimated Velocity Control

Activate it:

```bash
ros2 control switch_controllers --deactivate position_controller --activate velocity_controller
```

Send one command:

```bash
ros2 topic pub -1 /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
```

## Real Velocity Controller

`real_velocity_controller` writes to the `real_velocity` ros2_control interface, which the hardware interface maps to the Galil jog sequence.

If you launched `RSP.launch.py`, it is already active by default.

If need to switch:

```bash
ros2 control switch_controllers --deactivate position_controller --activate real_velocity_controller
```

Send one command:

```bash
ros2 topic pub -1 /real_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
```

## Sine Test Controller

`sine_real_velocity_controller` is a separate ros2_control controller plugin in `galil_test_controllers`. It's a placeholder that I used to test continuously changing velocity.

It:

- claims the same `joint/real_velocity` interfaces as `real_velocity_controller`
- generates its sine wave inside the controller-manager `update()` loop
- uses the same Galil jog backend through the hardware interface

Because it claims the same interfaces, it should not be active at the same time as `real_velocity_controller`.

### Activate the sine controller

From `RSP.launch.py`:

```bash
ros2 control switch_controllers --deactivate real_velocity_controller --activate sine_real_velocity_controller
```

### Configure the sine profile

Edit:

- [galil_driver/config/galil_controllers.yaml]

Meaning:

- `joint_index`: which single joint gets the sine command
- `amplitude`: peak velocity
- `frequency`: sine frequency in Hz
- `offset`: constant bias
- `phase`: initial phase (in radians)
- `duration`: seconds before the controller outputs zero and holds; `0.0` means run forever

### Use PlotJuggler

```bash
ros2 run plotjuggler plotjuggler
```

- measured motion/velocity is available on `/joint_states`
