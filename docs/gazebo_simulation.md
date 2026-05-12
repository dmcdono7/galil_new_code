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
