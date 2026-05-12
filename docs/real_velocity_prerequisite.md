# Real Velocity Prerequisite

Before using any controller that claims `joint/real_velocity`, fill the per-joint Galil init parameters in:

- `galil_description/urdf/galil.ros2_control.xacro`

Required parameters per joint:

- `real_velocity_kp`: Kp param
- `real_velocity_ki`: Ki param
- `real_velocity_kd`: Kd param
- `real_velocity_it`: filter (1=no filter, smaller = smoother but less sensitive)
- `real_velocity_ac`: max acceleration
- `real_velocity_dc`: max deceleration
- `torque_constant`: found in motor datasheets
- `gear_ratio`: joint gear ratio

Additional required information:
- `ip_address`: Robot IP address

