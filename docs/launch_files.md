## Launch Files

### `RSP.launch.py`

Starts:

- `joint_state_broadcaster` active
- `real_velocity_controller` active
- `position_controller` inactive
- `velocity_controller` inactive
- `sine_real_velocity_controller` inactive
- `mpc_controller` inactive

Run it with:

```bash
ros2 launch galil_driver RSP.launch.py
```

### `mpc.launch.py`

Starts:

- `joint_state_broadcaster` active
- `mpc_controller` active
- `position_controller` inactive
- `velocity_controller` inactive
- `sine_real_velocity_controller` inactive
- `real_velocity_controller` inactive

Run it with:

```bash
ros2 launch galil_driver mpc.launch.py
```
