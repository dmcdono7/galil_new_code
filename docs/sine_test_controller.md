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

Explanation:

- `joint_index`: which single joint gets the sine command
- `amplitude`: peak velocity
- `frequency`: sine frequency in Hz
- `offset`: constant bias
- `phase`: initial phase (in radians)
- `duration`: seconds before the controller outputs zero and holds; `0.0` means run forever

### Use PlotJuggler

A very useful tool for visualizing the data:

```bash
ros2 run plotjuggler plotjuggler
```

- measured motion/velocity is available on `/joint_states`
