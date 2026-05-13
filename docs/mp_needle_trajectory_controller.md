## MP Needle Trajectory Controller

`mp_needle_trajectory_controller` writes to the `position` ros2_control interface, which the hardware interface maps to the Galil PA (position absolute) command.

To launch:
* Use `mpc.launch.py` within the `galil_driver` package. This will automatically activate the model predictive control.
* For example, to use in simulation with fake hardware:
```bash
ros2 launch galil_driver mpc.launch.py use_fake_hardware:=true use_simulation:=true
```

Send a target needle tip position. For example:
```bash
ros2 topic pub /subject/state/target geometry_msgs/msg/PointStamped "point: {x: 50.0, y: 0.0, z: 0.0}}" --once
```

If using simulation and no real FBG sensor, publish fake needle tip positions:
```bash
ros2 launch needle_tip_sim needle_tip_pub.launch.py
```

If using a real FBG interrogator, compute needle tip position from FBG wavelengths using desired reconstruction algorithm. Publish the needle tip values as a `geometry_msgs::msg::PoseStamped` message to the `sensor/tip` topic. 

Once a target measurement is received and the tip position is being updated, the `mpc_controller` will drive the robot to the provided target position. 
