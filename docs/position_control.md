## Position Control

Send a command in this format:

```bash
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
```
