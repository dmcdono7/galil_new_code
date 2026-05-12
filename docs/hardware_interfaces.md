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

