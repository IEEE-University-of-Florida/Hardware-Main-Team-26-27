# Raspberry Pi Pico firmware

Implementation placeholders are in `src/` with paired headers in `include/`. The entry point and pinout are single files. See the [ESP32–Pico interface](../README.md#esp32pico-interface) for the board boundary.

| Module | Responsibility |
| --- | --- |
| `main.cpp` | Initialize the Pico with disabled outputs and run the motor control loop. |
| `pinout.hpp` | Hold the confirmed Pico GPIO assignments. |
| `hub_link` | Receive requests and return status over the selected transport. |
| `motor_protocol` | Define and encode the command and status messages used by both firmware projects. Keep this module independent of either board SDK. |
| `command_watchdog` | Track command freshness and gate motor outputs. |
| `motor_driver` | Translate logical commands into the signaling required by the motor controllers. |
| `drivetrain` | Apply requests for four wheel channels and handle any wheel feedback. |
| `downforce` | Apply requests and operating limits for the fifth motor channel. |

The Pico owns motor driver outputs. The proposed logical channel names are `front_left`, `front_right`, `rear_left`, `rear_right`, and `downforce`. Map those names to physical outputs once the drivetrain and wiring are confirmed. The fifth channel gets its own driver configuration and operating limits.

Keep driver mapping and output conversion separate from handling commands from the ESP32. Keep outputs disabled at boot, on stale commands, and when a stop or driver fault is reported. Confirm the required disarm and re-arm behavior with the team when implementing the link.

Record the exact Pico variant, motor controller input types, pin assignments, limits, and any available motor feedback here as those are finalized.
