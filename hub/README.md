# Hub firmware

The ESP32-S3 Sense is this year's hub. Application code is in `src/` and `include/`. The OTOS driver and its simulator live in [`otos_driver/`](otos_driver/README.md). The [ESP32–Pico interface](../README.md#esp32pico-interface) and race modules are scaffolding.

| Module | Responsibility |
| --- | --- |
| `main.cpp` | Initialize the OTOS and log readings for bench bring-up. |
| `board_config.hpp` | Read the configured OTOS GPIO assignments. |
| `camera` | Configure the camera and capture frames. |
| `vision` | Detect the green flag, pit assignment AprilTag, and track or obstacle cues. |
| `otos_localization` | Return a timestamped planar pose (`x`, `y`, heading) and tracking status from the SparkFun OTOS. |
| `otos_esp_idf_bus` | Connect the portable OTOS driver to the ESP-IDF I²C master. |
| `otos_odometry` | Convert OTOS samples to ROS odometry conventions and estimate covariance from observed travel. |
| `track_progress` | Use pose and available course observations to track lap crossings and approach to pit and finish areas. |
| `race_state_machine` | Wait for start, handle pit entry and exit, and stop at the finish or match timeout. |
| `motion_planner` | Turn race intent and sensor observations into four drivetrain requests and a separate downforce request. |
| `pico_link` | Send motor requests and receive controller status using the shared protocol. |

## ESP32 firmware build

Use a Linux development environment with ESP-IDF 5.4 or newer activated, GNU Make, and the micro-ROS build dependencies. From `hub/`, run the following commands. Set the actual SDA and SCL GPIO numbers under **Hub hardware** in `menuconfig` before flashing.

```sh
python -m pip install catkin_pkg colcon-common-extensions lark
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p <serial-port> flash monitor
```

The bench app retries OTOS startup every two seconds, so keep the robot stationary until pose readings appear. It logs `x` and `y` in meters, yaw in radians, and velocity in the robot's body frame. After startup, read failures and sensor warnings are reported without triggering recalibration. See the [OTOS bring-up and simulator instructions](otos_driver/README.md). [ESP-IDF build guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/start-project.html) · [Jazzy micro-ROS component](https://github.com/micro-ROS/micro_ros_espidf_component/tree/jazzy)

## ROS 2 on the ESP32

Last year's `hub/` was a native ROS 2 Jazzy workspace on a Linux Raspberry Pi. This year's ESP32-S3 runs ESP-IDF and is the target for the onboard micro-ROS client. `main/idf_component.yml` selects the Jazzy component. `idf.py` builds the board firmware, and `colcon` is used by the component to cross-build its ROS libraries.

The OTOS converter prepares pose and yaw orientation in `odom`, twist in `base_link`, and pose and twist covariance for `nav_msgs/Odometry`. The runtime still needs the node, publisher, ROS-clock timestamp, transport configuration, and a Jazzy micro-ROS Agent on a ROS 2 host. Use the agent for development and inspection with the current two-board design. Keep onboard race decisions and the ESP32–Pico motor link independent of the development host so match operation stays autonomous. [Jazzy micro-ROS component and transport examples](https://github.com/micro-ROS/micro_ros_espidf_component/tree/jazzy)
