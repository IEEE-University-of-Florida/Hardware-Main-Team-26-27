# SparkFun OTOS driver

The portable driver and simulator came from the team's `otos_driver.zip`. The hub uses this core through `OtosEspIdfBus` and `OtosLocalization`. The original STM32 HAL adapter and CubeIDE example remain in the supplied archive as references. Register definitions and scaling are based on [SparkFun's OTOS library](https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library). Its notice is included in [LICENSE.sparkfun.md](LICENSE.sparkfun.md).

## Simulator tests

Run from `hub/otos_driver/` with CMake 3.20 or newer and a desktop C++ compiler.

```sh
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

The simulator checks decoding, calibration, error handling, the hub wrapper, and odometry conversion. Its odometry tests cover the 90° velocity rotation, covariance growth, rejected samples, and reset behavior. Physical I²C transactions, sensor orientation, and tracking on the field need a board and sensor.

## Odometry reference

The body-frame velocity conversion and distance-based covariance model follow [Moritz Boeker's ROS 2 OTOS driver](https://github.com/moritzboeker/ros2_sparkfun_otos_driver). The [upstream MIT notice](LICENSE.ros2_sparkfun_otos_driver) is included here.

The converter adds the squared product of the configured drift coefficient and observed distance to the initial pose variance. Tune the bench defaults with repeated runs on the actual track. After an invalid sample or read failure it starts a new distance segment at the next valid pose, preserving accumulated variance. Call `reset()` when deliberately re-zeroing the sensor.

## Bench bring-up

- Connect the OTOS to the ESP32 with 3.3 V, ground, SDA, and SCL. Use the actual board pinout and confirm the bus has pull-ups. The sensor uses address `0x17`. Start at 100 kHz.
- Configure the two GPIO numbers in ESP-IDF `menuconfig` under **Hub hardware**. Keep the robot still and level while the driver checks the device, runs its self-test, and calibrates the IMU.
- The current bench app uses unity scalars and a zero mounting offset. Measure the actual offset from the robot origin, then measure a straight push and several full turns to determine the linear and angular scalars before using the readings for navigation.
- Check the pose when stationary, pushed a measured distance, and rotated a known angle on the track surface. Confirm the axes and yaw direction, then confirm a disconnected sensor reports a read failure and degraded tracking reports a status warning.
