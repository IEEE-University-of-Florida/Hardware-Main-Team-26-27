#pragma once

#include "otos.h"

struct OtosCalibration {
    float linear_scalar = 1.0f;
    float angular_scalar = 1.0f;
    otos::Pose offset = {};  // Meters and radians, relative to the robot origin.
};

struct OtosSample {
    uint64_t monotonic_time_us;
    otos::Pose position;
    otos::Pose velocity;
    otos::Pose acceleration;
    otos::Status status;

    bool valid() const { return !status.anyProblem(); }
};

class OtosLocalization {
public:
    // The robot must be stationary during initialization and calibration.
    otos::Err begin(otos::Bus& bus, const OtosCalibration& calibration);

    // On error the output is unchanged. A successful read can still report a
    // sensor warning via sample.valid(). The caller supplies a monotonic timestamp.
    otos::Err read(uint64_t monotonic_time_us, OtosSample& sample);

private:
    otos::Otos sensor_;
    bool ready_ = false;
};
