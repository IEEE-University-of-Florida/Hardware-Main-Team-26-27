#include "otos_localization.hpp"

otos::Err OtosLocalization::begin(otos::Bus& bus, const OtosCalibration& calibration) {
    using otos::Err;
    ready_ = false;
    sensor_.setLinearUnit(otos::LinearUnit::Meters);
    sensor_.setAngularUnit(otos::AngularUnit::Radians);

    Err result = sensor_.begin(bus);
    if (result != Err::Ok) return result;
    result = sensor_.selfTest();
    if (result != Err::Ok) return result;
    result = sensor_.calibrateImu();
    if (result != Err::Ok) return result;
    result = sensor_.setLinearScalar(calibration.linear_scalar);
    if (result != Err::Ok) return result;
    result = sensor_.setAngularScalar(calibration.angular_scalar);
    if (result != Err::Ok) return result;
    result = sensor_.setOffset(calibration.offset);
    if (result != Err::Ok) return result;
    result = sensor_.resetTracking();
    ready_ = result == Err::Ok;
    return result;
}

otos::Err OtosLocalization::read(uint64_t monotonic_time_us, OtosSample& sample) {
    using otos::Err;
    if (!ready_) return Err::NotBegun;

    OtosSample reading{};
    reading.monotonic_time_us = monotonic_time_us;
    Err result = sensor_.getPosVelAcc(reading.position, reading.velocity, reading.acceleration);
    if (result != Err::Ok) return result;
    result = sensor_.getStatus(reading.status);
    if (result == Err::Ok) sample = reading;
    return result;
}
