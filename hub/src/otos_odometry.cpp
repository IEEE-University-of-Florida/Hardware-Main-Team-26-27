#include "otos_odometry.hpp"

#include <cmath>
#include <cstddef>

namespace {
constexpr double kUnmeasuredVariance = 1e6;

std::array<double, 36> planarCovariance(const std::array<double, 3>& variance) {
    std::array<double, 36> covariance{};
    covariance[0] = variance[0];
    covariance[7] = variance[1];
    covariance[14] = kUnmeasuredVariance;  // z
    covariance[21] = kUnmeasuredVariance;  // roll
    covariance[28] = kUnmeasuredVariance;  // pitch
    covariance[35] = variance[2];
    return covariance;
}
}

OtosOdometryConverter::OtosOdometryConverter(const OtosOdometryConfig& config)
    : config_(config), twist_covariance_(planarCovariance(config.twist_variance)) {}

bool OtosOdometryConverter::update(const OtosSample& sample, OtosOdometry& output) {
    if (!sample.valid()) {
        markGap();
        return false;
    }

    if (has_previous_position_) {
        const double dx = static_cast<double>(sample.position.x) - previous_position_.x;
        const double dy = static_cast<double>(sample.position.y) - previous_position_.y;
        distance_m_ += std::hypot(dx, dy);
    }
    previous_position_ = sample.position;
    has_previous_position_ = true;

    auto variance = config_.pose_variance;
    for (std::size_t axis = 0; axis < variance.size(); ++axis) {
        const double drift = config_.pose_drift_per_meter[axis] * distance_m_;
        variance[axis] += drift * drift;
    }

    const double yaw = sample.position.h;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    output.monotonic_time_us = sample.monotonic_time_us;
    output.odom_pose = sample.position;
    output.orientation_z = std::sin(yaw / 2.0);
    output.orientation_w = std::cos(yaw / 2.0);
    // Express the tracking-frame velocity in the robot's axes with R(-yaw).
    output.body_velocity_x = cosine * sample.velocity.x + sine * sample.velocity.y;
    output.body_velocity_y = -sine * sample.velocity.x + cosine * sample.velocity.y;
    output.yaw_rate = sample.velocity.h;
    output.pose_covariance = planarCovariance(variance);
    output.twist_covariance = twist_covariance_;
    return true;
}

void OtosOdometryConverter::markGap() {
    has_previous_position_ = false;
}

void OtosOdometryConverter::reset() {
    distance_m_ = 0.0;
    markGap();
}
