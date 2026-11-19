#pragma once

#include <array>

#include "otos_localization.hpp"

struct OtosOdometryConfig {
    // Planar entries are x, y, and yaw. These defaults are for bench use.
    std::array<double, 3> pose_variance = {{1e-3, 1e-3, 1e-3}};
    std::array<double, 3> pose_drift_per_meter = {{5e-3, 5e-3, 5e-3}};
    std::array<double, 3> twist_variance = {{1e-3, 1e-3, 1e-3}};
};

struct OtosOdometry {
    uint64_t monotonic_time_us;
    otos::Pose odom_pose;
    double orientation_z;
    double orientation_w;
    double body_velocity_x;
    double body_velocity_y;
    double yaw_rate;
    std::array<double, 36> pose_covariance;
    std::array<double, 36> twist_covariance;
};

class OtosOdometryConverter {
public:
    explicit OtosOdometryConverter(const OtosOdometryConfig& config = OtosOdometryConfig{});

    // Pose is in odom and twist is in base_link. Invalid samples leave output unchanged.
    bool update(const OtosSample& sample, OtosOdometry& output);
    // A failed read breaks the distance segment without wiping accumulated drift.
    void markGap();
    // Reset the distance model when the sensor is deliberately re-zeroed.
    void reset();

private:
    OtosOdometryConfig config_;
    std::array<double, 36> twist_covariance_;
    double distance_m_ = 0.0;
    otos::Pose previous_position_{};
    bool has_previous_position_ = false;
};
