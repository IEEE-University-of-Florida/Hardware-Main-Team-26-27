// otos.h - driver for the SparkFun Qwiic Optical Tracking Odometry Sensor
// (PAA5160E1 + LSM6DSO, SparkFun SEN-24904), I2C address 0x17.
//
// The sensor's own firmware fuses the optical sensor and IMU and reports
// a pose (x, y, heading), velocity and acceleration. This driver reads and
// writes that register map. Register addresses and scaling constants follow
// SparkFun's open-source SparkFun_Qwiic_OTOS_Arduino_Library (MIT license).
//
// Portable C++11: no heap, no exceptions, no RTTI. I2C is behind otos::Bus,
// so the same code runs on ESP-IDF, STM32 HAL, or a PC simulator.
#ifndef OTOS_H
#define OTOS_H

#include <stddef.h>
#include <stdint.h>

namespace otos {

constexpr uint8_t kI2cAddress = 0x17;  // 7-bit address
constexpr uint8_t kProductId = 0x5F;

enum class Err : int8_t {
    Ok = 0,
    Bus = -1,           // I2C read/write failed (NACK, timeout, wiring)
    NotFound = -2,      // nothing answered at 0x17
    BadProductId = -3,  // something answered at 0x17 but it is not an OTOS
    SelfTestFailed = -4,
    Timeout = -5,       // self-test or IMU calibration never finished
    OutOfRange = -6,    // argument cannot be represented by the sensor
    NotBegun = -7       // begin() was not called or failed
};

const char* errName(Err e);

enum class LinearUnit : uint8_t { Meters, Inches };
enum class AngularUnit : uint8_t { Degrees, Radians };

struct Pose {
    float x;  // linear unit
    float y;  // linear unit
    float h;  // heading, angular unit. Counter-clockwise positive.
};

struct Status {
    uint8_t raw;
    bool warnTiltAngle;       // tilted too far: accelerometer ignored
    bool warnOpticalTracking; // optical tracking unreliable (height/surface)
    bool errorPaa;            // optical sensor fatal error
    bool errorLsm;            // IMU fatal error
    bool anyProblem() const { return raw != 0; }
};

// Minimal I2C interface. Register reads/writes auto-increment the address.
class Bus {
  public:
    virtual ~Bus() {}
    virtual bool ping() = 0;  // does a device ACK at 0x17?
    virtual bool read(uint8_t reg, uint8_t* buf, size_t len) = 0;
    virtual bool write(uint8_t reg, const uint8_t* buf, size_t len) = 0;
    virtual void delayMs(uint32_t ms) = 0;
};

class Otos {
  public:
    Otos();

    // Connects and checks the product ID. Does not change sensor settings.
    Err begin(Bus& bus);
    Err isConnected();
    Err getVersion(uint8_t& hwVersion, uint8_t& fwVersion);  // raw: major<<4|minor
    Err selfTest();

    // Removes IMU offsets. Sensor must sit still and flat. Calibration is lost
    // on power cycle, so call this at every startup. 255 samples ~ 0.6 s.
    Err calibrateImu(uint8_t samples = 255, bool waitUntilDone = true);
    Err imuCalibrationRemaining(uint8_t& samplesLeft);

    // Defaults: meters and degrees. Apply to every pose-style call below.
    void setLinearUnit(LinearUnit u);
    void setAngularUnit(AngularUnit u);
    LinearUnit linearUnit() const { return linUnit_; }
    AngularUnit angularUnit() const { return angUnit_; }

    // Scale-error correction, 0.872 .. 1.127 in steps of 0.001. Not stored by
    // the sensor: re-apply at every startup.
    Err setLinearScalar(float s);
    Err getLinearScalar(float& s);
    Err setAngularScalar(float s);
    Err getAngularScalar(float& s);

    // Zeroes the pose. Also recovers from rare tracking glitches.
    Err resetTracking();
    Err getStatus(Status& st);

    // Where the sensor sits relative to the robot's centre, so that the
    // reported pose is the robot's pose rather than the sensor's.
    Err setOffset(const Pose& p);
    Err getOffset(Pose& p);

    Err getPosition(Pose& p);
    Err setPosition(const Pose& p);  // |x|,|y| must be <= 10 m (unit-adjusted)
    Err getVelocity(Pose& v);        // units per second
    Err getAcceleration(Pose& a);    // units per second^2
    // One 18-byte burst read: a consistent snapshot (preferred in a loop).
    Err getPosVelAcc(Pose& pos, Pose& vel, Pose& acc);

    // Calibration helpers (run with both scalars at 1.0).
    // Linear: scalar = truth / measured, e.g. pushed 1000 mm, sensor says 1030.
    static float scalarFromMeasurement(float truth, float measured);
    // Angular: spin the sensor `spins` full turns (positive = counter-
    // clockwise, negative = clockwise), then pass the final reported heading
    // in degrees. Returns the scalar to set.
    static float angularScalarFromSpins(int spins, float reportedHeadingDeg);

  private:
    Err readRegs(uint8_t reg, uint8_t* buf, size_t len);
    Err writeRegs(uint8_t reg, const uint8_t* buf, size_t len);
    Err readPose(uint8_t reg, Pose& p, float xyToSi, float hToSi);
    Err writePose(uint8_t reg, const Pose& p);
    void decode(const uint8_t* b, Pose& p, float xyToSi, float hToSi) const;
    Err setScalar(uint8_t reg, float s);
    Err getScalar(uint8_t reg, float& s);

    Bus* bus_;
    LinearUnit linUnit_;
    AngularUnit angUnit_;
    float meterToUnit_;
    float radToUnit_;
};

}  // namespace otos

#endif  // OTOS_H
