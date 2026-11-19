#include "otos.h"

#include <math.h>

namespace otos {

namespace {

// Register map (SparkFun_Qwiic_OTOS_Arduino_Library, sfDevOTOS.h)
constexpr uint8_t kRegProductId = 0x00;
constexpr uint8_t kRegHwVersion = 0x01;  // 0x02 = firmware version
constexpr uint8_t kRegScalarLinear = 0x04;
constexpr uint8_t kRegScalarAngular = 0x05;
constexpr uint8_t kRegImuCalib = 0x06;
constexpr uint8_t kRegReset = 0x07;
constexpr uint8_t kRegSelfTest = 0x0F;
constexpr uint8_t kRegOffset = 0x10;  // x,y,h int16 little-endian
constexpr uint8_t kRegStatus = 0x1F;
constexpr uint8_t kRegPosition = 0x20;  // then velocity 0x26, accel 0x2C

// Self-test register bits
constexpr uint8_t kSelfTestStart = 1u << 0;
constexpr uint8_t kSelfTestInProgress = 1u << 1;
constexpr uint8_t kSelfTestPass = 1u << 2;

constexpr float kPi = 3.14159265358979323846f;

// int16 full scale -> physical units (all signed, +-32768 = +-full scale)
constexpr float kRawToMeter = 10.0f / 32768.0f;                     // +-10 m
constexpr float kRawToRad = kPi / 32768.0f;                         // +-pi
constexpr float kRawToMps = 5.0f / 32768.0f;                        // +-5 m/s
constexpr float kRawToRps = (2000.0f * kPi / 180.0f) / 32768.0f;    // +-2000 dps
constexpr float kRawToMpss = (16.0f * 9.80665f) / 32768.0f;         // +-16 g
constexpr float kRawToRpss = (kPi * 1000.0f) / 32768.0f;            // +-pi*1000

constexpr float kMeterToInch = 39.37f;
constexpr float kRadToDegree = 180.0f / kPi;

inline int16_t le16(const uint8_t* b) {
    return static_cast<int16_t>(static_cast<uint16_t>(b[0]) |
                                (static_cast<uint16_t>(b[1]) << 8));
}

// Round-to-nearest without relying on lroundf availability in minimal libcs.
inline int32_t roundToInt(float v) {
    return static_cast<int32_t>(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

}  // namespace

const char* errName(Err e) {
    switch (e) {
        case Err::Ok: return "ok";
        case Err::Bus: return "i2c bus error";
        case Err::NotFound: return "no device at 0x17";
        case Err::BadProductId: return "wrong product id";
        case Err::SelfTestFailed: return "self-test failed";
        case Err::Timeout: return "timeout";
        case Err::OutOfRange: return "value out of range";
        case Err::NotBegun: return "begin() not done";
    }
    return "unknown";
}

Otos::Otos()
    : bus_(nullptr),
      linUnit_(LinearUnit::Meters),
      angUnit_(AngularUnit::Degrees),
      meterToUnit_(1.0f),
      radToUnit_(kRadToDegree) {}

Err Otos::readRegs(uint8_t reg, uint8_t* buf, size_t len) {
    if (bus_ == nullptr) return Err::NotBegun;
    return bus_->read(reg, buf, len) ? Err::Ok : Err::Bus;
}

Err Otos::writeRegs(uint8_t reg, const uint8_t* buf, size_t len) {
    if (bus_ == nullptr) return Err::NotBegun;
    return bus_->write(reg, buf, len) ? Err::Ok : Err::Bus;
}

Err Otos::begin(Bus& bus) {
    bus_ = &bus;
    Err e = isConnected();
    if (e != Err::Ok) bus_ = nullptr;
    return e;
}

Err Otos::isConnected() {
    if (bus_ == nullptr) return Err::NotBegun;
    if (!bus_->ping()) return Err::NotFound;
    uint8_t id = 0;
    Err e = readRegs(kRegProductId, &id, 1);
    if (e != Err::Ok) return e;
    return id == kProductId ? Err::Ok : Err::BadProductId;
}

Err Otos::getVersion(uint8_t& hwVersion, uint8_t& fwVersion) {
    uint8_t v[2];
    Err e = readRegs(kRegHwVersion, v, 2);
    if (e != Err::Ok) return e;
    hwVersion = v[0];
    fwVersion = v[1];
    return Err::Ok;
}

Err Otos::selfTest() {
    if (bus_ == nullptr) return Err::NotBegun;
    uint8_t start = kSelfTestStart;
    Err e = writeRegs(kRegSelfTest, &start, 1);
    if (e != Err::Ok) return e;

    // Takes ~20 ms on firmware v1.0; allow up to 100 ms.
    for (int i = 0; i < 20; i++) {
        bus_->delayMs(5);
        uint8_t r = 0;
        e = readRegs(kRegSelfTest, &r, 1);
        if (e != Err::Ok) return e;
        if ((r & kSelfTestInProgress) == 0) {
            return (r & kSelfTestPass) ? Err::Ok : Err::SelfTestFailed;
        }
    }
    return Err::Timeout;
}

Err Otos::calibrateImu(uint8_t samples, bool waitUntilDone) {
    if (bus_ == nullptr) return Err::NotBegun;
    if (samples == 0) return Err::OutOfRange;
    Err e = writeRegs(kRegImuCalib, &samples, 1);
    if (e != Err::Ok) return e;
    bus_->delayMs(3);  // one sample period is 2.4 ms
    if (!waitUntilDone) return Err::Ok;

    // Register counts down to 0 as samples are taken (2.4 ms each).
    const uint16_t maxReads = static_cast<uint16_t>(samples) + 20u;
    for (uint16_t i = 0; i < maxReads; i++) {
        uint8_t left = 0;
        e = readRegs(kRegImuCalib, &left, 1);
        if (e != Err::Ok) return e;
        if (left == 0) return Err::Ok;
        bus_->delayMs(3);
    }
    return Err::Timeout;
}

Err Otos::imuCalibrationRemaining(uint8_t& samplesLeft) {
    return readRegs(kRegImuCalib, &samplesLeft, 1);
}

void Otos::setLinearUnit(LinearUnit u) {
    linUnit_ = u;
    meterToUnit_ = (u == LinearUnit::Meters) ? 1.0f : kMeterToInch;
}

void Otos::setAngularUnit(AngularUnit u) {
    angUnit_ = u;
    radToUnit_ = (u == AngularUnit::Radians) ? 1.0f : kRadToDegree;
}

Err Otos::setScalar(uint8_t reg, float s) {
    // Stored as a signed byte, multiples of 0.1% around 1.0.
    const float steps = (s - 1.0f) * 1000.0f;
    if (!(steps > -128.5f && steps < 127.5f)) return Err::OutOfRange;
    const int32_t raw = roundToInt(steps);
    const uint8_t b = static_cast<uint8_t>(static_cast<int8_t>(raw));
    return writeRegs(reg, &b, 1);
}

Err Otos::getScalar(uint8_t reg, float& s) {
    uint8_t b = 0;
    Err e = readRegs(reg, &b, 1);
    if (e != Err::Ok) return e;
    s = 1.0f + static_cast<int8_t>(b) * 0.001f;
    return Err::Ok;
}

Err Otos::setLinearScalar(float s) { return setScalar(kRegScalarLinear, s); }
Err Otos::getLinearScalar(float& s) { return getScalar(kRegScalarLinear, s); }
Err Otos::setAngularScalar(float s) { return setScalar(kRegScalarAngular, s); }
Err Otos::getAngularScalar(float& s) { return getScalar(kRegScalarAngular, s); }

Err Otos::resetTracking() {
    const uint8_t one = 0x01;
    return writeRegs(kRegReset, &one, 1);
}

Err Otos::getStatus(Status& st) {
    uint8_t r = 0;
    Err e = readRegs(kRegStatus, &r, 1);
    if (e != Err::Ok) return e;
    st.raw = r;
    st.warnTiltAngle = (r & (1u << 0)) != 0;
    st.warnOpticalTracking = (r & (1u << 1)) != 0;
    st.errorPaa = (r & (1u << 6)) != 0;
    st.errorLsm = (r & (1u << 7)) != 0;
    return Err::Ok;
}

void Otos::decode(const uint8_t* b, Pose& p, float xyToSi, float hToSi) const {
    p.x = le16(b) * xyToSi * meterToUnit_;
    p.y = le16(b + 2) * xyToSi * meterToUnit_;
    p.h = le16(b + 4) * hToSi * radToUnit_;
}

Err Otos::readPose(uint8_t reg, Pose& p, float xyToSi, float hToSi) {
    uint8_t b[6];
    Err e = readRegs(reg, b, sizeof(b));
    if (e != Err::Ok) return e;
    decode(b, p, xyToSi, hToSi);
    return Err::Ok;
}

Err Otos::writePose(uint8_t reg, const Pose& p) {
    // Position/offset registers: x,y in +-10 m, heading in +-pi.
    const float xm = p.x / meterToUnit_;
    const float ym = p.y / meterToUnit_;
    float hr = p.h / radToUnit_;
    if (!isfinite(hr)) return Err::OutOfRange;
    // Wrap heading into [-pi, pi) so 180 deg is representable.
    hr = fmodf(hr, 2.0f * kPi);
    if (hr >= kPi) hr -= 2.0f * kPi;
    if (hr < -kPi) hr += 2.0f * kPi;

    const float xr = xm / kRawToMeter;
    const float yr = ym / kRawToMeter;
    float hRaw = hr / kRawToRad;
    if (!(xr >= -32768.0f && xr <= 32767.0f)) return Err::OutOfRange;
    if (!(yr >= -32768.0f && yr <= 32767.0f)) return Err::OutOfRange;
    if (hRaw > 32767.0f) hRaw = 32767.0f;  // rounding at the +pi edge

    const int16_t raw[3] = {static_cast<int16_t>(roundToInt(xr)),
                            static_cast<int16_t>(roundToInt(yr)),
                            static_cast<int16_t>(roundToInt(hRaw))};
    uint8_t b[6];
    for (int i = 0; i < 3; i++) {
        const uint16_t u = static_cast<uint16_t>(raw[i]);
        b[2 * i] = static_cast<uint8_t>(u & 0xFF);
        b[2 * i + 1] = static_cast<uint8_t>(u >> 8);
    }
    return writeRegs(reg, b, sizeof(b));
}

Err Otos::setOffset(const Pose& p) { return writePose(kRegOffset, p); }
Err Otos::getOffset(Pose& p) { return readPose(kRegOffset, p, kRawToMeter, kRawToRad); }
Err Otos::setPosition(const Pose& p) { return writePose(kRegPosition, p); }
Err Otos::getPosition(Pose& p) { return readPose(kRegPosition, p, kRawToMeter, kRawToRad); }
Err Otos::getVelocity(Pose& v) { return readPose(kRegPosition + 6, v, kRawToMps, kRawToRps); }
Err Otos::getAcceleration(Pose& a) { return readPose(kRegPosition + 12, a, kRawToMpss, kRawToRpss); }

Err Otos::getPosVelAcc(Pose& pos, Pose& vel, Pose& acc) {
    uint8_t b[18];
    Err e = readRegs(kRegPosition, b, sizeof(b));
    if (e != Err::Ok) return e;
    decode(b, pos, kRawToMeter, kRawToRad);
    decode(b + 6, vel, kRawToMps, kRawToRps);
    decode(b + 12, acc, kRawToMpss, kRawToRpss);
    return Err::Ok;
}

float Otos::scalarFromMeasurement(float truth, float measured) {
    return (measured != 0.0f) ? truth / measured : 1.0f;
}

float Otos::angularScalarFromSpins(int spins, float reportedHeadingDeg) {
    // The sensor wraps heading to +-180, so only the leftover error shows.
    if (!isfinite(reportedHeadingDeg)) return NAN;
    float err = fmodf(reportedHeadingDeg, 360.0f);
    if (err > 180.0f) err -= 360.0f;
    if (err <= -180.0f) err += 360.0f;
    const float truth = 360.0f * static_cast<float>(spins);
    return scalarFromMeasurement(truth, truth + err);
}

}  // namespace otos
