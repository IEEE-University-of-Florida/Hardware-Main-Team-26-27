// test_otos.cpp - runs the driver against the simulated sensor.
// Expected values are computed by hand from the sensor's full-scale ranges
// (not by re-using the driver's constants), so a wrong constant fails here.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "otos.h"
#include "otos_localization.hpp"
#include "sim_otos.h"

using namespace otos;

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (cond) { g_pass++; }                                            \
        else { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); }  \
    } while (0)
#define NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

static void testConnect() {
    { SimBus s; Otos o; CHECK(o.begin(s) == Err::Ok); }
    { SimBus s; s.present = false; Otos o; CHECK(o.begin(s) == Err::NotFound);
      CHECK(o.resetTracking() == Err::NotBegun); }  // failed begin leaves it unusable
    { SimBus s; s.regs[0] = 0x12; Otos o; CHECK(o.begin(s) == Err::BadProductId); }
    { SimBus s; s.failAfterOps = 0; Otos o; CHECK(o.begin(s) == Err::Bus); }
    { Otos o; CHECK(o.isConnected() == Err::NotBegun); }
    { SimBus s; Otos o; o.begin(s); uint8_t hw = 0, fw = 0;
      CHECK(o.getVersion(hw, fw) == Err::Ok); CHECK(hw == 0x10 && fw == 0x10); }
}

static void testSelfTest() {
    { SimBus s; Otos o; o.begin(s); CHECK(o.selfTest() == Err::Ok);
      CHECK((s.regs[0x0F] & 0x04) != 0); }
    { SimBus s; s.selfTestPasses = false; Otos o; o.begin(s);
      CHECK(o.selfTest() == Err::SelfTestFailed); }
    { SimBus s; s.selfTestNeverEnds = true; Otos o; o.begin(s);
      CHECK(o.selfTest() == Err::Timeout); }
}

static void testCalibrate() {
    { SimBus s; Otos o; o.begin(s);
      CHECK(o.calibrateImu() == Err::Ok); CHECK(s.regs[0x06] == 0);
      CHECK(s.totalDelayMs < 200); }
    { SimBus s; s.calibStuck = true; Otos o; o.begin(s);
      CHECK(o.calibrateImu(50) == Err::Timeout); }
    { SimBus s; Otos o; o.begin(s); CHECK(o.calibrateImu(0) == Err::OutOfRange); }
    { SimBus s; Otos o; o.begin(s);
      CHECK(o.calibrateImu(200, false) == Err::Ok); CHECK(s.regs[0x06] == 200);
      uint8_t left = 0; CHECK(o.imuCalibrationRemaining(left) == Err::Ok);
      CHECK(left == 100); }  // sim drops 100 per read
}

static void testDecode() {
    SimBus s; Otos o; o.begin(s);
    Pose p;

    // +-10 m, +-pi full scale
    s.setRaw16(0x20, 16384); s.setRaw16(0x22, -32768); s.setRaw16(0x24, 16384);
    CHECK(o.getPosition(p) == Err::Ok);
    NEAR(p.x, 5.0f, 1e-4f);
    NEAR(p.y, -10.0f, 1e-4f);
    NEAR(p.h, 90.0f, 1e-3f);

    // explicit little-endian bytes: 0x1234 = 4660 -> 4660 * 10 / 32768
    s.regs[0x20] = 0x34; s.regs[0x21] = 0x12;
    o.getPosition(p);
    NEAR(p.x, 1.4221191f, 1e-4f);

    // small negative: 0xFFFF = -1 raw
    s.regs[0x20] = 0xFF; s.regs[0x21] = 0xFF;
    o.getPosition(p);
    NEAR(p.x, -10.0f / 32768.0f, 1e-7f);

    // units
    s.setRaw16(0x20, 16384); s.setRaw16(0x24, 16384);
    o.setLinearUnit(LinearUnit::Inches); o.setAngularUnit(AngularUnit::Radians);
    o.getPosition(p);
    NEAR(p.x, 5.0f * 39.37f, 1e-2f);
    NEAR(p.h, 1.5707963f, 1e-4f);
    o.setLinearUnit(LinearUnit::Meters); o.setAngularUnit(AngularUnit::Degrees);

    // velocity: +-5 m/s, +-2000 deg/s; accel: +-16 g, +-pi*1000 rad/s^2
    for (uint8_t i = 0; i < 3; i++) {
        s.setRaw16(static_cast<uint8_t>(0x26 + 2 * i), 16384);
        s.setRaw16(static_cast<uint8_t>(0x2C + 2 * i), 16384);
    }
    Pose v, a, pos;
    CHECK(o.getPosVelAcc(pos, v, a) == Err::Ok);
    NEAR(v.x, 2.5f, 1e-4f);
    NEAR(v.h, 1000.0f, 1e-2f);
    NEAR(a.x, 0.5f * 16.0f * 9.80665f, 1e-3f);
    NEAR(a.h, 90000.0f, 1.0f);  // 0.5 * pi * 1000 rad/s^2 in deg/s^2
    NEAR(pos.x, 5.0f, 1e-4f);

    // single-call getters agree with the burst read
    Pose v2, a2;
    o.getVelocity(v2); o.getAcceleration(a2);
    NEAR(v2.y, v.y, 1e-6f); NEAR(a2.h, a.h, 1e-3f);
}

static void testSetPosition() {
    SimBus s; Otos o; o.begin(s);
    Pose p = {5.0f, -5.0f, 90.0f};
    CHECK(o.setPosition(p) == Err::Ok);
    CHECK(s.getRaw16(0x20) == 16384);
    CHECK(s.getRaw16(0x22) == -16384);
    CHECK(s.getRaw16(0x24) == 16384);
    Pose r; o.getPosition(r);
    NEAR(r.x, 5.0f, 1e-3f); NEAR(r.y, -5.0f, 1e-3f); NEAR(r.h, 90.0f, 1e-2f);

    Pose neg = {0.0f, 0.0f, -90.0f};
    o.setPosition(neg);
    CHECK(s.regs[0x24] == 0x00 && s.regs[0x25] == 0xC0);  // 0xC000

    Pose half = {0.0f, 0.0f, 180.0f};  // +180 wraps to -180 = -32768
    CHECK(o.setPosition(half) == Err::Ok);
    CHECK(s.getRaw16(0x24) == -32768);

    Pose wrap = {0.0f, 0.0f, 450.0f};  // 450 = 90
    o.setPosition(wrap);
    CHECK(s.getRaw16(0x24) == 16384);

    Pose far = {100.0f, 0.0f, 0.0f};  // beyond +-10 m
    CHECK(o.setPosition(far) == Err::OutOfRange);
    Pose nan = {0.0f, 0.0f, NAN};
    CHECK(o.setPosition(nan) == Err::OutOfRange);
    Pose infinite = {0.0f, 0.0f, INFINITY};
    CHECK(o.setPosition(infinite) == Err::OutOfRange);
    Pose manyRotations = {0.0f, 0.0f, 1e30f};
    CHECK(o.setPosition(manyRotations) == Err::Ok);

    // inches: 196.85 in ~ 5 m
    o.setLinearUnit(LinearUnit::Inches);
    Pose in = {196.85f, 0.0f, 0.0f};
    CHECK(o.setPosition(in) == Err::Ok);
    CHECK(abs(s.getRaw16(0x20) - 16384) <= 2);
}

static void testOffsetAndReset() {
    SimBus s; Otos o; o.begin(s);
    Pose off = {0.05f, -0.10f, 0.0f};
    CHECK(o.setOffset(off) == Err::Ok);
    Pose g; CHECK(o.getOffset(g) == Err::Ok);
    NEAR(g.x, 0.05f, 1e-3f); NEAR(g.y, -0.10f, 1e-3f);
    CHECK(s.regs[0x10] != 0 || s.regs[0x11] != 0);  // really written at 0x10

    const int16_t offRawBefore = s.getRaw16(0x10);
    s.setRaw16(0x20, 1000); s.setRaw16(0x26, 500);
    CHECK(o.resetTracking() == Err::Ok);
    CHECK(s.getRaw16(0x20) == 0 && s.getRaw16(0x26) == 0);
    CHECK(s.getRaw16(0x10) == offRawBefore);  // offset not wiped by reset
}

static void testScalars() {
    SimBus s; Otos o; o.begin(s);
    float v = 0;
    CHECK(o.setLinearScalar(0.971f) == Err::Ok);
    CHECK(s.regs[0x04] == 0xE3);  // -29
    CHECK(o.getLinearScalar(v) == Err::Ok); NEAR(v, 0.971f, 1e-5f);
    CHECK(o.setAngularScalar(1.004f) == Err::Ok);
    CHECK(s.regs[0x05] == 4);
    CHECK(o.getAngularScalar(v) == Err::Ok); NEAR(v, 1.004f, 1e-5f);
    CHECK(o.setLinearScalar(1.0f) == Err::Ok); CHECK(s.regs[0x04] == 0);
    CHECK(o.setLinearScalar(0.872f) == Err::Ok); CHECK(s.regs[0x04] == 0x80);
    CHECK(o.setLinearScalar(1.127f) == Err::Ok); CHECK(s.regs[0x04] == 127);
    CHECK(o.setLinearScalar(0.870f) == Err::OutOfRange);
    CHECK(o.setLinearScalar(1.130f) == Err::OutOfRange);
    CHECK(o.setLinearScalar(0.0f) == Err::OutOfRange);
    CHECK(o.setAngularScalar(NAN) == Err::OutOfRange);
    CHECK(o.setLinearScalar(INFINITY) == Err::OutOfRange);
    CHECK(o.setLinearScalar(1e30f) == Err::OutOfRange);
    CHECK(s.regs[0x04] == 127);  // rejected writes changed nothing
}

static void testStatus() {
    SimBus s; Otos o; o.begin(s); Status st;
    CHECK(o.getStatus(st) == Err::Ok); CHECK(!st.anyProblem());
    s.regs[0x1F] = 0x02;
    o.getStatus(st); CHECK(st.warnOpticalTracking && !st.warnTiltAngle && st.anyProblem());
    s.regs[0x1F] = 0x01 | 0x40 | 0x80;
    o.getStatus(st);
    CHECK(st.warnTiltAngle && st.errorPaa && st.errorLsm && !st.warnOpticalTracking);
}

static void testBusFailures() {
    SimBus s; Otos o; o.begin(s);
    s.failAfterOps = s.ops;  // everything from now on fails
    Pose p = {1, 2, 3}; Pose a, b, c; float f = 0; Status st; uint8_t u;
    CHECK(o.getPosition(p) == Err::Bus);
    NEAR(p.x, 1, 0); NEAR(p.h, 3, 0);  // output untouched on failure
    CHECK(o.getPosVelAcc(a, b, c) == Err::Bus);
    CHECK(o.setPosition(p) == Err::Bus);
    CHECK(o.getLinearScalar(f) == Err::Bus);
    CHECK(o.setAngularScalar(1.0f) == Err::Bus);
    CHECK(o.getStatus(st) == Err::Bus);
    CHECK(o.resetTracking() == Err::Bus);
    CHECK(o.selfTest() == Err::Bus);
    CHECK(o.calibrateImu() == Err::Bus);
    CHECK(o.getVersion(u, u) == Err::Bus);
}

static void testCalibrationHelpers() {
    // SparkFun's worked examples
    NEAR(Otos::angularScalarFromSpins(10, -15.0f), 3600.0f / 3585.0f, 1e-5f);
    NEAR(Otos::scalarFromMeasurement(100.0f, 103.0f), 0.97087f, 1e-4f);
    // clockwise spins: truth negative, sensor reports +15 leftover
    NEAR(Otos::angularScalarFromSpins(-10, 15.0f), 3600.0f / 3585.0f, 1e-5f);
    // overshoot: sensor counts too much
    CHECK(Otos::angularScalarFromSpins(10, 15.0f) < 1.0f);
    // perfect
    NEAR(Otos::angularScalarFromSpins(10, 0.0f), 1.0f, 1e-6f);
    // degenerate input does not divide by zero
    NEAR(Otos::scalarFromMeasurement(100.0f, 0.0f), 1.0f, 0);
    CHECK(isnan(Otos::angularScalarFromSpins(10, INFINITY)));
}

static void testHubLocalization() {
    SimBus bus;
    OtosLocalization localization;
    OtosSample sample{};
    CHECK(localization.read(1, sample) == Err::NotBegun);

    bus.selfTestPasses = false;
    CHECK(localization.begin(bus, OtosCalibration{}) == Err::SelfTestFailed);
    CHECK(localization.read(1, sample) == Err::NotBegun);
    bus.selfTestPasses = true;
    CHECK(localization.begin(bus, OtosCalibration{}) == Err::Ok);

    bus.setRaw16(0x20, 16384);
    bus.setRaw16(0x24, 16384);
    CHECK(localization.read(42, sample) == Err::Ok);
    CHECK(sample.valid() && sample.monotonic_time_us == 42);
    NEAR(sample.position.x, 5.0f, 1e-4f);
    NEAR(sample.position.h, 1.5707963f, 1e-4f);

    bus.regs[0x1F] = 0x02;
    CHECK(localization.read(43, sample) == Err::Ok);
    CHECK(!sample.valid() && sample.status.warnOpticalTracking);

    bus.failAfterOps = bus.ops + 1;  // Burst read succeeds, status read fails.
    CHECK(localization.read(44, sample) == Err::Bus);
    CHECK(sample.monotonic_time_us == 43);
}

int main() {
    testConnect();
    testSelfTest();
    testCalibrate();
    testDecode();
    testSetPosition();
    testOffsetAndReset();
    testScalars();
    testStatus();
    testBusFailures();
    testCalibrationHelpers();
    testHubLocalization();
    printf("%d checks passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
