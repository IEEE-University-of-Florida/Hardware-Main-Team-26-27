// sim_otos.h - in-memory fake of the OTOS register file, for testing the
// driver on a PC. Behaviour mirrors SparkFun's library/docs: little-endian
// int16 registers, auto-incrementing reads/writes, countdown IMU calibration,
// self-test with in-progress/pass/fail bits.
#ifndef SIM_OTOS_H
#define SIM_OTOS_H

#include <string.h>

#include "otos.h"

class SimBus : public otos::Bus {
  public:
    uint8_t regs[256];
    bool present = true;           // ping() result
    int failAfterOps = -1;         // >=0: bus ops start failing after this many
    int ops = 0;
    uint32_t totalDelayMs = 0;
    bool calibStuck = false;       // calibration never counts down
    int selfTestBusyReads = 2;     // reads that show "in progress"
    bool selfTestPasses = true;
    bool selfTestNeverEnds = false;

    SimBus() {
        memset(regs, 0, sizeof(regs));
        regs[0x00] = otos::kProductId;
        regs[0x01] = 0x10;  // hw 1.0
        regs[0x02] = 0x10;  // fw 1.0
    }

    void setRaw16(uint8_t reg, int16_t v) {
        regs[reg] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);
        regs[reg + 1] = static_cast<uint8_t>(static_cast<uint16_t>(v) >> 8);
    }
    int16_t getRaw16(uint8_t reg) const {
        return static_cast<int16_t>(regs[reg] | (regs[reg + 1] << 8));
    }

    bool ping() override { return present; }

    bool read(uint8_t reg, uint8_t* buf, size_t len) override {
        if (failNow()) return false;
        for (size_t i = 0; i < len; i++) {
            const uint8_t r = static_cast<uint8_t>(reg + i);
            if (r == 0x06 && regs[0x06] > 0 && !calibStuck) {
                // One read ~ 3 ms ~ 1 sample... run fast: 100 samples per read
                regs[0x06] = regs[0x06] > 100 ? regs[0x06] - 100 : 0;
            }
            if (r == 0x0F) stepSelfTest();
            buf[i] = regs[r];
        }
        return true;
    }

    bool write(uint8_t reg, const uint8_t* buf, size_t len) override {
        if (failNow()) return false;
        for (size_t i = 0; i < len; i++) {
            const uint8_t r = static_cast<uint8_t>(reg + i);
            if (r == 0x07 && (buf[i] & 1)) {  // reset tracking
                memset(&regs[0x20], 0, 18);
            } else if (r == 0x0F) {
                if (buf[i] & 1) { selfTestLeft = selfTestBusyReads; regs[0x0F] = 0x02; }
            } else if (r == 0x00 || r == 0x01 || r == 0x02 || (r >= 0x1F && r <= 0x1F)) {
                // read-only
            } else {
                regs[r] = buf[i];
            }
        }
        return true;
    }

    void delayMs(uint32_t ms) override { totalDelayMs += ms; }

  private:
    int selfTestLeft = -1;

    bool failNow() {
        ops++;
        return failAfterOps >= 0 && ops > failAfterOps;
    }
    void stepSelfTest() {
        if (selfTestLeft < 0) return;
        if (selfTestNeverEnds) { regs[0x0F] = 0x02; return; }
        if (selfTestLeft > 0) { selfTestLeft--; regs[0x0F] = 0x02; return; }
        regs[0x0F] = selfTestPasses ? 0x04 : 0x08;
        selfTestLeft = -1;
    }
};

#endif  // SIM_OTOS_H
