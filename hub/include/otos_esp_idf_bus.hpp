#pragma once

#include "driver/i2c_master.h"
#include "otos.h"

class OtosEspIdfBus final : public otos::Bus {
public:
    OtosEspIdfBus(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t device);

    bool ping() override;
    bool read(uint8_t reg, uint8_t* data, size_t length) override;
    bool write(uint8_t reg, const uint8_t* data, size_t length) override;
    void delayMs(uint32_t milliseconds) override;

private:
    i2c_master_bus_handle_t bus_;
    i2c_master_dev_handle_t device_;
};
