#include "otos_esp_idf_bus.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr int kTimeoutMs = 50;
constexpr size_t kMaxWriteLength = 6;  // One OTOS planar pose or offset.
}

OtosEspIdfBus::OtosEspIdfBus(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t device)
    : bus_(bus), device_(device) {}

bool OtosEspIdfBus::ping() {
    return i2c_master_probe(bus_, otos::kI2cAddress, kTimeoutMs) == ESP_OK;
}

bool OtosEspIdfBus::read(uint8_t reg, uint8_t* data, size_t length) {
    return i2c_master_transmit_receive(device_, &reg, 1, data, length, kTimeoutMs) == ESP_OK;
}

bool OtosEspIdfBus::write(uint8_t reg, const uint8_t* data, size_t length) {
    if (length == 0 || length > kMaxWriteLength) {
        return false;
    }
    uint8_t frame[1 + kMaxWriteLength] = {reg};
    for (size_t index = 0; index < length; ++index) {
        frame[index + 1] = data[index];
    }
    return i2c_master_transmit(device_, frame, length + 1, kTimeoutMs) == ESP_OK;
}

void OtosEspIdfBus::delayMs(uint32_t milliseconds) {
    vTaskDelay(pdMS_TO_TICKS(milliseconds) + 1);
}
