#include "board_config.hpp"
#include "otos_esp_idf_bus.hpp"
#include "otos_localization.hpp"
#include "otos_odometry.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr char kLogTag[] = "otos";
}

extern "C" void app_main() {
    using namespace hub_board;
    if (kOtosSdaGpio < 0 || kOtosSclGpio < 0 || kOtosSdaGpio == kOtosSclGpio) {
        ESP_LOGW(kLogTag, "Set distinct OTOS SDA and SCL GPIO numbers in menuconfig > Hub hardware");
        return;
    }

    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = static_cast<gpio_num_t>(kOtosSdaGpio);
    bus_config.scl_io_num = static_cast<gpio_num_t>(kOtosSclGpio);
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;

    i2c_master_bus_handle_t master = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &master));

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = otos::kI2cAddress;
    device_config.scl_speed_hz = kOtosI2cHz;

    i2c_master_dev_handle_t device = nullptr;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master, &device_config, &device));

    OtosEspIdfBus bus(master, device);
    OtosLocalization localization;
    OtosOdometryConverter odometry;
    ESP_LOGI(kLogTag, "Keep the robot still during OTOS calibration");
    // Bench bring-up only. Once running, a read failure never restarts calibration.
    auto initialized = localization.begin(bus, OtosCalibration{});
    while (initialized != otos::Err::Ok) {
        ESP_LOGW(kLogTag, "Startup failed: %s; retrying in 2 seconds", otos::errName(initialized));
        vTaskDelay(pdMS_TO_TICKS(2000));
        initialized = localization.begin(bus, OtosCalibration{});
    }

    while (true) {
        OtosSample sample{};
        OtosOdometry reading{};
        const auto result = localization.read(static_cast<uint64_t>(esp_timer_get_time()), sample);
        if (result != otos::Err::Ok) {
            odometry.markGap();
            ESP_LOGW(kLogTag, "Read failed: %s", otos::errName(result));
        } else if (!odometry.update(sample, reading)) {
            ESP_LOGW(kLogTag, "Tracking status: 0x%02x", static_cast<unsigned int>(sample.status.raw));
        } else {
            ESP_LOGI(kLogTag, "x=%.3f m y=%.3f m yaw=%.3f rad body_vx=%.3f m/s body_vy=%.3f m/s",
                     reading.odom_pose.x, reading.odom_pose.y, reading.odom_pose.h,
                     reading.body_velocity_x, reading.body_velocity_y);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
