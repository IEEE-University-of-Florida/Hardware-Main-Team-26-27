#pragma once

#include "sdkconfig.h"

namespace hub_board {
constexpr int kOtosSdaGpio = CONFIG_HUB_OTOS_SDA_GPIO;
constexpr int kOtosSclGpio = CONFIG_HUB_OTOS_SCL_GPIO;
constexpr unsigned int kOtosI2cHz = 100000;
}
