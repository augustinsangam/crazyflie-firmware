#include "battery.hpp"

extern "C" {
#include <pm.h>
}

#include <array>
#include <cstdint>

const static std::array battery_thresholds = {
    3.00F,  // 00%
    3.60F,  // 05%
    3.78F,  // 10%
    3.805F, // 15%
    3.83F,  // 20%
    3.85F,  // 25%
    3.87F,  // 30%
    3.88F,  // 35%
    3.89F,  // 40%
    3.905F, // 45%
    3.92F,  // 50%
    3.94F,  // 55%
    3.96F,  // 60%
    3.98F,  // 65%
    4.00F,  // 70%
    4.02F,  // 75%
    4.04F,  // 80%
    4.07F,  // 85%
    4.10F   // 90%
};

static float pmBatteryChargeFromVoltage(float voltage) {
	if (voltage < battery_thresholds[0]) {
		return 0.0F;
	}

	if (voltage > battery_thresholds.back()) {
		return 100.0F;
	}

	std::size_t charge = 0;
	while (voltage > battery_thresholds.at(charge)) {
		++charge;
	}
	return static_cast<float>(charge) * 10.0F / 2.0F;
}

float getBatteryPercentage() {
	return pmBatteryChargeFromVoltage(pmGetBatteryVoltage());
}
