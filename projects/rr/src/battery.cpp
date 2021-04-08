#include "battery.hpp"

extern "C" {
#include <pm.h>
}

#include <array>
#include <cstdint>

const static std::array battery_thresholds = {
    3.27F, // 0   %
    3.61F, // 5   %
    3.69F, // 10  %
    3.71F, // 15  %
    3.73F, // 20  %
    3.75F, // 25  %
    3.77F, // 30  %
    3.79F, // 35  %
    3.80F, // 40  %
    3.82F, // 45  %
    3.84F, // 50  %
    3.85F, // 55  %
    3.87F, // 60  %
    3.91F, // 65  %
    3.95F, // 70  %
    3.98F, // 75  %
    4.02F, // 80  %
    4.08F, // 85  %
    4.11F, // 90  %
    4.15F, // 95  %
    4.20F  // 100 %
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
