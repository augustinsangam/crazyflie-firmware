#include "porting.hpp"
#include "battery.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>

namespace fr {
extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}
} // namespace fr

extern "C" namespace cf {
#include <commander.h>
#include <configblock.h>
#include <debug.h>
#include <estimator_kalman.h>
#include <log.h>
#include <param.h>
#include <radiolink.h>
#include <range.h>
#include <stabilizer_types.h>
#include <system.h>
#include <usec_time.h>
} // namespace cf

namespace porting {

void DroneLayer::kalman_estimated_pos(exploration::point_t *pos) {
	cf::estimatorKalmanGetEstimatedPos(reinterpret_cast<cf::point_t *>(pos));
}

bool DroneLayer::kalman_crashed() {
	auto var_id = cf::paramGetVarId("kalman", "resetEstimation");
	return cf::paramGetUint(var_id) != 0;
}

void DroneLayer::radiolink_broadcast_packet(exploration::P2PPacket *packet) {
	cf::radiolinkSendP2PPacketBroadcast(
	    reinterpret_cast<cf::P2PPacket *>(packet));
}

void DroneLayer::system_wait_start() { cf::systemWaitStart(); }

void DroneLayer::delay_ms(uint32_t ms) { fr::vTaskDelay(ms); }

void DroneLayer::commander_set_point(exploration::setpoint_t *sp, int prio) {
	cf::commanderSetSetpoint(reinterpret_cast<cf::setpoint_t *>(sp), prio);
}

std::uint64_t DroneLayer::config_block_radio_address() {
	return cf::configblockGetRadioAddress();
}

std::uint8_t DroneLayer::deck_bc_multiranger() {
	auto var_id = cf::paramGetVarId("deck", "bcMultiranger");
	return static_cast<std::uint8_t>(cf::paramGetInt(var_id));
}

std::uint8_t DroneLayer::deck_bc_flow2() {
	auto var_id = cf::paramGetVarId("deck", "bcFlow2");
	return static_cast<std::uint8_t>(cf::paramGetUint(var_id));
}

std::uint8_t DroneLayer::radio_rssi() {
	auto var_id = cf::logGetVarId("radio", "rssi");
	return *static_cast<std::uint8_t *>(cf::logGetAddress(var_id));
}

std::float_t DroneLayer::kalman_state_z() {
	auto var_id = cf::logGetVarId("kalman", "stateZ");
	return *static_cast<float *>(cf::logGetAddress(var_id));
}

std::float_t DroneLayer::stabilizer_yaw() {
	auto var_id = cf::logGetVarId("stabilizer", "yaw");
	return *static_cast<float *>(cf::logGetAddress(var_id));
}

std::float_t DroneLayer::get_battery_level() {
	// TODO ()
	// return getBatteryPercentage() / 100.0F;
	return 1.0F;
}

std::float_t DroneLayer::range_front() {
	return cf::rangeGet(cf::rangeDirection_t::rangeFront) / 1000.0F;
}

std::float_t DroneLayer::range_left() {
	return cf::rangeGet(cf::rangeDirection_t::rangeLeft) / 1000.0F;
}

std::float_t DroneLayer::range_back() {
	return cf::rangeGet(cf::rangeDirection_t::rangeBack) / 1000.0F;
}

std::float_t DroneLayer::range_right() {
	return cf::rangeGet(cf::rangeDirection_t::rangeRight) / 1000.0F;
}

std::float_t DroneLayer::range_up() {
	return cf::rangeGet(cf::rangeDirection_t::rangeUp) / 1000.0F;
}

void DroneLayer::debug_print(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	fr::eprintf(fr::consolePutchar, fmt, args);
	va_end(args);
}

std::uint64_t timestamp_us() { return fr::usecTimestamp(); }

} // namespace porting
