#include "porting.hpp"

#include <cmath>
#include <cstdint>

namespace fr {
extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}
} // namespace fr

extern "C" namespace cf {
#include <commander.h>
#include <configblock.h>
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

void Porting::kalman_estimated_pos(exploration::point_t *pos) {
	cf::estimatorKalmanGetEstimatedPos(reinterpret_cast<cf::point_t *>(pos));
}

void Porting::p2p_register_cb(void (*cb)(exploration::P2PPacket *)) {
	cf::p2pRegisterCB(reinterpret_cast<cf::P2PCallback>(cb));
}

void Porting::radiolink_broadcast_packet(exploration::P2PPacket *packet) {
	cf::radiolinkSendP2PPacketBroadcast(
	    reinterpret_cast<cf::P2PPacket *>(packet));
}

void Porting::system_wait_start() { cf::systemWaitStart(); }

void Porting::delay_ticks(uint32_t ticks) { fr::vTaskDelay(ticks); }

void Porting::commander_set_point(exploration::setpoint_t *sp, int prio) {
	cf::commanderSetSetpoint(reinterpret_cast<cf::setpoint_t *>(sp), prio);
}

std::uint64_t Porting::config_block_radio_address() {
	return cf::configblockGetRadioAddress();
}

std::uint8_t Porting::deck_bc_multiranger() {
	auto var_id = cf::paramGetVarId("deck", "bcMultiranger");
	return static_cast<std::uint8_t>(cf::paramGetInt(var_id));
}

std::uint8_t Porting::deck_bc_flow2() {
	auto var_id = cf::paramGetVarId("deck", "bcFlow2");
	return static_cast<std::uint8_t>(cf::paramGetUint(var_id));
}

std::uint8_t Porting::radio_rssi() {
	auto var_id = cf::logGetVarId("radio", "rssi");
	return *static_cast<std::uint8_t *>(cf::logGetAddress(var_id));
}

std::float_t Porting::kalman_state_z() {
	auto var_id = cf::logGetVarId("kalman", "stateZ");
	return *static_cast<float *>(cf::logGetAddress(var_id));
}

std::float_t Porting::stabilizer_yaw() {
	auto var_id = cf::logGetVarId("stabilizer", "yaw");
	return *static_cast<float *>(cf::logGetAddress(var_id));
}

std::float_t Porting::range_front() {
	return cf::rangeGet(cf::rangeDirection_t::rangeFront);
}

std::float_t Porting::range_left() {
	return cf::rangeGet(cf::rangeDirection_t::rangeLeft);
}

std::float_t Porting::range_back() {
	return cf::rangeGet(cf::rangeDirection_t::rangeBack);
}

std::float_t Porting::range_right() {
	return cf::rangeGet(cf::rangeDirection_t::rangeRight);
}

std::float_t Porting::range_up() {
	return cf::rangeGet(cf::rangeDirection_t::rangeUp);
}

std::uint64_t timestamp_us() { return fr::usecTimestamp(); }

} // namespace porting
