#include <cstdint>

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <commander.h>
#include <configblock.h>
#include <estimator_kalman.h>
#include <log.h>
#include <param.h>
#include <radiolink.h>
#include <range.h>
#include <stabilizer_types.h>
#include <system.h>
#include <task.h>
#include <usec_time.h>

#include <porting/porting.hpp>

/**
 * Get microsecond-resolution timestamp.
 */
uint64_t porting::us_timestamp(void) { return usecTimestamp(); }

uint64_t porting::config_block_get_radio_address(void) {
	return configblockGetRadioAddress();
}

void porting::system_wait_start(void) { systemWaitStart(); }

void porting::ticks_delay(uint32_t nTicksToDelay) { vTaskDelay(nTicksToDelay); }
uint32_t porting::ms_to_ticks(uint16_t ms) { return M2T(ms); }

void porting::commander_set_setpoint(exploration::setpoint_t *setpoint,
                                     int priority) {
	commanderSetSetpoint(reinterpret_cast<setpoint_t *>(setpoint), priority);
}

void porting::estimator_kalman_get_estimated_pos(exploration::point_t *pos) {
	return estimatorKalmanGetEstimatedPos(reinterpret_cast<point_t *>(pos));
}

bool porting::send_p2p_packet_broadcast(exploration::P2PPacket *p2pp) {
	return radiolinkSendP2PPacketBroadcast(reinterpret_cast<P2PPacket *>(p2pp));
}

uint8_t porting::get_deck_bc_multiranger() {
	paramVarId_t varid = paramGetVarId("deck", "bcMultiranger");
	return static_cast<uint8_t>(paramGetInt(varid));
}

uint8_t porting::get_deck_bc_flow2() {
	paramVarId_t varid = paramGetVarId("deck", "bcFlow2");
	return static_cast<uint8_t>(paramGetUint(varid));
}

float porting::get_kalman_state_z() {
	logVarId_t varid = logGetVarId("kalman", "stateZ");
	return logGetFloat(varid);
}

float porting::get_stabilizer_yaw() {
	logVarId_t varid = logGetVarId("stabilizer", "yaw");
	return logGetFloat(varid);
}

uint8_t porting::get_radio_rssi() {
	logVarId_t varid = logGetVarId("radio", "rssi");
	return static_cast<uint8_t>(logGetFloat(varid));
}

/* Between 0 and 1 */
float porting::get_front_range() {
	return rangeGet(rangeDirection_t::rangeFront);
}

/* Between 0 and 1 */
float porting::get_right_range() {
	return rangeGet(rangeDirection_t::rangeFront);
}

/* Between 0 and 1 */
float porting::get_left_range() {
	return rangeGet(rangeDirection_t::rangeLeft);
}

/* Between 0 and 1 */
float porting::get_back_range() {
	return rangeGet(rangeDirection_t::rangeBack);
}

/* Between 0 and 1 */
float porting::get_up_range() { return rangeGet(rangeDirection_t::rangeUp); }
