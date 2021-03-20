#ifndef COMPAT_H
#define COMPAT_H

#include "porting.h"

#define M2T(x) x
#define commanderSetSetpoint(x, y) commander_set_point(x, y)
#define configblockGetRadioAddress() config_block_radio_address()
#define estimatorKalmanGetEstimatedPos(x) kalman_estimated_pos(x)
#define logGetVarIdkalmanstateZ() kalman_state_z()
#define logGetVarIdradiorssi() radio_rssi()
#define logGetVarIdstabilizeryaw() stabilizer_yaw()
#define p2pRegisterCB(x) p2p_register_cb(x)
#define paramGetVarIddeckbcFlow2() deck_bc_flow2()
#define paramGetVarIddeckbcMultiranger() deck_bc_multiranger()
#define rangeGetrangeBack() range_back()
#define rangeGetrangeFront() range_front()
#define rangeGetrangeLeft() range_left()
#define rangeGetrangeRight() range_right()
#define rangeGetrangeUp() range_up()
#define systemWaitStart() system_wait_start()
#define usecTimestamp() timestamp_us()
#define vTaskDelay(x) delay_ticks(x)

#endif /* COMPAT_H */
