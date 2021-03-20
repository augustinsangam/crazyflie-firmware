#ifndef PORTING_H
#define PORTING_H

#include <math.h>
#include <stdint.h>

#include "types.h"

void kalman_estimated_pos(point_t *pos);

void p2p_register_cb(void (*cb)(P2PPacket *));

void system_wait_start(void);

void delay_ticks(uint32_t ticks);

void commander_set_point(setpoint_t *sp, int prio);

uint64_t timestamp_us(void);
uint64_t config_block_radio_address(void);

uint8_t deck_bc_multiranger(void);
uint8_t deck_bc_flow2(void);

uint8_t radio_rssi(void);

float_t kalman_state_z(void);

float_t stabilizer_yaw(void);

float_t range_front(void);
float_t range_left(void);
float_t range_back(void);
float_t range_right(void);
float_t range_up(void);

#endif /* PORTING_H */
