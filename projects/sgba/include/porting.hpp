#ifndef PORTING_HPP
#define PORTING_HPP

#include <cmath>
#include <cstdint>

#include "exploration/types.hpp"

#define M2T(x) x

void kalman_estimated_pos(exploration::point_t *pos);

void p2p_register_cb(void (*cb)(exploration::P2PPacket *));

void radiolink_broadcast_packet(exploration::P2PPacket *packet);

void system_wait_start();

void delay_ticks(uint32_t ticks);

void commander_set_point(exploration::setpoint_t *sp, int prio);

uint64_t timestamp_us();
uint64_t config_block_radio_address();

uint8_t deck_bc_multiranger();
uint8_t deck_bc_flow2();

uint8_t radio_rssi();

float_t kalman_state_z();

float_t stabilizer_yaw();

float_t range_front();
float_t range_left();
float_t range_back();
float_t range_right();
float_t range_up();

#endif /* PORTING_HPP */
