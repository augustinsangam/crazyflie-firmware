#include "exploration/StateMachine.hpp"
#include "porting.hpp"

#include <cstring>

#define M_PI 3.14159F
#define STATE_MACHINE_COMMANDER_PRI 3

namespace exploration {

static int32_t find_minimum(uint8_t a[], int32_t n) {
	int32_t c, min, index;

	min = a[0];
	index = 0;

	for (c = 1; c < n; c++) {
		if (a[c] < min) {
			index = c;
			min = a[c];
		}
	}

	return index;
}

static void take_off(setpoint_t *sp, float velocity) {
	sp->mode.x = modeVelocity;
	sp->mode.y = modeVelocity;
	sp->mode.z = modeVelocity;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->velocity.z = velocity;
	sp->mode.yaw = modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}

static void land(setpoint_t *sp, float velocity) {
	sp->mode.x = modeVelocity;
	sp->mode.y = modeVelocity;
	sp->mode.z = modeVelocity;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->velocity.z = -velocity;
	sp->mode.yaw = modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}

static void hover(setpoint_t *sp, float height) {
	sp->mode.x = modeVelocity;
	sp->mode.y = modeVelocity;
	sp->mode.z = modeAbs;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->position.z = height;
	sp->mode.yaw = modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}

static void vel_command(setpoint_t *sp, float vel_x, float vel_y,
                        float yaw_rate, float height) {
	sp->mode.x = modeVelocity;
	sp->mode.y = modeVelocity;
	sp->mode.z = modeAbs;
	sp->velocity.x = vel_x;
	sp->velocity.y = vel_y;
	sp->position.z = height;
	sp->mode.yaw = modeVelocity;
	sp->attitudeRate.yaw = yaw_rate;
	sp->velocity_body = true;
}

static void shut_off_engines(setpoint_t *sp) {
	sp->mode.x = modeDisable;
	sp->mode.y = modeDisable;
	sp->mode.z = modeDisable;
	sp->mode.yaw = modeDisable;
}

void StateMachine::init() {
	init_median_filter_f(&medFilt, 5);
	init_median_filter_f(&medFilt_2, 5);
	init_median_filter_f(&medFilt_3, 13);
	// p2pRegisterCB(p2pcallbackHandler);
	auto address = config_block_radio_address();
	my_id = (uint8_t)((address)&0x00000000ff);

#if METHOD != 1
	p_reply.port = 0x00;
	p_reply.data[0] = my_id;
	memcpy(&p_reply.data[1], &rssi_angle, sizeof(float));
	p_reply.size = 5;
#endif

	system_wait_start();
	delay_ticks(3000);
}

void StateMachine::step() {
	// some delay before the whole thing starts
	delay_ticks(10);

	// For every 1 second, reset the RSSI value to high if it hasn't been
	// received for a while
	for (uint8_t it = 0; it < 9; it++)
		if (timestamp_us() >= time_array_other_drones[it] + 1000 * 1000) {
			time_array_other_drones[it] = timestamp_us() + 1000 * 1000 + 1;
			rssi_array_other_drones[it] = 150;
			rssi_angle_array_other_drones[it] = 500.0f;
		}

	// get RSSI, id and angle of closests crazyflie.
	auto id_inter_closest = (uint8_t)find_minimum(rssi_array_other_drones, 9);
	auto rssi_inter_closest = rssi_array_other_drones[id_inter_closest];
	auto rssi_angle_inter_closest =
	    rssi_angle_array_other_drones[id_inter_closest];

	auto rssi_inter_filtered =
	    (uint8_t)update_median_filter_f(&medFilt_2, (float)rssi_inter_closest);

	// checking init of multiranger and flowdeck
	uint8_t multiranger_isinit = deck_bc_multiranger();
	uint8_t flowdeck_isinit = deck_bc_flow2();

	// get current height and heading
	auto height = kalman_state_z();
	float heading_deg = stabilizer_yaw();
	auto heading_rad = heading_deg * (float)M_PI / 180.0f;

	// t RSSI of beacon
	rssi_beacon = radio_rssi();
	auto rssi_beacon_filtered =
	    (uint8_t)update_median_filter_f(&medFilt_3, (float)rssi_beacon);

	/* filter rssi
	static int pos_avg = 0;
	static long sum = 0;
	static int arrNumbers[26] = {35};
	static int len = sizeof(arrNumbers) / sizeof(int);
	rssi_beacon_filtered = (uint8_t)movingAvg(arrNumbers, &sum, pos_avg,
	len, (int)rssi_beacon);*/

	// Select which laser range sensor readings to use
	if (multiranger_isinit) {
		front_range = (float)range_front() / 1000.0f;
		right_range = (float)range_right() / 1000.0f;
		left_range = (float)range_left() / 1000.0f;
		back_range = (float)range_back() / 1000.0f;
		up_range = (float)range_up() / 1000.0f;
	}

	// Get position estimate of kalman filter
	point_t pos;
	kalman_estimated_pos(&pos);

	// Initialize setpoint
	setpoint_t setpoint_BG;
	memset(&setpoint_BG, 0, sizeof(setpoint_BG));

	// Filtere uprange, since it sometimes gives a low spike that
	auto up_range_filtered = update_median_filter_f(&medFilt, up_range);
	if (up_range_filtered < 0.05f) {
		up_range_filtered = up_range;
	}
	// up_range_filtered = 1.0f;
	//***************** Manual Startup procedure*************//

	// TODO: shut off engines when crazyflie is on it's back.

	/*    // indicate if top range is hit while it is not flying yet, then
	   start counting if (keep_flying == false && manual_startup==false &&
	   up_range <0.2f && on_the_ground == true)
	    {
	      manual_startup = true;
	      time_stamp_manual_startup_command = xTaskGetTickCount();
	    }

	    // While still on the ground, but indicated that manual startup is
	   requested, keep checking the time if (keep_flying == false &&
	   manual_startup == true)
	    {
	        uint32_t currentTime = xTaskGetTickCount();
	        // If 3 seconds has passed, start flying.
	        if ((currentTime -time_stamp_manual_startup_command) >
	   MANUAL_STARTUP_TIMEOUT)
	        {
	          keep_flying = true;
	          manual_startup = false;
	        }
	    }*/

	// Don't fly if multiranger/updownlaser is not connected or the uprange
	// is activated

	if (flowdeck_isinit && multiranger_isinit) {
		correctly_initialized = true;
	}

#if METHOD == 3
	uint8_t rssi_beacon_threshold = 41;
	if (keep_flying == true &&
	    (!correctly_initialized || up_range < 0.2f ||
	     (!outbound_ && rssi_beacon_filtered < rssi_beacon_threshold))) {
		keep_flying = 0;
	}
#else
	if (keep_flying == true && (!correctly_initialized || up_range < 0.2f)) {
		keep_flying = 0;
	}
#endif

	state = 0;

	// Main flying code
	if (keep_flying) {
		if (taken_off) {
			/*
			 * If the flight is given a OK
			 *  and the crazyflie has taken off
			 *   then perform state machine
			 */
			float vel_w_cmd = 0;
			float vel_x_cmd;
			float vel_y_cmd;

			hover(&setpoint_BG, nominal_height);

#if METHOD == 1 // WALL_FOLLOWING
                // wall following state machine
			// state = exploration_controller_.wall_follower(
			state = exploration_controller_.wall_follower(
			    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, right_range,
			    heading_rad, 1);
#endif
#if METHOD == 2 // WALL_FOLLOWER_AND_AVOID
			if (id_inter_closest > my_id) {
				rssi_inter_filtered = 140;
			}

			state = exploration_controller_.wall_follower(
			    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, left_range,
			    right_range, heading_rad, rssi_inter_filtered);
#endif
#if METHOD == 3 // SwWARM GRADIENT BUG ALGORITHM

			bool priority = false;
			if (id_inter_closest > my_id) {
				priority = true;
			} else {
				priority = false;
			}
			// TODO make outbound depended on battery.
			state = exploration_controller_.controller(
			    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, &rssi_angle, &state_wf_,
			    front_range, left_range, right_range, back_range, heading_rad,
			    (float)pos.x, (float)pos.y, rssi_beacon_filtered,
			    rssi_inter_filtered, rssi_angle_inter_closest, priority,
			    outbound_);

			memcpy(&p_reply.data[1], &rssi_angle, sizeof(float));

#endif

			// convert yaw rate commands to degrees
			float vel_w_cmd_convert = vel_w_cmd * 180.0f / (float)M_PI;

			// Convert relative commands to world commands (not necessary
			// anymore)
			/*float psi = heading_rad;
			float vel_x_cmd_convert =  cosf(-psi) * vel_x_cmd + sinf(-psi) *
			vel_y_cmd; float vel_y_cmd_convert = -sinf(-psi) * vel_x_cmd +
			cosf(-psi) * vel_y_cmd;*/
			// float vel_y_cmd_convert = -1 * vel_y_cmd;
			vel_command(&setpoint_BG, vel_x_cmd, vel_y_cmd, vel_w_cmd_convert,
			            nominal_height);
			// on_the_ground = false;
		} else {
			/*
			 * If the flight is given a OK
			 *  but the crazyflie  has not taken off
			 *   then take off
			 */
			if (timestamp_us() >= takeoffdelaytime + 1000 * 1000 * my_id) {

				take_off(&setpoint_BG, nominal_height);
				if (height > nominal_height) {
					taken_off = true;

#if METHOD == 1 // wall following
                // exploration_controller_.init(0.4F, 0.5F, 1);
					exploration_controller_.wall_follower_init(0.4F, 0.5, 1);
#elif METHOD == 2 // wallfollowing with avoid
					if (my_id % 2 == 1) {
						exploration_controller_.wall_follower_init(0.4F, 0.5,
						                                           -1);
					} else {
						exploration_controller_.wall_follower_init(0.4F, 0.5,
						                                           1);
					}
#elif METHOD == 3 // Swarm Gradient Bug Algorithm
					if (my_id == 4 || my_id == 8) {
						exploration_controller_.init(0.4, 0.5, -0.8);
					} else if (my_id == 2 || my_id == 6) {
						exploration_controller_.init(0.4, 0.5, 0.8);
					} else if (my_id == 3 || my_id == 7) {
						exploration_controller_.init(0.4, 0.5, -2.4);
					} else if (my_id == 5 || my_id == 9) {
						exploration_controller_.init(0.4, 0.5, 2.4);
					} else {
						exploration_controller_.init(0.4, 0.5, 0.8);
					}
#endif
				}
				// on_the_ground = false;
			} else {
				shut_off_engines(&setpoint_BG);
				taken_off = false;
			}
		}
	} else {
		if (taken_off) {
			/*
			 * If the flight is given a not OK
			 *  but the crazyflie  has already taken off
			 *   then land
			 */
			land(&setpoint_BG, 0.2f);
			if (height < 0.1f) {
				shut_off_engines(&setpoint_BG);
				taken_off = false;
			}
			// on_the_ground = false;

		} else {

			/*
			 * If the flight is given a not OK
			 *  and crazyflie has landed
			 *   then keep engines off
			 */
			shut_off_engines(&setpoint_BG);
			takeoffdelaytime = timestamp_us();
			// on_the_ground = true;
		}
	}

#if METHOD != 1
	if (timestamp_us() >= radioSendBroadcastTime + 1000 * 500) {
		radiolink_broadcast_packet(&p_reply);
		radioSendBroadcastTime = timestamp_us();
	}

#endif
	commander_set_point(&setpoint_BG, STATE_MACHINE_COMMANDER_PRI);
}

void StateMachine::p2p_callback_handler(P2PPacket *p) {
	auto id_inter_ext = p->data[0];

	if (id_inter_ext == 0x63) {
		// rssi_beacon =rssi_inter;
		keep_flying = p->data[1];
	} else if (id_inter_ext == 0x64) {
		rssi_beacon = p->rssi;

	} else {
		auto rssi_inter = p->rssi;
		float rssi_angle_inter_ext;
		memcpy(&rssi_angle_inter_ext, &p->data[1], sizeof(float));

		rssi_array_other_drones[id_inter_ext] = rssi_inter;
		time_array_other_drones[id_inter_ext] = timestamp_us();
		rssi_angle_array_other_drones[id_inter_ext] = rssi_angle_inter_ext;
	}
}

} // namespace exploration
