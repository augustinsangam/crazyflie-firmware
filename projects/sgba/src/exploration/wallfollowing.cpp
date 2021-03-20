/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#include "exploration/wallfollowing.hpp"
#include "math_supp.hpp"
#include "porting.hpp"
#include <math.h>

// variables
static float ref_distance_from_wall = 0;
static float max_speed = 0.5;
static float max_rate = 0.5;
static float direction = 1;
static bool first_run = false;
static int state = 1;
float state_start_time;

void wall_follower_init(float new_ref_distance_from_wall, float max_speed_ref,
                        int init_state) {
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
	first_run = true;
	state = init_state;
}

// Static helper functions
static bool logic_is_close_to(float real_value, float checked_value,
                              float margin) {
	return real_value > checked_value - margin &&
	       real_value < checked_value + margin;
}

// Static command functions
static void command_turn(float *vel_x, float *vel_w, float ref_rate) {
	*vel_x = 0.0;
	*vel_w = direction * ref_rate;
}

static void command_align_corner(float *vel_y, float *vel_w, float ref_rate,
                                 float range,
                                 float wanted_distance_from_corner) {

	if (range > wanted_distance_from_corner + 0.3F) {
		*vel_w = direction * ref_rate;
		*vel_y = 0;
		return;
	}

	if (range > wanted_distance_from_corner) {
		*vel_y = direction * (-1 * max_speed / 3);
	} else {
		*vel_y = direction * (max_speed / 3);
	}
	*vel_w = 0;
}

static void command_hover(float *vel_x, float *vel_y, float *vel_w) {
	*vel_x = 0.0;
	*vel_y = 0.0;
	*vel_w = 0.0;
}

static void command_forward_along_wall(float *vel_x, float *vel_y,
                                       float range) {
	*vel_x = max_speed;
	bool check_distance_wall =
	    logic_is_close_to(ref_distance_from_wall, range, 0.1F);
	*vel_y = 0;
	if (check_distance_wall) {
		return;
	}

	if (range > ref_distance_from_wall) {
		*vel_y = direction * (-1 * max_speed / 2);
	} else {
		*vel_y = direction * (max_speed / 2);
	}
}

static void command_turn_around_corner_and_adjust(float *vel_x, float *vel_y,
                                                  float *vel_w, float radius,
                                                  float range) {
	*vel_x = max_speed;
	*vel_w = direction * (-1 * (*vel_x) / radius);
	bool check_distance_to_wall =
	    logic_is_close_to(ref_distance_from_wall, range, 0.1F);
	if (!check_distance_to_wall) {
		if (range > ref_distance_from_wall) {
			*vel_y = direction * (-1 * max_speed / 3);

		}

		else {
			*vel_y = direction * (max_speed / 3);
		}
	}
}

static void command_turn_and_adjust(float *vel_y, float *vel_w, float rate,
                                    float /* range */) {
	*vel_w = direction * rate;
	*vel_y = 0;
}

static int transition(int new_state) {
	state_start_time =
	    static_cast<float>(static_cast<double>(timestamp_us()) / 1e6);
	return new_state;
}

void adjustDistanceWall(float distance_wall_new) {
	ref_distance_from_wall = distance_wall_new;
}

int wall_follower(float *vel_x, float *vel_y, float *vel_w, float front_range,
                  float side_range, float current_heading, float direction_turn) {
	direction = direction_turn;
	static float previous_heading = 0;
	static float angle = 0;
	static bool around_corner_go_back = false;
	auto now = static_cast<float>(static_cast<double>(timestamp_us()) / 1e6);

	if (first_run) {
		previous_heading = current_heading;
		//  around_corner_first_turn = false;
		around_corner_go_back = false;
		first_run = false;
	}

	/***********************************************************
	 * State definitions
	 ***********************************************************/
	// 1 = forward
	// 2 = hover
	// 3 = turn_to_find_wall
	// 4 = turn_to_allign_to_wall
	// 5 = forward along wall
	// 6 = rotate_around_wall
	// 7 = rotate_in_corner
	// 8 = find corner

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state == 1) { // FORWARD
		if (front_range < ref_distance_from_wall + 0.2F) {
			state = transition(3);
		}
	} else if (state == 2) { // HOVER

	} else if (state == 3) { // TURN_TO_FIND_WALL
		// check if wall is found
		bool side_range_check =
		    side_range < ref_distance_from_wall / std::cos(0.78F) + 0.2F;
		bool front_range_check =
		    front_range < ref_distance_from_wall / std::cos(0.78F) + 0.2F;
		if (side_range_check && front_range_check) {
			previous_heading = current_heading;
			angle = direction *
			        (1.57F - std::atan(front_range / side_range) + 0.1F);
			state = transition(4); // go to turn_to_allign_to_wall
		}
		if (side_range < 1.0F && front_range > 2.0F) {
			//  around_corner_first_turn = true;
			around_corner_go_back = false;
			previous_heading = current_heading;
			state = transition(8); // go to rotate_around_wall
		}
	} else if (state == 4) { // TURN_TO_ALLIGN_TO_WALL
		bool allign_wall_check = logic_is_close_to(
		    wrap_to_pi(current_heading - previous_heading), angle, 0.1F);
		if (allign_wall_check) {
			// prev_side_range = side_range;
			state = transition(5);
		}
	} else if (state == 5) { // FORWARD_ALONG_WALL

		// If side range is out of reach,
		//    end of the wall is reached
		if (side_range > ref_distance_from_wall + 0.3F) {
			//  around_corner_first_turn = true;
			state = transition(8);
		}
		// If front range is small
		//    then corner is reached
		if (front_range < ref_distance_from_wall + 0.2F) {
			previous_heading = current_heading;
			state = transition(7);
		}

	} else if (state == 6) { // ROTATE_AROUND_WALL
		if (front_range < ref_distance_from_wall + 0.2F) {
			state = transition(3);
		}

	} else if (state == 7) { // ROTATE_IN_CORNER
		// Check if heading goes over 0.8 rad
		bool check_heading_corner = logic_is_close_to(
		    std::fabs(wrap_to_pi(current_heading - previous_heading)), 0.8F,
		    0.1F);
		if (check_heading_corner) {
			state = transition(3);
		}

	} else if (state == 8) { // FIND_CORNER
		if (side_range <= ref_distance_from_wall) {
			state = transition(6);
		}

	}

	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x = 0;
	float temp_vel_y = 0;
	float temp_vel_w = 0;

	if (state == 1) { // FORWARD
		temp_vel_x = max_speed;
		temp_vel_y = 0.0;
		temp_vel_w = 0.0;

	} else if (state == 2) { // HOVER
		command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);

	} else if (state == 3) { // TURN_TO_FIND_WALL
		command_turn(&temp_vel_x, &temp_vel_w, max_rate);
		temp_vel_y = 0.0;

	} else if (state == 4) { // TURN_TO_ALLIGN_TO_WALL

		if (now - state_start_time < 1.0F) {
			command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
		} else { // then turn again
			command_turn(&temp_vel_x, &temp_vel_w, max_rate);
			temp_vel_y = 0;
		}

	} else if (state == 5) { // FORWARD_ALONG_WALL
		command_forward_along_wall(&temp_vel_x, &temp_vel_y, side_range);
		temp_vel_w = 0.0F;

		// command_forward_along_wallHeadingSine(&temp_vel_x,
		// &temp_vel_y,&temp_vel_w, side_range);

	} else if (state == 6) { // ROTATE_AROUND_WALL
		// If first time around corner
		// first try to find the corner again

		// if side range is larger than prefered distance from wall
		if (side_range > ref_distance_from_wall + 0.5F) {

			// check if scanning has already occured
			if (wrap_to_pi(std::fabs(current_heading - previous_heading)) >
			    0.8F) {
				around_corner_go_back = true;
			}
			// turn and adjust distnace to corner from that point
			if (around_corner_go_back) {
				// go back if it already went into one direction
				command_turn_and_adjust(&temp_vel_y, &temp_vel_w, -1 * max_rate,
				                        side_range);
				temp_vel_x = 0.0F;
			} else {
				command_turn_and_adjust(&temp_vel_y, &temp_vel_w, max_rate,
				                        side_range);
				temp_vel_x = 0.0F;
			}
		} else {
			// continue to turn around corner
			previous_heading = current_heading;
			around_corner_go_back = false;
			command_turn_around_corner_and_adjust(
			    &temp_vel_x, &temp_vel_y, &temp_vel_w, ref_distance_from_wall,
			    side_range);
		}

	} else if (state == 7) { // ROTATE_IN_CORNER
		command_turn(&temp_vel_x, &temp_vel_w, max_rate);
		temp_vel_y = 0;

	} else if (state == 8) { // FIND_CORNER
		command_align_corner(&temp_vel_y, &temp_vel_w, -1 * max_rate,
		                     side_range, ref_distance_from_wall);
		temp_vel_x = 0;
	}

	else {
		// State does not exist so hover!!
		command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
	}

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;

	return state;
}
