#include "exploration/WallFollowing.hpp"
#include "math_supp.hpp"
#include "porting.hpp"
#include <cmath>

void exploration::WallFollowing::wall_follower_init(
    float ref_distance_from_wall, float max_speed, int state) {
	ref_distance_from_wall_ = ref_distance_from_wall;
	max_speed_ = max_speed;
	first_run_ = true;
	state_ = state;
}

// Static helper functions
static bool logic_is_close_to(float real_value, float checked_value,
                              float margin) {
	return real_value > checked_value - margin &&
	       real_value < checked_value + margin;
}

// Static command functions
static void command_turn(float *vel_x, float *vel_w, float ref_rate,
                         float direction) {
	*vel_x = 0.0;
	*vel_w = direction * ref_rate;
}

static void command_align_corner(float *vel_y, float *vel_w, float ref_rate,
                                 float range, float wanted_distance_from_corner,
                                 float direction, float max_speed) {

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

static void command_forward_along_wall(float *vel_x, float *vel_y, float range,
                                       float direction, float max_speed,
                                       float ref_distance_from_wall) {
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

static void command_turn_around_corner_and_adjust(
    float *vel_x, float *vel_y, float *vel_w, float radius, float range,
    float direction, float max_speed, float ref_distance_from_wall) {
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
                                    float /* range */, float direction) {
	*vel_w = direction * rate;
	*vel_y = 0;
}

static int transition(int new_state, float *state_start_time) {
	*state_start_time =
	    static_cast<float>(static_cast<double>(timestamp_us()) / 1e6);
	return new_state;
}

void exploration::WallFollowing::adjustDistanceWall(float distance_wall_new) {
	ref_distance_from_wall_ = distance_wall_new;
}

int exploration::WallFollowing::wall_follower(float *vel_x, float *vel_y,
                                              float *vel_w, float front_range,
                                              float side_range,
                                              float current_heading,
                                              float direction_turn) {
	direction_ = direction_turn;
	static float previous_heading = 0;
	static float angle = 0;
	static bool around_corner_go_back = false;
	auto now = static_cast<float>(static_cast<double>(timestamp_us()) / 1e6);

	if (first_run_) {
		previous_heading = current_heading;
		//  around_corner_first_turn = false;
		around_corner_go_back = false;
		first_run_ = false;
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

	if (state_ == 1) { // FORWARD
		if (front_range < ref_distance_from_wall_ + 0.2F) {
			state_ = transition(3, &state_start_time_);
		}
	} else if (state_ == 2) { // HOVER

	} else if (state_ == 3) { // TURN_TO_FIND_WALL
		// check if wall is found
		bool side_range_check =
		    side_range < ref_distance_from_wall_ / std::cos(0.78F) + 0.2F;
		bool front_range_check =
		    front_range < ref_distance_from_wall_ / std::cos(0.78F) + 0.2F;
		if (side_range_check && front_range_check) {
			previous_heading = current_heading;
			angle = direction_ *
			        (1.57F - std::atan(front_range / side_range) + 0.1F);
			state_ = transition(
			    4, &state_start_time_); // go to turn_to_allign_to_wall
		}
		if (side_range < 1.0F && front_range > 2.0F) {
			//  around_corner_first_turn = true;
			around_corner_go_back = false;
			previous_heading = current_heading;
			state_ =
			    transition(8, &state_start_time_); // go to rotate_around_wall
		}
	} else if (state_ == 4) { // TURN_TO_ALLIGN_TO_WALL
		bool allign_wall_check = logic_is_close_to(
		    wrap_to_pi(current_heading - previous_heading), angle, 0.1F);
		if (allign_wall_check) {
			// prev_side_range = side_range;
			state_ = transition(5, &state_start_time_);
		}
	} else if (state_ == 5) { // FORWARD_ALONG_WALL

		// If side range is out of reach,
		//    end of the wall is reached
		if (side_range > ref_distance_from_wall_ + 0.3F) {
			//  around_corner_first_turn = true;
			state_ = transition(8, &state_start_time_);
		}
		// If front range is small
		//    then corner is reached
		if (front_range < ref_distance_from_wall_ + 0.2F) {
			previous_heading = current_heading;
			state_ = transition(7, &state_start_time_);
		}

	} else if (state_ == 6) { // ROTATE_AROUND_WALL
		if (front_range < ref_distance_from_wall_ + 0.2F) {
			state_ = transition(3, &state_start_time_);
		}

	} else if (state_ == 7) { // ROTATE_IN_CORNER
		// Check if heading goes over 0.8 rad
		bool check_heading_corner = logic_is_close_to(
		    std::fabs(wrap_to_pi(current_heading - previous_heading)), 0.8F,
		    0.1F);
		if (check_heading_corner) {
			state_ = transition(3, &state_start_time_);
		}

	} else if (state_ == 8) { // FIND_CORNER
		if (side_range <= ref_distance_from_wall_) {
			state_ = transition(6, &state_start_time_);
		}
	}

	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x = 0;
	float temp_vel_y = 0;
	float temp_vel_w = 0;

	if (state_ == 1) { // FORWARD
		temp_vel_x = max_speed_;
		temp_vel_y = 0.0;
		temp_vel_w = 0.0;

	} else if (state_ == 2) { // HOVER
		command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);

	} else if (state_ == 3) { // TURN_TO_FIND_WALL
		command_turn(&temp_vel_x, &temp_vel_w, max_rate_, direction_);
		temp_vel_y = 0.0;

	} else if (state_ == 4) { // TURN_TO_ALLIGN_TO_WALL

		if (now - state_start_time_ < 1.0F) {
			command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
		} else { // then turn again
			command_turn(&temp_vel_x, &temp_vel_w, max_rate_, direction_);
			temp_vel_y = 0;
		}

	} else if (state_ == 5) { // FORWARD_ALONG_WALL
		command_forward_along_wall(&temp_vel_x, &temp_vel_y, side_range,
		                           direction_, max_speed_,
		                           ref_distance_from_wall_);
		temp_vel_w = 0.0F;

		// command_forward_along_wallHeadingSine(&temp_vel_x,
		// &temp_vel_y,&temp_vel_w, side_range);

	} else if (state_ == 6) { // ROTATE_AROUND_WALL
		// If first time around corner
		// first try to find the corner again

		// if side range is larger than prefered distance from wall
		if (side_range > ref_distance_from_wall_ + 0.5F) {

			// check if scanning has already occured
			if (wrap_to_pi(std::fabs(current_heading - previous_heading)) >
			    0.8F) {
				around_corner_go_back = true;
			}
			// turn and adjust distnace to corner from that point
			if (around_corner_go_back) {
				// go back if it already went into one direction
				command_turn_and_adjust(&temp_vel_y, &temp_vel_w,
				                        -1 * max_rate_, side_range, direction_);
				temp_vel_x = 0.0F;
			} else {
				command_turn_and_adjust(&temp_vel_y, &temp_vel_w, max_rate_,
				                        side_range, direction_);
				temp_vel_x = 0.0F;
			}
		} else {
			// continue to turn around corner
			previous_heading = current_heading;
			around_corner_go_back = false;
			command_turn_around_corner_and_adjust(
			    &temp_vel_x, &temp_vel_y, &temp_vel_w, ref_distance_from_wall_,
			    side_range, direction_, max_speed_, ref_distance_from_wall_);
		}

	} else if (state_ == 7) { // ROTATE_IN_CORNER
		command_turn(&temp_vel_x, &temp_vel_w, max_rate_, direction_);
		temp_vel_y = 0;

	} else if (state_ == 8) { // FIND_CORNER
		command_align_corner(&temp_vel_y, &temp_vel_w, -1 * max_rate_,
		                     side_range, ref_distance_from_wall_, direction_,
		                     max_speed_);
		temp_vel_x = 0;
	} else {
		// State does not exist so hover!!
		command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
	}

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;

	return state_;
}
