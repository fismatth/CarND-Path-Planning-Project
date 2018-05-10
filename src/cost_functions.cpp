#include "cost_functions.h"

#include <algorithm>
#include <math.h>
#include "helpers.h"
#include <iostream>



double logistic(double x)
{
	return 2.0 / (1 + exp(-x)) - 1.0;
}

double standing_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	size_t last = traj.s_values.size() - 1;
	double s_begin = traj.s_values[0];
	double s_end = traj.s_values[1];
	return fabs(s_end - s_begin) < 1e-3 ? 1.0 : 0.0;
}

double speed_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double avg_v = 0.0;

	for (auto v_i : traj.v)
	{
		avg_v += v_i;
	}

	avg_v /= traj.v.size();

	return avg_v > 1e-4 ? logistic(fabs(TARGET_SPEED - avg_v) / TARGET_SPEED) : 1.0;
}

double efficiency_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double s_dist = traj.s_values.back() - traj.s_values.front();
	if (s_dist < 0.0)
	{
		s_dist += max_s;
	}

	return 1.0 / s_dist;
}

double exceeds_max_speed(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	for (auto v_i : traj.v)
	{
		if (v_i > MAX_SPEED)
		{
			return 1.0;
		}
	}

	return 0.0;
}

double max_acceleration_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double max_exceeded = 0.0;

	for (auto a_i: traj.a)
	{
		if (a_i > 0.9 * MAX_ACCELERATION)
		{
			max_exceeded += 1.0;
		}
	}

	return max_exceeded;
	//return logistic(*std::max_element(traj.a.begin(), traj.a.end())); // > 0.95 * MAX_ACCELERATION ? 1.0 : 0.0;
}

double total_acceleration_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double total_a_above_expected = 0.0;

	for (auto a_i : traj.a)
	{
		if (fabs(a_i) > EXPECTED_ACCELERATION)
		{
			total_a_above_expected += fabs(a_i) - EXPECTED_ACCELERATION;
		}
	}

	return logistic(total_a_above_expected / traj.a.size());
}

double max_jerk_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double max_exceeded = 0.0;

	for (auto j_i: traj.jerk)
	{
		if (j_i > 0.95 * MAX_JERK)
		{
			max_exceeded += 1.0;
		}
	}

	return max_exceeded;

	//return logistic(*std::max_element(traj.jerk.begin(), traj.jerk.end()));// > 0.95 * MAX_JERK ? 1.0 : 0.0;
}

double total_jerk_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double total = 0.0;

	for (auto jerk_i : traj.jerk)
	{
		total += fabs(jerk_i);
	}

	return logistic(total / traj.jerk.size());
}

double off_road_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	for (auto d_i : traj.d_values)
	{
		if (d_i - 0.5 * LANE_WIDTH < 0.0 || d_i + 0.5 * LANE_WIDTH > NUM_LANES * LANE_WIDTH)
		{
			return 1.0;
		}
	}

	return 0.0;
}

double collision_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double num_collisions = 0.0;

	for (auto veh : other)
	{
		// under-estimate of speed in s direction used for vehicles in front
		double veh_v_s_low = 0.9 * sqrt(pow(veh.vx, 2) + pow(veh.vy, 2));
		// over-estimate of speed in s direction used for vehicles from behind
		double veh_v_s_high = 1.1 * sqrt(pow(veh.vx, 2) + pow(veh.vy, 2));

		for (int i = 0; i < traj.d_values.size(); ++i)
		{
			// normally, we should have left_lane == right_lane, but when changing lanes (or accidentally driving over lane boundaries), car could be on two lanes
			int left_lane = (traj.d_values[i] - 0.25 * LANE_WIDTH) / LANE_WIDTH;
			int right_lane = (traj.d_values[i] + 0.25 * LANE_WIDTH) / LANE_WIDTH;
			int veh_lane_left = (veh.d - 0.25 * LANE_WIDTH) / LANE_WIDTH;
			int veh_lane_right = (veh.d + 0.25 * LANE_WIDTH) / LANE_WIDTH;

			if (left_lane == veh_lane_left || right_lane == veh_lane_left || right_lane == veh_lane_right || left_lane == veh_lane_right)
			{
				//
				double s_car_i = traj.s_values[i];
				double s_dist_low = fabs((veh.s + veh_v_s_low * dt * i) - s_car_i);
				double s_dist_high = fabs((veh.s + veh_v_s_high * dt * i) - s_car_i);

				if (s_dist_low <= 2.0 * VEHICLE_RADIUS || s_dist_high <= 2.0 * VEHICLE_RADIUS || (veh.s + veh_v_s_low * dt * i < s_car_i && s_car_i < veh.s + veh_v_s_high * dt * i))
				{
					num_collisions += 1.0;
				}
			}

		}

	}

	return num_collisions;
//	return 0.0;
}

double other_veh_gap_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double num_low_buffer = 0.0;

	for (auto veh : other)
	{
		// under-estimate of speed in s direction used for vehicles in front
		double veh_v_s_low = 0.9 * sqrt(pow(veh.vx, 2) + pow(veh.vy, 2));
		// over-estimate of speed in s direction used for vehicles from behind
		double veh_v_s_high = 1.1 * sqrt(pow(veh.vx, 2) + pow(veh.vy, 2));

		for (int i = 0; i < traj.d_values.size(); ++i)
		{
			// normally, we should have left_lane == right_lane, but when changing lanes (or accidentally driving over lane boundaries), car could be on two lanes
			int left_lane = (traj.d_values[i] - 0.25 * LANE_WIDTH) / LANE_WIDTH;
			int right_lane = (traj.d_values[i] + 0.25 * LANE_WIDTH) / LANE_WIDTH;
			int veh_lane_left = (veh.d - 0.25 * LANE_WIDTH) / LANE_WIDTH;
			int veh_lane_right = (veh.d + 0.25 * LANE_WIDTH) / LANE_WIDTH;

			if (left_lane == veh_lane_left || right_lane == veh_lane_left || right_lane == veh_lane_right || left_lane == veh_lane_right)
			{
				double s_car_i = traj.s_values[i];
				double s_dist_low = fabs((veh.s + veh_v_s_low * dt * i) - s_car_i);
				double s_dist_high = fabs((veh.s + veh_v_s_high * dt * i) - s_car_i);

				if (s_dist_low <= 10.0 * VEHICLE_RADIUS || s_dist_high <= 10.0 * VEHICLE_RADIUS || (veh.s + veh_v_s_low * dt * i < s_car_i && s_car_i < veh.s + veh_v_s_high * dt * i))
				{
					num_low_buffer += 1.0;
				}
			}
		}
	}

	return num_low_buffer;
}

double max_dist_from_center_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double max_dist = 0.0;

	for (auto d_i : traj.d_values)
	{
		int lane = d_i / LANE_WIDTH;
		double center = lane * LANE_WIDTH + 0.5 * LANE_WIDTH;
		double center_dist = fabs(center - d_i);

		if (center_dist > max_dist)
		{
			max_dist = center_dist;
		}
	}

	return logistic(max_dist);
}

double keep_lane_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double total_diff_from_center = 0.0;
	int start_lane = car.d / LANE_WIDTH;
	double start_center = start_lane * LANE_WIDTH + 0.5 * LANE_WIDTH;
	int end_lane = traj.d_values.back() / LANE_WIDTH;
	double end_center = end_lane * LANE_WIDTH + 0.5 * LANE_WIDTH;
//	cout << "d_values = " << endl;
//	cout << traj.d_values;

	for (auto d_i : traj.d_values)
	{
		total_diff_from_center += min(fabs(d_i - start_center), fabs(d_i - end_center));
//		cout << " += " << d_i << " - " << center << " == " << d_i - center << " -> " << total_diff_from_center << endl;
	}

//	if ( logistic(total_diff_from_center) != logistic(total_diff_from_center))
//	{
//		cout << "NaN for logistic(" << total_diff_from_center << ")" << endl;
//		return 0.0;
//	}
	return logistic(total_diff_from_center / traj.d_values.size());
}

double right_direction_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	// if s is decreasing more than once, we are going in the wrong direction (once is possible when going from last to first waypoint)
	int count_s_decreasing = 0;

	for (int i = 0; i < traj.s_values.size() - 1; ++i)
	{
		if (traj.s_values[i] > traj.s_values[i+1])
		{
			++count_s_decreasing;
		}
	}

	return count_s_decreasing > 1 ? 1.0 : 0.0;
}

double min_d_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double total = 0.0;

	for (int i = 1; i < traj.d_values.size(); ++i)
	{
		total += fabs((traj.d_values[i] - traj.d_values[i-1]) / (traj.s_values[i] - traj.s_values[i-1]));
	}

	return logistic(total);
}

double min_lane_changes_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	int lane_changes = 0;

	for (int i = 1; i < traj.d_values.size(); ++i)
	{
		int prev_lane = traj.d_values[i - 1] / LANE_WIDTH;
		int lane = traj.d_values[i] / LANE_WIDTH;

		if (prev_lane != lane)
		{
			++lane_changes;
		}
	}

	switch (lane_changes) {
		case 1:
			// one lane change is ok
			return 0.3;
			break;
		case 2:
			// two lane changes at a time only if really needed
			return 1.0;
			break;
		default:
			// more than 2 lane changes at a time is crazy
			return (double)lane_changes;
			break;
	}
	return lane_changes;
}

double min_total_yaw_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double yaw_0 = yaw_diff(car.x - cos(car.yaw), car.y - sin(car.yaw), car.x, car.y, traj.x_values[0], traj.y_values[0]);
//	cout << cos(car.yaw) << endl;
//	cout << car.x - cos(car.yaw) << ", " <<  car.y - sin(car.yaw) << ", " << car.x << ", " << car.y << ", " << traj.x_values[0] << ", " << traj.y_values[0] << endl;
	double yaw_total = yaw_0;
//	cout << "yaw_0 = " << yaw_0 << endl;

	for (int i = 2; i < traj.x_values.size(); ++i)
	{
		yaw_total += yaw_diff(traj.x_values[i - 2], traj.y_values[i - 2], traj.x_values[i - 1], traj.y_values[i - 1], traj.x_values[i], traj.y_values[i]);
	}

	return logistic(yaw_total);
}

double good_goal_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	int last_idx = traj.d_values.size() - 1;
	double d_last = traj.d_values[last_idx];
	double d_pre_last = traj.d_values[last_idx - 1];
	int lane = d_last / LANE_WIDTH;
	double diff_from_center = fabs((lane + 0.5) * LANE_WIDTH - d_last);

	if (diff_from_center > 0.1 * LANE_WIDTH)
	{
		return 1.0;
	}

	// TODO what is a good threshold?
//	if (fabs(d_last - d_pre_last) > 0.02)
//	{
//		return 1.0;
//	}

	return 0.0;
}

double min_d_range_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double max_d = *std::max_element(traj.d_values.begin(), traj.d_values.end());
	double min_d = *std::min_element(traj.d_values.begin(), traj.d_values.end());
	double max_s = *std::max_element(traj.s_values.begin(), traj.s_values.end());
	double min_s = *std::min_element(traj.s_values.begin(), traj.s_values.end());

	return (max_d - min_d) / (max_s - min_s);
}

