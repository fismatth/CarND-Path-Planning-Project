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

	for (auto a_i: traj.jerk)
	{
		if (a_i > 0.95 * MAX_JERK)
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
		if (d_i < 0.0 || d_i > NUM_LANES * LANE_WIDTH)
		{
			return 1.0;
		}
	}

	return 0.0;
}

double collision_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
//	double num_collisions = 0.0;

	for (auto veh : other)
	{
		// under-estimate of speed in s direction of other vehicle
		double veh_v_s = 0.9 * sqrt(pow(veh.vx, 2) + pow(veh.vy, 2));

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
				double s_dist = fabs((veh.s + veh_v_s * dt * i) - traj.s_values[i]);

				if (s_dist <= 2.0 * VEHICLE_RADIUS)
				{
					return 1.0;
					//num_collisions += 1.0;
				}
			}

//			double veh_x = veh.x + i * dt * veh.vx;
//			double veh_y = veh.y + i * dt * veh.vy;
//
//			double dist = distance(veh_x, veh_y, traj.x_values[i], traj.y_values[i]);
//
//			if (dist <= 2.0 * VEHICLE_RADIUS)
//			{
//				num_collisions += 1.0;
//			}
		}

	}

	return 0.0;
//	return num_collisions;
}

double other_veh_gap_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other)
{
	double buffer_cost = 0.0;

	for (auto veh : other)
	{
		for (int i = 0; i < traj.x_values.size(); ++i)
		{
			double veh_x = veh.x;// + i * dt * veh.vx;
			double veh_y = veh.y;// + i * dt * veh.vy;

			double buffer = distance(veh_x, veh_y, traj.x_values[i], traj.y_values[i]);

			if (buffer < 3.0 * VEHICLE_RADIUS)
			{
				buffer_cost += 3.0 * VEHICLE_RADIUS - buffer;
			}
		}
	}

	return logistic(buffer_cost);
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

//	cout << "d_values = " << endl;
//	cout << traj.d_values;

	for (auto d_i : traj.d_values)
	{
		int lane = d_i / LANE_WIDTH;
		double center = lane * LANE_WIDTH + 0.5 * LANE_WIDTH;
		total_diff_from_center += fabs(d_i - center);
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
			return 0.1;
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
