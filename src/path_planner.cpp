#include "path_planner.h"

#include <algorithm>
#include <math.h>
#include <limits>
#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"


ostream& operator<<(ostream& os, const Car& car)
{
	return os << "Car: (x, y, s, d, yaw, speed, a, delta) = " << car.x << ", " << car.y << ", " << car.s << ", " << car.d << ", " << car.yaw << ", " << car.speed;// << ", " << car.a << ", " << car.delta;
}

ostream& operator<<(ostream& os, const Trajectory& traj)
{
	return os << "Trajectory (x, y):" << endl << traj.x_values << traj.y_values <<endl;
}


PathPlanner::PathPlanner(const MapData& waypoints) :
		_map_data(waypoints)
{
}


double PathPlanner::compute_cost(const Trajectory& trajectory, const Car& initial_state, const vector<Vehicle>& others, bool verbose)
{

	TrajectoryInformation traj_info;
	traj_info.x_values = trajectory.x_values;
	traj_info.y_values = trajectory.y_values;

	for (int i = 0; i < trajectory.x_values.size(); ++i)
	{
		vector<double> frenet_coords = getFrenet(trajectory.x_values[i], trajectory.y_values[i], initial_state.yaw, _map_data.map_waypoints_x, _map_data.map_waypoints_y);
		traj_info.s_values.push_back(frenet_coords[0]);
		traj_info.d_values.push_back(frenet_coords[1]);

		if (i > 0)
		{
			double x_i = trajectory.x_values[i];
			double y_i = trajectory.y_values[i];
			double x_i_1 = trajectory.x_values[i-1];
			double y_i_1 = trajectory.y_values[i-1];

			double v_i = distance(x_i, y_i, x_i_1, y_i_1) / dt;
			traj_info.v.push_back(v_i);
		}

		auto local_xy = compute_local_coords(initial_state.yaw, initial_state.x, initial_state.y, trajectory.x_values[i], trajectory.y_values[i]);
		traj_info.x_local.push_back(local_xy.first);
		traj_info.y_local.push_back(local_xy.second);
	}

	differentiate(dt, traj_info.v, traj_info.a);
	differentiate(dt, traj_info.a, traj_info.jerk);

	differentiate(dt, traj_info.x_local, traj_info.v_x);
	differentiate(dt, traj_info.v_x, traj_info.a_x);
	differentiate(dt, traj_info.y_local, traj_info.v_y);
	differentiate(dt, traj_info.v_y, traj_info.a_y);

	double cost = 0.0;

	for (int i = 0; i < _cost_functions.size(); ++i)
	{
		if (verbose)
		{
			cout << "Cost fct. " << i << " gives " << _cost_functions[i](traj_info, initial_state, others) << endl;
		}

		cost += _weights[i] * _cost_functions[i](traj_info, initial_state, others);
	}

	if (verbose)
	{
		cout << "Cost = " << cost << endl;
	}

	return cost;
}

pair<Eigen::VectorXd, Eigen::VectorXd> PathPlanner::compute_local_trajectory(Car& car_state, double d_goal, double acceleration)
{
	vector<double> start_x(2);
	vector<double> end_x(2);
	vector<double> start_y(2);
	vector<double> end_y(2);

	start_x[0] = 0.0;
	start_x[1] = car_state.x_local_dot;
//	start_x[2] = car_state.x_local_dot_dot;

	start_y[0] = 0.0;
	start_y[1] = car_state.y_local_dot;
//	start_y[2] = car_state.y_local_dot_dot;

	pair<double, double> xy_global_start = compute_global_coords(car_state.yaw, car_state.x, car_state.y, start_x[0], start_y[0]);
	auto sd_start = getFrenet(xy_global_start.first, xy_global_start.second, car_state.yaw, _map_data.map_waypoints_x, _map_data.map_waypoints_y);
	double total_distance = (car_state.speed + 0.5 * acceleration) * T;
	double d_distance = fabs(car_state.d - d_goal);
	double s_distance = sqrt(pow(total_distance, 2) - pow(d_distance, 2));
	double s_end = sd_start[0] + s_distance;
	double d_end = d_goal;
	auto xy_global_end = getXY(s_end, d_end, _map_data.map_waypoints_s, _map_data.map_waypoints_x, _map_data.map_waypoints_y);
	pair<double, double> xy_local_end = compute_local_coords(car_state.yaw, car_state.x, car_state.y, xy_global_end[0], xy_global_end[1]);

	//pair<double, double> xy_global_end = compute_global_coords(car_state.yaw, car_state.x, car_state.y, end_x[0], end_y[0]);
	//auto sd_end = getFrenet(xy_global_end[0], xy_global_end[1], car_state.yaw, _map_data.map_waypoints_x, _map_data.map_waypoints_y);
	// use reference point near end point to get the yaw of the road
	double s_ref = s_end + 0.5;
	double d_ref = d_end;
	auto xy_global_ref = getXY(s_ref, d_ref, _map_data.map_waypoints_s, _map_data.map_waypoints_x, _map_data.map_waypoints_y);
	Eigen::Vector2d e_x(1.0, 0.0);
	Eigen::Vector2d v_road_end(xy_global_ref[0] - xy_global_end[0], xy_global_ref[1] - xy_global_end[1]);
	double yaw_road_end = yaw(e_x, v_road_end);
	double yaw_local = yaw_road_end - car_state.yaw;
//	double y_dot = sin(yaw_local) / cos(yaw_local);

	// speed at end of planned trajectory
	car_state.speed += acceleration;

	double x_dot = cos(yaw_local) * car_state.speed;
	double y_dot = sin(yaw_local) * car_state.speed;
	double x_dot_dot = cos(yaw_local) * acceleration;
	double y_dot_dot = sin(yaw_local) * acceleration;

	end_x[0] = xy_local_end.first;
	end_x[1] = x_dot;
//	end_x[2] = x_dot_dot;

	end_y[0] = xy_local_end.second;
	end_y[1] = y_dot;
//	end_y[2] = y_dot_dot;

	Eigen::VectorXd x_coeffs = PolyTrajectoryConstantAcceleration(start_x, end_x, T);
	Eigen::VectorXd y_coeffs = PolyTrajectoryPosVelConditions(start_y, end_y, T);

	return make_pair(x_coeffs, y_coeffs);
}

void PathPlanner::generate_trajectories(Trajectory current, Car car_state, Trajectory& best, double& best_value, const Car& initial_state, const vector<Vehicle>& others)
{
	double y_diff_max = LANE_WIDTH;
	int num_d_steps = 5;
	double d_step = 0.025;

	int best_lane = -1;
	double best_d_goal = -1;
	int best_acc_factor = -10;
	double best_acc = 0.0;

	for (int lane = 0; lane < NUM_LANES; ++lane)
	{
		double d_center = (lane + 0.5) * LANE_WIDTH;

		for (int d_factor =-num_d_steps; d_factor <= num_d_steps; ++d_factor)
		{
			double d_goal = d_center + d_factor * d_step;

			for (int acc_factor = 3; acc_factor >= -3; --acc_factor)
			{
				Trajectory new_trajectory(current);
				Car new_car_state(car_state);

				double t = dt;
				double acceleration = acc_factor * 1.5;

				if (5.0 * fabs(d_goal - car_state.d) > (car_state.speed + 0.5 * acceleration) * T)
				{
					// exclude trajectories with high change in d direction relative to change in s direction
					continue;
				}

				pair<Eigen::VectorXd, Eigen::VectorXd> xy_coeffs = compute_local_trajectory(new_car_state, d_goal, acceleration);

				while (t <=  T )
				{
					// compute local way point
					double x_local = eval_polynomial(xy_coeffs.first, t);
					double y_local = eval_polynomial(xy_coeffs.second, t);

					// transform to global coordinates
					pair<double, double> xy_global = compute_global_coords(car_state.yaw, car_state.x, car_state.y, x_local, y_local);

					// add waypoint to trajectory
					new_trajectory.x_values.push_back(xy_global.first);
					new_trajectory.y_values.push_back(xy_global.second);

					t += dt;
				}

				double cost = compute_cost(new_trajectory, initial_state, others, false);

				if (lane == _desired_lane)
				{
					// reduce cost if trajectory follows last plan (keep lane or change to specific lane)
					cost *= 0.5;
				}

				if (cost < best_value)
				{
					best = new_trajectory;
					best_value = cost;

					best_lane = lane;
					best_d_goal = d_goal;
					best_acc_factor = acc_factor;
					best_acc = acceleration;
				}
			}
		}
	}

	_desired_lane = best_lane;

//	cout << "Best lane, d_goal, acc_factor, acceleration: " << best_lane << ", " << best_d_goal << ", " << best_acc_factor << ", " << best_acc << endl;
}

Trajectory PathPlanner::plan(const Trajectory& previous, const Car& car,
		const vector<Vehicle>& other)
{
	int start_idx = -1;
	double min_diff = numeric_limits<double>::infinity();

	// use _current_trajectory instead of previous due to loss of accuracy when sending & receiving
	if (previous.x_values.size() > 0)
	{
		for (int i = 0; i < _current_trajectory.x_values.size(); ++i)
		{
			double dist = distance(previous.x_values[0], previous.y_values[0], _current_trajectory.x_values[i], _current_trajectory.y_values[i]);

			if (dist < min_diff)
			{
				start_idx = i;
				min_diff = dist;
			}
		}
	}

	if (start_idx == -1)
	{
		cout << "Could not find start idx" << endl;
		start_idx = 0;
	}

	Trajectory start_trajectory;

	// copy up to _num_copied points from previous trajectory for stability
	for (int i = 0; i < std::min(_num_copied, _current_trajectory.x_values.size() - start_idx); ++i)
	{
		start_trajectory.x_values.push_back(_current_trajectory.x_values[start_idx + i]);
		start_trajectory.y_values.push_back(_current_trajectory.y_values[start_idx + i]);
	}

	_current_trajectory.x_values.clear();
	_current_trajectory.y_values.clear();

	int last_idx = start_trajectory.x_values.size() - 1;
	// estimated state of the car after following first _num_copied points from previous trajectory
	Car start_state(car);

	if (start_trajectory.x_values.size() >= 1)
	{
		start_state.x = start_trajectory.x_values[last_idx];
		start_state.y = start_trajectory.y_values[last_idx];
	}

	if (start_trajectory.x_values.size() >= 2)
	{
		double x_t = start_trajectory.x_values[last_idx];
		double x_t_1 = start_trajectory.x_values[last_idx - 1];
		double y_t = start_trajectory.y_values[last_idx];
		double y_t_1 = start_trajectory.y_values[last_idx - 1];

		double v_t = distance(x_t, y_t, x_t_1, y_t_1) / dt;
		start_state.speed = v_t;
		Eigen::Vector2d e_x(1.0, 0.0);
		Eigen::Vector2d driving_direction(x_t - x_t_1, y_t - y_t_1);
		start_state.yaw = yaw(e_x, driving_direction);

		if (start_trajectory.x_values.size() >= 3)
		{
			double x_t_2 = start_trajectory.x_values[last_idx - 2];
			double y_t_2 = start_trajectory.y_values[last_idx - 2];
			double v_t_1 = distance(x_t_1, y_t_1, x_t_2, y_t_2) / dt;

			start_state.a = (v_t - v_t_1) / dt;

			Eigen::Vector2d prev_driving_direction(x_t_1 - x_t_2, y_t_1 - y_t_2);
			double yaw_t_1 = yaw(e_x, prev_driving_direction);
			start_state.yaw_dot = (start_state.yaw - yaw_t_1) / dt;

			vector<double> x_local(start_trajectory.x_values.size());
			vector<double> y_local(start_trajectory.y_values.size());

			for (int i = 0; i < start_trajectory.x_values.size(); ++i)
			{
				pair<double, double> xy_local = compute_local_coords(start_state.yaw, start_state.x, start_state.y, start_trajectory.x_values[i], start_trajectory.y_values[i]);
				x_local[i] = xy_local.first;
				y_local[i] = xy_local.second;
			}

			vector<double> x_local_dot, y_local_dot;//, y_local_dot_dot, x_local_dot_dot
			differentiate(dt, x_local, x_local_dot);
//			differentiate(dt, x_local_dot, x_local_dot_dot);
			differentiate(dt, y_local, y_local_dot);
//			differentiate(dt, y_local_dot, y_local_dot_dot);

			start_state.x_local_dot = x_local_dot.back();
//			start_state.x_local_dot_dot = x_local_dot_dot.back();
			start_state.y_local_dot = y_local_dot.back();
//			start_state.y_local_dot_dot = y_local_dot_dot.back();
		}
	}

	// generate possible trajectories and choose best one (min. cost value)
	double best_value = numeric_limits<double>::infinity();

	generate_trajectories(start_trajectory, start_state, _current_trajectory, best_value, car, other);

	// uncomment to see the values for each cost function for the chosen trajectory
//	compute_cost(_current_trajectory, car, other, true);

//	cout << "Send: " << _current_trajectory;
//	cout << _current_trajectory.x_values.size() << endl;

	return _current_trajectory;
}

void PathPlanner::register_cost_function(CostFunctionType func, double weight)
{
	_cost_functions.push_back(func);
	_weights.push_back(weight);
}
