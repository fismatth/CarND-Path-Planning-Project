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
	int idx = 0;

	for (int i = -NUM_SPLINES_PER_SIDE; i <= NUM_SPLINES_PER_SIDE; ++i, ++idx)
	{
		vector<double> s {0.0, 0.25, 0.5, 0.75, 1.0};
		// at most change 2 lanes with one spline
		double d_diff_total = i * 2.0 * LANE_WIDTH / NUM_SPLINES_PER_SIDE;
		vector<double> d {0.0, 0.15 * d_diff_total, 0.5 * d_diff_total, 0.85 * d_diff_total, d_diff_total};
//		_basic_splines[idx].set_boundary(tk::spline::first_deriv, 0.0, tk::spline::first_deriv, i * 0.05);
		_basic_splines[idx].set_points(s, d);
//		cout << "Set right derivative to " << i * 0.5 << endl;
		cout << "Set points for idx = " << idx << ": " << endl;
		cout << s << d << endl;
	}

	for (int i = -NUM_SPLINES_PER_SIDE; i <= NUM_SPLINES_PER_SIDE; ++i, ++idx)
	{
		vector<double> s {0.0, 0.5, 1.0};
		double d_diff_total = i * 10.0 / NUM_SPLINES_PER_SIDE;
		vector<double> d {0.0, 0.3 * d_diff_total, d_diff_total};
		_basic_splines[idx].set_points(s, d);
		cout << "Set points for idx = " << idx << ": " << endl;
		cout << s << d << endl;
	}
//	double t = 0.0;
//
//	while (t < 1.5)
//	{
//		cout << "f(" << t << ") = " << _basic_splines[3](t) << endl;
//		t += dt;
//	}
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
	}

	differentiate(dt, traj_info.v, traj_info.a);
	differentiate(dt, traj_info.a, traj_info.jerk);



//	if (verbose)
//	{
//		cout << "Traj. s, d, v, a, jerk" << endl;
//		cout << traj_info.s_values << traj_info.d_values << traj_info.v << traj_info.a << traj_info.jerk << endl;
//	}

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
//		cout << traj_info.v << traj_info.a << traj_info.jerk << endl;
//		cout << traj_info.d_values << endl;
		cout << "Cost = " << cost << endl;
	}

	return cost;
}

void PathPlanner::generate_trajectories(Trajectory current, Car car_state, Trajectory& best, double& best_value, const Car& initial_state, const vector<Vehicle>& others)
{
	int best_spline = -1;
	int best_acceleration = -10;

	for (int spline_idx = 0; spline_idx < NUM_SPLINES; ++spline_idx)
	{
		for (int acceleration = 2; acceleration >= -2; --acceleration)
		{
			Trajectory new_trajectory(current);
			Car new_car_state(car_state);

			double max_time_interval = 1.5;
			double t = dt;
			double a = 2.0;//max(1e-5, new_car_state.speed / 50.0);

			new_car_state.speed += acceleration * a * dt;

			if (new_car_state.speed <= 0)
			{
				continue;
			}

			double x_offset = car_state.x;
			double y_offset = car_state.y;

			if (current.x_values.size() > 0)
			{
				size_t last_idx = current.x_values.size() - 1;
				double x_offset = current.x_values[last_idx];
				double y_offset = current.y_values[last_idx];
			}

			double x_local = 0.0;
			double y_local = 0.0;
			double yaw  = car_state.yaw;

			bool invalid_trajectory = false;
			//double x_max = 50.0;//min(50.0, max(30.0, max_time_interval * new_car_state.speed));
			double x_max = new_car_state.speed * max_time_interval + 0.5 * pow(max_time_interval, 2) * a * acceleration;

			while (t <= max_time_interval)//(x_local <= x_max && )
			{
				// ...
				x_local += dt * new_car_state.speed;
				y_local = _basic_splines[spline_idx](x_local / x_max);

//				double x_prev = x_local;
//				double y_prev = y_local;
//				y_local = _basic_splines[spline_idx](t / max_time_interval);
//				double tmp = pow(new_car_state.speed * dt, 2) - pow(y_local - y_prev, 2);
//				if (tmp < 0.0)
//				{
//					// change in y requires higher speed
//					invalid_trajectory = true;
//					break;
//				}
//				x_local = x_prev + sqrt(tmp);

				new_car_state.x = x_local * cos(yaw) - y_local * sin(yaw) + x_offset;
				new_car_state.y = x_local * sin(yaw) + y_local * cos(yaw) + y_offset;

				// add waypoint to trajectory
				new_trajectory.x_values.push_back(new_car_state.x);
				new_trajectory.y_values.push_back(new_car_state.y);

				new_car_state.speed += acceleration * a * dt;
				t += dt;
			}

			if (invalid_trajectory)
			{
				continue;
			}

//			cout << "Spline " << spline_idx << ":" << endl;
			double cost = compute_cost(new_trajectory, initial_state, others, false);

			if (cost < best_value)
			{
				best = new_trajectory;
				best_value = cost;

				best_spline = spline_idx;
				best_acceleration = acceleration;
			}
		}
	}

	cout << "Best spline " << best_spline << " using acceleration = " << best_acceleration <<  " having cost value " << best_value << endl;
}

Trajectory PathPlanner::plan(const Trajectory& previous, const Car& car,
		const vector<Vehicle>& other)
{
//	cout << "Received: " << previous << endl;

	int start_idx = -1;
	double min_diff = numeric_limits<double>::infinity();

	// use _current_trajectory instead of previous due to loss of accuracy when sending & receiving
	for (int i = 0; i < _current_trajectory.x_values.size(); ++i)
	{
		//if (fabs(previous.x_values[0] - _current_trajectory.x_values[i]) < 1e-8 && fabs(previous.y_values[0] - _current_trajectory.y_values[i]) < 1e-8)
		double dist = distance(previous.x_values[0], previous.y_values[0], _current_trajectory.x_values[i], _current_trajectory.y_values[i]);
		if (dist < min_diff)
		{
			start_idx = i;
			min_diff = dist;

			//break;
		}
	}

	if (start_idx == -1)
	{
		cout << "Could not find start idx" << endl;
		start_idx = 0;
//		char c;
//		cin >> c;
	}
//	else
//	{
//		cout << "Found start idx " << start_idx << " with diff = " << min_diff << endl;
//		cout << previous.x_values[0] << " vs. " << _current_trajectory.x_values[start_idx] << endl;
//		cout << previous.y_values[0] << " vs. " << _current_trajectory.y_values[start_idx] << endl;
//	}

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
	// estimated state of the car after following first 20 points from previous trajectory
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

//		if (start_trajectory.x_values.size() >= 3)
//		{
//			double x_t_2 = start_trajectory.x_values[last_idx - 2];
//			double y_t_2 = start_trajectory.y_values[last_idx - 2];
//			double v_t_1 = distance(x_t_1, y_t_1, x_t_2, y_t_2) / dt;
//
//			start_state.a = (v_t - v_t_1) / dt;
//		}
	}

	// generate possible trajectories and choose best one (min. cost value)
	double best_value = numeric_limits<double>::infinity();

	generate_trajectories(start_trajectory, start_state, _current_trajectory, best_value, car, other);

	compute_cost(_current_trajectory, car, other, true);

//	cout << "Send: " << _current_trajectory;
//	cout << _current_trajectory.x_values.size() << endl;
//	char c;
//	cin >> c;

	return _current_trajectory;
}

void PathPlanner::register_cost_function(CostFunctionType func, double weight)
{
	_cost_functions.push_back(func);
	_weights.push_back(weight);
}
