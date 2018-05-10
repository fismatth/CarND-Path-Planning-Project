#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <functional>
#include <string>
#include <iostream>
#include "spline.h"
#include "helpers.h"


using namespace std;

struct Trajectory
{
	vector<double> x_values;
	vector<double> y_values;
};

struct TrajectoryInformation
{
	vector<double> x_values;
	vector<double> y_values;
	vector<double> s_values;
	vector<double> d_values;
	vector<double> v;
	vector<double> a;
	vector<double> jerk;

	vector<double> x_local;
	vector<double> y_local;
	vector<double> v_x;
	vector<double> v_y;
	vector<double> a_x;
	vector<double> a_y;
};

struct MapData
{
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
};

struct Car
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	// we don't get this from the simulator -> use default values
	double a = 0.0;
	double yaw_dot = 0.0;
	double x_local_dot = 0.0;
	double x_local_dot_dot = 0.0;
	double y_local_dot = 0.0;
	double y_local_dot_dot = 0.0;
};

ostream& operator<<(ostream& os, const Car& car);

class PathPlanner
{
public:

	typedef function<
			double(const TrajectoryInformation&, const Car&, const vector<Vehicle>&)> CostFunctionType;

	PathPlanner(const MapData& waypoints);

	Trajectory plan(const Trajectory& previous, const Car& car,
			const vector<Vehicle>& other);

	void register_cost_function(CostFunctionType func, double weight);

private:

	double compute_cost(const Trajectory& trajectory, const Car& initial_state, const vector<Vehicle>& others, bool verbose = false);

	pair<Eigen::VectorXd, Eigen::VectorXd> compute_jmt(Car& car_state, double d_goal, double acceleration);

	void generate_trajectories(Trajectory current, Car car_state, Trajectory& best, double& best_value, const Car& initial_state, const vector<Vehicle>& others);

	MapData _map_data;
	Trajectory _current_trajectory;
	vector<CostFunctionType> _cost_functions;
	vector<double> _weights;

	int _desired_lane = 1;

	double T = 1.5;
	size_t _num_copied = 20;

	static const int NUM_SPLINES_PER_SIDE = 20;
	// factor of 2 due to lane change splines + small correction splines
	static const int NUM_SPLINES = 2 * ( 2 * NUM_SPLINES_PER_SIDE + 1);
	tk::spline _basic_splines[NUM_SPLINES];
};

#endif // PATH_PLANER_H
