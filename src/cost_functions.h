#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H


#include <vector>
#include "path_planner.h"
#include "helpers.h"


double standing_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double speed_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double efficiency_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double exceeds_max_speed_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double max_acceleration_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double total_acceleration_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double max_jerk_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double total_jerk_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double off_road_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double collision_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double other_veh_gap_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double free_trajectory_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double max_dist_from_center_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double keep_lane_cf(const TrajectoryInformation&, const Car&, const vector<Vehicle>&);

double right_direction_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double min_d_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double min_lane_changes_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double min_total_yaw_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double good_goal_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double min_d_range_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

double exceeds_a_y_cf(const TrajectoryInformation& traj, const Car& car, const vector<Vehicle>& other);

#endif
