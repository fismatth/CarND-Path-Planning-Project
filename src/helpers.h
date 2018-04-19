#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <vector>
#include <math.h>
#include <vector>
#include <iostream>

using namespace std;

const double dt = 0.02;
const double max_s = 6945.554;

const double MPH_2_MPERSEC = 0.44704;
const double MAX_SPEED = 50.0 * MPH_2_MPERSEC;// ?
const double TARGET_SPEED = 0.95 * MAX_SPEED;
const double MAX_ACCELERATION = 10.0;
const double MAX_JERK = 10.0;
const double EXPECTED_ACCELERATION = 1.0;
const double VEHICLE_RADIUS = 1.5;

const double LANE_WIDTH = 4.0;
const unsigned int NUM_LANES = 3;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
		const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
		const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
		const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y);

struct Vehicle
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
};

vector<Vehicle> vehicles_from_sensor_fusion(
		const vector<vector<double>>& sensor_fusion);

void differentiate(double dt, const vector<double>& x, vector<double>& dx);

double yaw_diff(double x0, double y0, double x1, double y1, double x2, double y2);

template<typename T>
ostream& operator<<(ostream& os, const std::vector<T>& v)
{
	if (v.size() == 0)
	{
		return os << "[]" << endl;
	}

	os << "[";

	for (size_t i = 0; i < v.size() - 1; ++i)
	{
		os << v[i] << ", ";
	}

	return os << v[v.size()-1] << "]" << endl;
}

#endif // HELPERS_H
