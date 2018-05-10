#include "helpers.h"
#include <iostream>
#include <math.h>
#include "Eigen-3.3/Eigen/Dense"

// For converting back and forth between radians and degrees.
constexpr double pi()
{
	return M_PI;
}
double deg2rad(double x)
{
	return x * pi() / 180;
}
double rad2deg(double x)
{
	return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
		const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
		const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));


	double angle = fabs(theta - heading);
	angle = min(2 * pi() - angle, angle);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
		const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1],
				maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return
	{	frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
			(maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return
	{	x,y};

}

vector<Vehicle> vehicles_from_sensor_fusion(
		const vector<vector<double>>& sensor_fusion)
{
	vector<Vehicle> vehicles(sensor_fusion.size());

	for (int i = 0; i < sensor_fusion.size(); ++i)
	{
		vehicles[i].id = sensor_fusion[i][0];
		vehicles[i].x = sensor_fusion[i][1];
		vehicles[i].y = sensor_fusion[i][2];
		vehicles[i].vx = sensor_fusion[i][3];
		vehicles[i].vy = sensor_fusion[i][4];
		vehicles[i].s = sensor_fusion[i][5];
		vehicles[i].d = sensor_fusion[i][6];
	}

	return vehicles;
}


void differentiate(double dt, const vector<double>& x, vector<double>& dx)
{
	dx.resize(x.size() - 1);

	for (int i = 0; i < x.size() - 1; ++i)
	{
		dx[i] = (x[i+1] - x[i]) / dt;
	}
}

double yaw(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
	if (v1.norm() * v2.norm() > 0.0)
	{
		double abs_yaw = acos(max(-1.0, min(1.0, v1.dot(v2) / (v1.norm() * v2.norm()))));
		double delta = v1.x() * v2.y() - v1.y() * v2.x();
		if (delta < 0.0)
		{
			return -abs_yaw;
		}
		else
		{
			return abs_yaw;
		}
	}
	else
	{
		return 0.0;
	}
}

double yaw_diff(double x0, double y0, double x1, double y1, double x2, double y2)
{
	Eigen::Vector2d v1(x1 - x0, y1 - y0);
	Eigen::Vector2d v2(x2 - x1, y2 - y1);

	return yaw(v1, v2);
}

Eigen::VectorXd solve_least_squares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
	//return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	return A.colPivHouseholderQr().solve(b);
}

Eigen::MatrixXd compute_poly_matrix(const Eigen::VectorXd& t, int order)
{
	int rows = t.size();
	int cols = order + 1;
//	int order = size - 1;
	Eigen::MatrixXd A(rows, cols);

	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			A(i, j) = pow(t(i), j);
		}
	}

	return A;
}

double eval_polynomial(const Eigen::VectorXd& coeffs, double x)
{
	double result = 0.0;
	double order = coeffs.size() - 1;

	for (int i = 0; i < coeffs.size(); ++i)
	{
		result += coeffs(i) * pow(x, i);
	}

	return result;
}

Eigen::VectorXd JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    double T_2 = T*T;
    double T_3 = T_2 * T;
    double T_4 = T_3 * T;
    double T_5 = T_4 * T;
    Eigen::Matrix3d A;
    A << T_3, T_4, T_5,
         3.0 * T_2, 4.0 * T_3, 5.0 * T_4,
         6.0 * T, 12.0 * T_2, 20.0 * T_3;
    Eigen::Vector3d b;
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T_2),
         end[1] - (start[1] + start[2] * T),
         end[2] - start[2];
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    Eigen::VectorXd result(6);
    result << start[0], start[1], 0.5 * start[2], x(0), x(1), x(2);

    return result;
}

Eigen::VectorXd JMT_ConstantAcceleration(vector< double> start, vector <double> end, double T)
{
	Eigen::VectorXd result(3);

	result << start[0], start[1], (0.5 * (start[1] + end[1]) - start[1]) / T; //(end[0] - start[1] * T - start[0]) / (T * T)

	return result;
}

Eigen::VectorXd JMT_KeepVelocity(vector< double> start, vector <double> end, double T)
{
	  double s_i = start[0];
	  double s1_i = start[1];
	  double s2_i = start[2];
	  double s1_f = end[1];
	  double s2_f = end[2];

	  double A1 = s1_f - (s1_i + s2_i*T);
	  double A2 = s2_f-s2_i;

	  Eigen::VectorXd coeffs(5);

	  coeffs << s_i,
			    s1_i,
	            s2_i*0.5,
	            (3.*A1 - A2*T)/(3.*T*T),
	            (A2*T - 2.*A1)/(4.*T*T*T);

	  return coeffs;
}

Eigen::VectorXd JMT_PosVelConditions(vector<double> start, vector<double> end, double T)
{
	double T_2 = T*T;
	double T_3 = T_2 * T;
	Eigen::MatrixXd A(4, 4);

	// conditions:
	// s(0) = start[0] = s_0
	// v(0) = start[1] = v_0
	// s(T) = end[0] = s_T
	// v(T) = end[1] = v_T
	A << 1.0, 0.0, 0.0, 0.0,
		 0.0, 1.0, 0.0, 0.0,
		 1.0, T, T_2, T_3,
		 0.0, 1.0, 2.0 * T, 3.0 * T_2;
	Eigen::VectorXd b(4);
	b << start[0],
		 start[1],
		 end[0],
		 end[1];

	Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

	return x;
}


Eigen::VectorXd JMT_Approximation(vector<double> start, vector<double> end, double T)
{
	double T_2 = T*T;
	double T_3 = T_2 * T;
	double T_4 = T_3 * T;
	double T_5 = T_4 * T;
	double T_half = 0.5 * T;
	double T_half_2 = T_half * T_half;
	double T_half_3 = T_half_2 * T_half;
	double T_half_4 = T_half_3 * T_half;
	Eigen::MatrixXd A(7, 6);

	// conditions:
	// s(0) = start[0] = s_0
	// v(0) = start[1] = v_0
	// a(0) = start[2] = a_0
	// s(T) = end[0] = s_T
	// v(T) = end[1] = v_T
	// a(T) = end[2] = a_T
	// v_avg = 1/T * integral_{0..T}v(t)dt = 1/T * (s(T) - s(0)) = 0.5 * (v_0 + v_T)
	A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		 1.0, T, T_2, T_3, T_4, T_5,
		 0.0, 1.0, 2.0 * T, 3.0 * T_2, 4.0 * T_3, 5.0 * T_4,
		 0.0, 0.0, 2.0, 3.0 * 2.0 * T, 4.0 * 3.0 * T_2, 5.0 * 4.0 * T_3,
		 0.0, 1.0, T, T_2, T_3, T_4;
	Eigen::VectorXd b(7);
	b << start[0],
		 start[1],
		 start[2],
		 end[0],
		 end[1],
		 end[2],
		 0.5 * (start[1] + end[1]);
	Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

	return x;
}

pair<double, double> compute_global_coords(double yaw, double x_offset, double y_offset, double x_local, double y_local)
{
	double x_global = x_local * cos(yaw) - y_local * sin(yaw) + x_offset;
	double y_global = x_local * sin(yaw) + y_local * cos(yaw) + y_offset;
	return make_pair(x_global, y_global);
}

pair<double, double> compute_local_coords(double yaw, double x_offset, double y_offset, double x_global, double y_global)
{
	// translation
	double x_tmp = x_global - x_offset;
	double y_tmp = y_global - y_offset;

	// rotation
	double x_local = x_tmp * cos(-yaw) - y_tmp * sin(-yaw);
	double y_local = x_tmp * sin(-yaw) + y_tmp * cos(-yaw);

	return make_pair(x_local, y_local);
}
