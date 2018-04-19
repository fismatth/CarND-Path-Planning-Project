#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "path_planner.h"
#include "cost_functions.h"

using namespace std;

// for convenience
using json = nlohmann::json;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0


  MapData map_data;
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_data.map_waypoints_x.push_back(x);
  	map_data.map_waypoints_y.push_back(y);
  	map_data.map_waypoints_s.push_back(s);
  	map_data.map_waypoints_dx.push_back(d_x);
  	map_data.map_waypoints_dy.push_back(d_y);
  }

	PathPlanner path_planner(map_data);
	path_planner.register_cost_function(speed_cf, 1.0);
	path_planner.register_cost_function(keep_lane_cf, 1.0);
	path_planner.register_cost_function(min_lane_changes_cf, 1.0);
	path_planner.register_cost_function(min_total_yaw_cf, 0.5);
	path_planner.register_cost_function(collision_cf, 1.0);
	path_planner.register_cost_function(off_road_cf, 1.0);
//	path_planner.register_cost_function(max_acceleration_cf, 1.0);

//	path_planner.register_cost_function(standing_cf, 1.0);
//	path_planner.register_cost_function(exceeds_max_speed, 1.0);
//	path_planner.register_cost_function(total_acceleration_cf, 0.1);
//	path_planner.register_cost_function(max_jerk_cf, 0.0);
//	path_planner.register_cost_function(total_jerk_cf, 0.0);
//	path_planner.register_cost_function(other_veh_gap_cf, 1.0);
//	path_planner.register_cost_function(right_direction_cf, 1.0);
//	path_planner.register_cost_function(min_d_cf, 1.0);
//	path_planner.register_cost_function(max_dist_from_center_cf, 1.0);

  h.onMessage([&path_planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
        	Car car;
          	car.x = j[1]["x"];
          	car.y = j[1]["y"];
          	car.s = j[1]["s"];
          	car.d = j[1]["d"];
          	car.yaw = deg2rad(j[1]["yaw"]);
          	car.speed = j[1]["speed"];
          	car.speed *= MPH_2_MPERSEC;

          	// Previous path data given to the Planner
          	Trajectory previous;
          	vector<double> previous_x = j[1]["previous_path_x"];
          	vector<double> previous_y = j[1]["previous_path_y"];
          	previous.x_values = previous_x;
          	previous.y_values = previous_y;
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
          	auto other_vehicles = vehicles_from_sensor_fusion(sensor_fusion);

          	json msgJson;

          	Trajectory new_trajectory = path_planner.plan(previous, car, other_vehicles);


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = new_trajectory.x_values;
          	msgJson["next_y"] = new_trajectory.y_values;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
