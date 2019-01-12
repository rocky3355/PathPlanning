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
#include "CarStatus.hpp"
#include "LaneChangeInfo.hpp"
#include "Parameters.h"
#include "spline.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

  // Load up map values for waypoint's x,y,s and d normalized normal std::vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < map_waypoints_x.size(); i++)
	{
		double map_x = map_waypoints_x[i];
		double map_y = map_waypoints_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta)
{

	int closestWaypoint = ClosestWaypoint(x, y);

	double map_x = map_waypoints_x[closestWaypoint];
	double map_y = map_waypoints_y[closestWaypoint];

	double heading = std::atan2((map_y-y),(map_x-x));

	double angle = std::fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == map_waypoints_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta)
{
	int next_wp = NextWaypoint(x, y, theta);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = map_waypoints_x.size()-1;
	}

	double n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
	double n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];
	double x_x = x - map_waypoints_x[prev_wp];
	double x_y = y - map_waypoints_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-map_waypoints_x[prev_wp];
	double center_y = 2000-map_waypoints_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(map_waypoints_x[i],map_waypoints_y[i],map_waypoints_x[i+1],map_waypoints_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d)
{
	int prev_wp = -1;

	while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%map_waypoints_x.size();

	double heading = std::atan2((map_waypoints_y[wp2]-map_waypoints_y[prev_wp]),(map_waypoints_x[wp2]-map_waypoints_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-map_waypoints_s[prev_wp]);

	double seg_x = map_waypoints_x[prev_wp]+seg_s * std::cos(heading);
	double seg_y = map_waypoints_y[prev_wp]+seg_s * std::sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d * std::cos(perp_heading);
	double y = seg_y + d * std::sin(perp_heading);

	return {x,y};
}

double getCenterOfLane(int lane) {
	double center = lane * LANE_WIDTH + LANE_WIDTH / 2;
	return center;
}

LaneChangeInfo* checkCarsOnAdjacentLane(int lane_shift, const CarStatus& car_status, const std::vector<std::vector<double>>& sensor_fusion) {
	int target_lane = car_status.lane + lane_shift;
	if (target_lane < MIN_LANE || target_lane > MAX_LANE) {
		return nullptr;
	}

	int nearest_car_idx = -1;
	double lowest_distance = ADJECENT_MAX_DISTANCE_FOR_CONSIDERATION;

	for (int i = 0; i < sensor_fusion.size(); i++) {
		double s = sensor_fusion[i][5];
		double d = sensor_fusion[i][6];
		double longitudinal_distance = s - car_status.s;

		if (d > target_lane * LANE_WIDTH && d < (target_lane + 1) * LANE_WIDTH) {
			double free_space = std::abs(longitudinal_distance);
			if (free_space < MIN_FREE_SPACE_LANE_CHANGE) {
				return nullptr;
			}

			if (longitudinal_distance > 0 && longitudinal_distance < lowest_distance) {
				nearest_car_idx = i;
				lowest_distance = longitudinal_distance;
			}
		}
	}

	double delta_v = 0;
	if (nearest_car_idx > -1) {
		double vx = sensor_fusion[nearest_car_idx][3];
		double vy = sensor_fusion[nearest_car_idx][4];
		double v = std::sqrt(vx * vx + vy * vy) * MS_TO_MPH;
		delta_v = TARGET_VELOCITY_MPH - v;
	}

	LaneChangeInfo* info = new LaneChangeInfo(delta_v);
	return info;
}

void performAction(CarStatus& car_status, bool car_in_front, const std::vector<std::vector<double>>& sensor_fusion) {
	if (car_status.current_action == nullptr) {
		LaneChangeInfo* info_left = checkCarsOnAdjacentLane(-1, car_status, sensor_fusion);
		LaneChangeInfo* info_right = checkCarsOnAdjacentLane(1, car_status, sensor_fusion);

		std::vector<PathPlannerAction> actions;
		double straight_cost = std::abs(TARGET_VELOCITY_MPH - car_status.desired_velocity_mph) * SPEED_COST + std::abs(car_status.lane - DESIRED_LANE) * LANE_COST;
		actions.push_back(PathPlannerAction(0, straight_cost));

		//std::cout << "Straight: " << straight_cost;
		int time_since_last_last_change = std::chrono::duration_cast<milliseconds>(Clock::now() - car_status.last_lane_change).count();

		if (time_since_last_last_change > TIME_UNTIL_NEXT_LANE_CHANGE_MS && (car_status.lane != DESIRED_LANE || car_in_front)) {
			if (info_left != nullptr) {
				double lane_change_left_cost = std::abs((car_status.lane - 1) - DESIRED_LANE) * LANE_COST + info_left->adjacent_car_delta_speed * SPEED_COST;
				actions.push_back(PathPlannerAction(-1, lane_change_left_cost));
				delete info_left;
				//std::cout << "  -  Left: " << lane_change_left_cost;
			}
			if (info_right != nullptr) {
				double lane_change_right_cost = std::abs((car_status.lane + 1) - DESIRED_LANE) * LANE_COST + info_right->adjacent_car_delta_speed * SPEED_COST;
				actions.push_back(PathPlannerAction(1, lane_change_right_cost));
				delete info_right;
				//std::cout << "  -  Right: " << lane_change_right_cost;
			}
		}
		//std::cout << std::endl;

		std::sort(actions.begin(), actions.end());
		PathPlannerAction desired_action = actions[0];
		car_status.lane += desired_action.lane_shift;

		if (desired_action.lane_shift != 0) {
			car_status.current_action = new PathPlannerAction(desired_action.lane_shift, desired_action.cost);
		}
	}
	else {
		if (car_status.d > car_status.lane * LANE_WIDTH && car_status.d < (car_status.lane + 1) * LANE_WIDTH) {
			delete car_status.current_action;
			car_status.current_action = nullptr;
			car_status.last_lane_change = Clock::now();
		}
	}
}

double getBrakingValue(const CarStatus& car_status, const std::vector<std::vector<double>>& sensor_fusion) {
	for (int i = 0; i < sensor_fusion.size(); i++) {
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double s = sensor_fusion[i][5];
		double d = sensor_fusion[i][6];

		if (d > car_status.lane * LANE_WIDTH && d < (car_status.lane + 1) * LANE_WIDTH) {
			double distance = s - car_status.s;
			double distance_threshold = car_status.speed / 1.2;

			if (s > car_status.s && distance < distance_threshold) {
					double distance_violation = distance_threshold - distance;
					double car_vel_mph = std::sqrt(vx * vx + vy * vy) * MS_TO_MPH;
					double speed_diff = car_status.desired_velocity_mph - car_vel_mph;
					double brake_value = speed_diff * BRAKE_SPEED_FACTOR + distance_violation * BRAKE_DIST_FACTOR;
					return brake_value;
			}
		}
	}

	return 0.0;
}

void calculateNextPoints(const CarStatus& car_status, const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y,
												std::vector<double>& next_x_vals, std::vector<double>& next_y_vals) {

	double desired_velocity_ms = car_status.desired_velocity_mph * MPH_TO_MS;

	double smooth_step = desired_velocity_ms * TIME_TO_REACH_POINT;
	std::vector<double> rough_x(NUMBER_OF_ROUGH_POINTS);
	std::vector<double> rough_y(NUMBER_OF_ROUGH_POINTS);
	int prev_size = previous_path_x.size();

	double ref_x = car_status.x;
	double ref_y = car_status.y;
	double ref_yaw = deg2rad(car_status.yaw);

	if (prev_size < 2) {
		double prev_car_x = car_status.x - std::cos(car_status.yaw);
		double prev_car_y = car_status.y - std::sin(car_status.yaw);
		rough_x[0] = prev_car_x;
		rough_x[1] = car_status.x;
		rough_y[0] = prev_car_y;
		rough_y[1] = car_status.y;
	}
	else {
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];
		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		rough_x[0] = ref_x_prev;
		rough_x[1] = ref_x;
		rough_y[0] = ref_y_prev;
		rough_y[1] = ref_y;
	}

	for (int i = 2; i < NUMBER_OF_ROUGH_POINTS; i++) {
		double next_s = car_status.s + (i - 1) * ROUGH_STEP_SIZE * (car_status.current_action == nullptr ? 1.0 : LANE_CHANGE_ROUGH_STEP_SIZE_FACTOR);
		double next_d = getCenterOfLane(car_status.lane);
		std::vector<double> xy = getXY(next_s, next_d);
		rough_x[i] = xy[0];
		rough_y[i] = xy[1];
	}

	double yaw_sin = std::sin(-ref_yaw);
	double yaw_cos = std::cos(-ref_yaw);

	for (int i = 0; i < NUMBER_OF_ROUGH_POINTS; i++) {
		double shift_x = rough_x[i] - ref_x;
		double shift_y = rough_y[i] - ref_y;
		rough_x[i] = shift_x * yaw_cos - shift_y * yaw_sin;
		rough_y[i] = shift_x * yaw_sin + shift_y * yaw_cos;
	}

	tk::spline spline;
	spline.set_points(rough_x, rough_y);

	for (int i = 0; i < prev_size; i++) {
		next_x_vals[i] = previous_path_x[i];
		next_y_vals[i] = previous_path_y[i];
	}

	yaw_sin = std::sin(ref_yaw);
	yaw_cos = std::cos(ref_yaw);

	for (int i = 0; i < next_x_vals.size() - prev_size; i++) {
		double x_point = (i + 1) * smooth_step;
		double y_point = spline(x_point);

		double x_ref = x_point;

		x_point = x_ref * yaw_cos - y_point * yaw_sin;
		y_point = x_ref * yaw_sin + y_point * yaw_cos;
		x_point += ref_x;
		y_point += ref_y;

		next_x_vals[prev_size + i] = x_point;
		next_y_vals[prev_size + i] = y_point;
	}     
}

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
  	std::istringstream iss(line);
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
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

	CarStatus car_status;

  h.onMessage([&car_status](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
        
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	car_status.x = j[1]["x"];
          	car_status.y = j[1]["y"];
          	car_status.s = j[1]["s"];
          	car_status.d = j[1]["d"];
          	car_status.yaw = j[1]["yaw"];
          	car_status.speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	std::vector<double> previous_path_x = j[1]["previous_path_x"];
          	std::vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	std::vector<std::vector<double>> sensor_fusion = j[1]["sensor_fusion"];

						double braking_value = getBrakingValue(car_status, sensor_fusion);
						bool car_in_front = braking_value > 0;

            if (car_in_front) {
                braking_value = braking_value > MAX_SPEED_DIFF_PER_STEP ? MAX_SPEED_DIFF_PER_STEP : braking_value;
                car_status.desired_velocity_mph -= braking_value;
            }
            // Don't use car_status.speed, it lags behind and will cause desired velocity to get too high
            else if (car_status.desired_velocity_mph < TARGET_VELOCITY_MPH) {
                car_status.desired_velocity_mph += MAX_SPEED_DIFF_PER_STEP;
            }

						performAction(car_status, car_in_front, sensor_fusion);

          	std::vector<double> next_x_vals(NUMBER_OF_PATH_POINTS);
          	std::vector<double> next_y_vals(NUMBER_OF_PATH_POINTS);

						calculateNextPoints(car_status, previous_path_x, previous_path_y, next_x_vals, next_y_vals);

						json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

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
