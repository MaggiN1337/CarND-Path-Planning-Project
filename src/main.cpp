#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
//#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

const double MAX_VELOCITY = 49.5;
const double ACCELERATION = .5; //.224;
const double MIN_DISTANCE = 30.0;
/*
 * list of vehicle states
 * KL = keep lane
 * PLCL = prepare lane change left
 * PLCR = prepare lane change right
 * LCL = lane change left
 * LCR = lane change right
 */
const vector<string> VEHICLE_STATES = {"KL", "PLCL", "PLCR", "LCL", "LCR"};

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
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
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
    // start in lane 1;
    int my_lane = 1;

    // initial vehicle state
    string current_state = VEHICLE_STATES[0];

    // set ideal velocity in mph
    double ref_vel = 1.0;

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                        &map_waypoints_dx, &map_waypoints_dy, &ref_vel, &my_lane, &current_state]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {


        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    int prev_size = previous_path_x.size();

                    //START state machine for lane shift options
                    bool change_left_exists = false;
                    bool change_right_exists = false;
                    if (my_lane == 0) {
                        change_right_exists = true;
                    } else if (my_lane == 1) {
                        change_left_exists = true;
                        change_right_exists = true;
                    } else if (my_lane == 2) {
                        change_left_exists = true;
                    }
                    //END state machine for lane shift options

                    //vectors to store lane information for possible lane changing
                    vector<double> lane_speeds = {MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY};
                    vector<double> dist_to_next_car_in_lane = {9999, 9999, 9999};

                    //START avoid hitting other cars

                    //calculate range of lanes
                    double range_of_left_lane = (2 + 4 * my_lane - 2); //0=0m, 1=4m, 2=8m
                    double range_of_right_lane = (2 + 4 * my_lane + 2);  //0=4m, 1=8m, 2=12m

                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    bool too_close = false;

                    //iterate over detected vehicles
                    for (int i = 0; i < sensor_fusion.size(); i++) {
                        //get vehicle's values
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_next_car_s = sensor_fusion[i][5];
                        float d = sensor_fusion[i][6];

                        // calculate lane of vehicle using d
                        int lane_of_next_car = 0;
                        if (d < 8 && d > 4) {
                            lane_of_next_car = 1;
                        } else if (d < 12 && d > 8) {
                            lane_of_next_car = 2;
                        }

                        double check_next_car_speed = sqrt(vx * vx + vy * vy);
                        check_next_car_s += ((double) prev_size * .02 * check_next_car_speed);

                        //store vehicle information for possible lane shifting
                        //if vehicle is on left lane
                        if (lane_of_next_car == my_lane &&
                            dist_to_next_car_in_lane[lane_of_next_car] > check_next_car_s) {

                            lane_speeds[lane_of_next_car] = check_next_car_speed;
                            dist_to_next_car_in_lane[lane_of_next_car] = check_next_car_s;
                        }

                        //if car is in my lane && distance gets too small
                        if (lane_of_next_car == my_lane && check_next_car_s > car_s &&
                            ((check_next_car_s - car_s) < MIN_DISTANCE)) {
                            //reduce speed
                            too_close = true;
                        }
                    }

                    //first, slow down if car in front
                    if (too_close) {
                        ref_vel -= ACCELERATION;

                        //START do lane change

                        if (current_state == VEHICLE_STATES[3] || current_state == VEHICLE_STATES[4]) {
                            //TODO finish lane shift completely
                            if (car_d == 2 || car_d == 6 || car_d == 10) {
                                current_state = VEHICLE_STATES[0];
                            }
                        } else if (change_left_exists &&
                                   (dist_to_next_car_in_lane[my_lane - 1] - car_s) > MIN_DISTANCE) {
                            //TODO call cost function, to identify best lane change move and switch to state PLCL or PLCR
                            my_lane -= 1;
                        } else if (change_right_exists &&
                                   (dist_to_next_car_in_lane[my_lane + 1] - car_s) > MIN_DISTANCE) {
                            my_lane += 1;
                        }
                        //END do lane change

                    } else if (ref_vel < MAX_VELOCITY) {
                        ref_vel += ACCELERATION;
                    }

                    //END avoid hitting other cars

                    // START use spline to smooth ride
                    // waypoints to use spline
                    vector<double> ptsx;
                    vector<double> ptsy;

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    // if no previous path exists, assume car started recently
                    if (prev_size < 2) {
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    }
                        //else use endpoints of previous path
                    else {
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    //add Frenet waypoints
                    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * my_lane), map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * my_lane), map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * my_lane), map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for (int i = 0; i < ptsx.size(); i++) {
                        //shift car ref angle to 0 degrees
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }

                    //create a spline
                    tk::spline spline;
                    spline.set_points(ptsx, ptsy);

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //start with the previous path points
                    for (int i = 0; i < prev_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    double target_x = 30.0;
                    double target_y = spline(target_x);
                    double target_dist = sqrt(target_x * target_x + target_y * target_y);

                    double x_add_on = 0;

                    // apply spline on path
                    for (int i = 1; i <= 50 - prev_size; i++) {

                        double N = (target_dist / (.02 * ref_vel / 2.24));
                        double x_point = x_add_on + target_x / N;
                        double y_point = spline(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        //transfer back to global coordinates
                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }
                    // END use spline to smooth ride

                    json msgJson;

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

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