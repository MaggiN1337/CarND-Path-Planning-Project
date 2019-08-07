#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "cost.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

const float MAX_VELOCITY = 49.5;
const double ACCELERATION = .5; //.224;
const double DISTANCE_TO_OVERTAKE = 60.0;
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
        if (length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (!s.empty()) {
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
//                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    int prev_size = previous_path_x.size();

                    //vectors to store lane information for possible lane changing
                    vector<float> lane_speeds = {MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY};
                    vector<double> dist_to_next_car_in_lane_front = {999, 999, 999};
                    vector<double> dist_to_next_car_in_lane_rear = {999, 999, 999};

                    //calculate distance to next vehicle
                    double expected_distance = car_speed / 2;

                    //START avoid hitting other cars

                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    bool too_close = false;
                    bool way_too_close = false;

                    //iterate over detected vehicles
                    for (auto & i : sensor_fusion) {
                        //get vehicle's values
                        double vx = i[3];
                        double vy = i[4];
                        double check_next_car_s = i[5];
                        float d = i[6];

                        // calculate lane of vehicle using d, ignoring exact position on the lines
                        int lane_of_next_car = 0;
                        if (d < 8 && d > 4) {
                            lane_of_next_car = 1;
                        } else if (d < 12 && d > 8) {
                            lane_of_next_car = 2;
                        }

                        double check_next_car_speed = sqrt(vx * vx + vy * vy);

                        check_next_car_s += ((double) prev_size * .02 * check_next_car_speed);

                        double distance_to_me = check_next_car_s - car_s;

                        //debug
//                        cout << "Car " << i << " on lane " << lane_of_next_car << " is in this distance: "
//                             << distance_to_me << endl;

                        //START store vehicle information for possible lane shifting
                        // if current vehicle is closer than previous vehicle on its lane
                        if (distance_to_me > 0 && distance_to_me < dist_to_next_car_in_lane_front[lane_of_next_car]) {
                            //store distance and speed to next vehicle in front

                            //get lane speed of next car in front an convert to mph
                            if (distance_to_me > DISTANCE_TO_OVERTAKE) {
                                lane_speeds[lane_of_next_car] = MAX_VELOCITY;
                            } else {
                                lane_speeds[lane_of_next_car] = check_next_car_speed * 2.24;
                            }
                            //get relative distance to my car
                            dist_to_next_car_in_lane_front[lane_of_next_car] = distance_to_me;

                        } else if (distance_to_me < 0 &&
                                   abs(distance_to_me) < abs(dist_to_next_car_in_lane_rear[lane_of_next_car])) {
                            //store distance to next vehicle from behind
                            //get relative distance to my car
                            dist_to_next_car_in_lane_rear[lane_of_next_car] = distance_to_me;
                        }
                        //END store vehicle information for possible lane shifting

                        //if car is in my lane && distance gets too small
                        if (lane_of_next_car == my_lane && check_next_car_s > car_s &&
                            distance_to_me < expected_distance) {
                            if (distance_to_me < expected_distance / 2) {
                                //brake very hard
                                way_too_close = true;
                            }
                            //reduce speed
                            too_close = true;
                        }
                    }

                    //START state machine for lane shift options
                    bool change_left_exists = false;
                    bool change_right_exists = false;

                    //check if lane change is possible by law
                    if (my_lane == 0) {
                        change_right_exists = true;
                    } else if (my_lane == 1) {
                        change_left_exists = true;
                        change_right_exists = true;
                    } else if (my_lane == 2) {
                        change_left_exists = true;
                    }

                    //check if lanes have enough room for me
                    change_left_exists = change_left_exists &&
                                         (dist_to_next_car_in_lane_front[my_lane - 1]) > expected_distance &&
                                         (dist_to_next_car_in_lane_rear[my_lane - 1]) < -expected_distance * 0.6;

                    change_right_exists = change_right_exists &&
                                          (dist_to_next_car_in_lane_front[my_lane + 1]) > expected_distance &&
                                          (dist_to_next_car_in_lane_rear[my_lane + 1]) < -expected_distance * 0.6;
                    //END state machine for lane shift options

                    //debug
//                    for (int i = 0; i < 3; i++) {
//                        cout << car_speed << " is my speed" << endl;
//                        cout << lane_speeds[i] << " mph on lane " << i << endl;
//                        cout << dist_to_next_car_in_lane_front[i] << " m distance to vehicle front on lane " << i
//                             << endl;
//                        cout << dist_to_next_car_in_lane_rear[i] << " m distance to vehicle back on lane " << i << endl;
//                    }
//                    cout << "my state: " << current_state << endl;
//                    cout << dist_to_next_car_in_lane_front[my_lane] << " m distance to vehicle in front" << endl;
//                    cout << dist_to_next_car_in_lane_rear[my_lane] << " m distance to vehicle in back" << endl;
//                    cout << "LCL exists: " << change_left_exists << endl;
//                    cout << "LCR exists: " << change_right_exists << endl;

                    if (too_close) {

                        //slow down if car in front is still faster
                        if (ref_vel > lane_speeds[my_lane] * 0.9) {
                            ref_vel -= ACCELERATION;
                        }

                        //brake hard to avoid collision
                        if (way_too_close) {
                            ref_vel -= ACCELERATION * 5;
                        }

                        //START do lane change

                        //1. do not change lanes if already in state LCL or LCR
                        // change state to Keep Lane, if change is nearly completed
                        if (current_state == VEHICLE_STATES[3] || current_state == VEHICLE_STATES[4]) {

                            //set state to KL if vehicle is back to middle of lane
                            if (abs(my_lane + 2 - car_d) < 0.2) {
                                current_state = VEHICLE_STATES[0];
                            }

                        }
                        /*call cost function, to identify best lane change move and switch to state PLCL or PLCR
                         *
                         * TODO 0. consider all lanes even if double change is required
                         * 1. higher speed, closer to maximum is better
                         * 2. empty lane is better or at least DISTANCE_TO_OVERTAKE (100) meters
                         * 3. left change is better (by always using first in if-conditions)
                         *
                         * */

//                        float cost;
//                        cout << car_speed << " is my speed" << endl;
//                        cout << "lane speeds: " << lane_speeds[0] << ", " << lane_speeds[1] << ", " << lane_speeds[2] << endl;
//                        for (int i=0; i< 3; i++){
//                            for (int j=0; j < 3; j++){
//                                cost = inefficiency_cost(MAX_VELOCITY, i, j, lane_speeds);
//                                cout << "Change to lane " << i << " & then to lane " << j << " costs: " << cost << endl;
//                            }
//                        }

                        // 2. calculate costs of lane change options if in state Keep lane
                        if (current_state == VEHICLE_STATES[0] && change_left_exists && change_right_exists) {
                            float cost_CL = inefficiency_cost(MAX_VELOCITY, my_lane - 1, my_lane - 1, lane_speeds);
                            float cost_CR = inefficiency_cost(MAX_VELOCITY, my_lane + 1, my_lane + 1, lane_speeds);
                            float cost_KL = inefficiency_cost(MAX_VELOCITY, my_lane, my_lane, lane_speeds);

                            if (cost_CL < cost_KL && cost_CL < cost_CR) {
                                current_state = VEHICLE_STATES[1];
                            } else if (cost_CR < cost_KL && cost_CR < cost_CL) {
                                current_state = VEHICLE_STATES[2];
                            }

                        } else if (change_left_exists) {
                            float cost_CL = inefficiency_cost(MAX_VELOCITY, my_lane - 1, my_lane - 1, lane_speeds);
                            float cost_KL = inefficiency_cost(MAX_VELOCITY, my_lane, my_lane, lane_speeds);
                            //PLCL
                            if (cost_CL < cost_KL) {
                                current_state = VEHICLE_STATES[1];
                            }

                        } else if (change_right_exists) {
                            float cost_CR = inefficiency_cost(MAX_VELOCITY, my_lane + 1, my_lane + 1, lane_speeds);
                            float cost_KL = inefficiency_cost(MAX_VELOCITY, my_lane, my_lane, lane_speeds);
                            //PLCR
                            if (cost_CR < cost_KL) {
                                current_state = VEHICLE_STATES[2];
                            }
                        }

                        //3. prepare for lane change if lane is free
                        if (current_state == VEHICLE_STATES[1] && change_left_exists) {
                            //change lane and state
                            my_lane -= 1;
                            current_state = VEHICLE_STATES[3];

                            //speed up too slow
                            if (car_speed < MAX_VELOCITY * 0.9) {
                                ref_vel += ACCELERATION;
                            }
                        } else if (current_state == VEHICLE_STATES[2] && change_right_exists) {
                            //change lane and state
                            my_lane += 1;
                            current_state = VEHICLE_STATES[4];

                            //speed up too slow
                            if (car_speed < MAX_VELOCITY * 0.9) {
                                ref_vel += ACCELERATION;
                            }
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