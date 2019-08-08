# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

Model Documentation by Marcus Neuert, 08.08.2019

### Project Goals and how I managed them

No. | Criteria | How I solved it 
---|---------|---------------
1|Drive at least 4.32 miles without incident..|Keep distance and speed in safe range, do lane changes safe but completely, calculate costs and compare lane changing options
2|The car drives according to the speed limit.|Stay below 49.5 mph and accelerate with .5-steps to always stay below 49.99 mph
3|Max Acceleration and Jerk are not Exceeded.|Acceleration (positive & negative in each frame by 0.5 mph will hold Max Accel low, using Spline to change lanes smoothly keeps the Jerk low
4|Car does not have collisions. | Always hold distance to speed/2 and brake harder than usual if suddenly another vehicle jumps into the lane and distance get too small  
5|The car stays in its lane, except for less than 3 sec. between changing lanes.| Stay in state PrepareLaneChange until vehicle is in the middle of the new lane
6|The car is able to change lanes|Used a simple cost function to compare lane change or keep lane possibilities within the next 60 meters

### Code explanation

##### 1. Drive 4.32 miles safely 
![Screenshot](/doc/finished_4.35_miles.PNG)

* To drive safely all the way, you could simply hold the lane and follow the vehicle in front of the car.
* To drive it also faster, you need to change to faster lanes, if they are free for at least about 60 meters to overtake.
* To avoid collisions with vehicles that change suddenly with low distance, you need to prepare a harder brake method

##### 2. Stay below speed limit

* The maximum velocity + acceleration-step should never exceed the speed limit. Therefore I set them to 49.5 and 0.5.
* Also if the current speed is 49.4 and the vehicle will accelerate for a short time to 49.9, 
it will slow down in the next step
* This makes sure that even in a curve the speed limit will not be exceeded
   
lines   234 - 244:                 
                   
                    if (too_close) {

                        //slow down if car in front is still faster
                        if (ref_vel > lane_speeds[my_lane] * 0.9) {
                            ref_vel -= ACCELERATION;
                        }

                        //brake hard to avoid collision
                        if (way_too_close) {
                            ref_vel -= ACCELERATION * 5;
                        }
                        
lines 328 - 330:

                    } else if (ref_vel < MAX_VELOCITY) {
                        ref_vel += ACCELERATION;
                    }

##### 3. Accel and Jerk below limits

* How to stay below the speed limit, see point 2 above
* To keep the Jerk low, we can use the Spline class
  * Spline uses the previous position to calculate the next few waypoints, considering the jaw_rate 
  * Lines 334 to 439
  
##### 4. No collisions

To avoid collisions, the vehicle will check if another car is
* in its lane in front
* the distance in the predicted next step is lower than the half of the current speed
* suddenly another car in the front is way too close to it (because it suddenly changed the lane), 
the car will brake 5-times harder, see code in point 2 (lines 241-244)



lines 184 - 193: 
                           
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

##### 5. Stay in lane or do fast lane changes

Using the state machine helps a lot for the lane changes.

    const vector<string> VEHICLE_STATES = {"KL", "PLCL", "PLCR", "LCL", "LCR"};

The lines 246 to 326 concetrate on lane changing but only the following part helps to avoid toggling between the lanes:

                        //1. do not change lanes if already in state LCL or LCR
                        // change state to Keep Lane, if change is nearly completed
                        if (current_state == VEHICLE_STATES[3] || current_state == VEHICLE_STATES[4]) {

                            //set state to KL if vehicle is back to middle of lane
                            if (abs(my_lane + 2 - car_d) < 0.2) {
                                current_state = VEHICLE_STATES[0];
                            }

                        }
                        
Therfore you need to specify a range for d, that tells the vehicle that the lane change is completed.       

##### 6. Car is able to change lanes

To figure out if another lane exists and has enough space, the state machine in lines 196-218 are necessary, 
for the vehicle to change safely. I  choose the distance to the vehicle from behind to be smaller than the distance to 
the front, because we only change with a higher speed and if the new lane is free for the next 60 meters to accelerate
and overtake.

e.g. this code is relevant

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
                                         
Later in the code, the lines 258 to 326 implement to call of the cost function based on the lane change possibilities.

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

                        }
                        
The cost function I used in cost.cpp, I took from the classroom:

    float cost = (2*target_speed - lane_speeds[intended_lane] - lane_speeds[final_lane]) / target_speed;

### Possible improvement for next time
1. Use a vehicle class for all vehicles
2. Use more cost functions than just if-loops
3. Force double lane change if it is the best solution
4. Prepare for the law, if overtaking right and/or left is forbidden
5. Clean up main class