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
* This makes sure that even in a curve the speed limit will not be exceeded
* Also if the current speed is 49.4 and the vehicle will accelerate for a short time to 49.9, it is safe

##### 3. Accel and Jerk below limits

* How to stay below the speed limit, see point 2 above
* To keep the Jerk low, we can use the Spline class
  * Spline uses the previous position to calcualte the next few waypoints, considering the jaw_rate 
  * Lines 334 to 439
  
##### 4. No collsions



##### 5. Stay in lane or do fast lane changes



##### 6. Car is able to change lanes


### Possible improvement for next time
1. Use a vehicle class for all vehicles
2. Use more cost functions than just if-loops
3. Force double lane change if it is the best solution
4. Prepare for the law, if overtaking right and/or left is forbidden
5. Clean up main class