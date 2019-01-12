# Path Planning
The goal of this project was to create a simple path planner that can make its way through highway traffic. The following steps show how the algorithm works. The algorithm can be tweaked very detailed through the parameters file to achieve different behaviors.

### 1. Trajectory generation
The trajectory the car should follow will have equally spaced waypoints, the distance between them being calculated by the desired velocity. The distances between the points will always be travelled within 20ms. Therefore, the spacing can be used to control the car's speed. The generator uses Frenet coordinates to create some "rough" waypoints along the s-coordinate and the desired lane (i.e. the d-coordinate), each having a longitudinal spacing of 30m by default. Using Frenet coordinates is much easier for this purpose than using the world's xy coordinate system. A smooth transition between the points, meaning reducing the jerk, is requried. Thus, for each iteration, the last two points of the previous trajectory are used as start for the new trajectory. This helps smoothening the path, as a spline will be used together with the rough points to ensure that all waypoints will be passed in a smoothly manner. So there will be now hard "cut" where an old trajectory ends and a new one starts. A fixed number of points will generated between (and possibly also beyond) the rough points. As already mentioned the spacing has to be according to the desired speed. In the end, the waypoints are being transformed to the world's coordinate system and sent to the simulator.

### 2. Trajectory selection
The trajectory selection does not exactly work like this would be normally done in a car. Instead of having multiple trajectories with each a cost assigned to it, it selects an action. There are three actions: Stay on lane, lane change left and lane change right. Braking because of ahead vehicles or trying to reach the target speed are not considered as actions and will always be applied. The selector will evaluate if an action is possible and then assign a cost to it. An action is always coupled to a lane shift value (-1, 0, 1) which indicates how the current lane should be shifted. The lane shift will result in a certain d value, leading to a certain trajectory within the trajectory generator. No action can be executed during a lane change, the selector has to wait until a lane change has been finished. Furthermore, a lane change can only performed after a certain time span has passed since the last lane change. This is required to let the car stabilize itself in its new lane, as well as preventing too many lane changes, which would be inconvenient for the passengers. Lastly, a lane change will be only considered if a car is obstructing the way or if the car is not in its desired lane. This saves some computational power whenever a lane change is clearly not an option. For lane changes, the distance between the rough points will be increased in order to generate smoother trajectories with less jerk. The default spacing would lead to a higher jerk that violates the simulator's rules. However, using that increased spacing for the usual trajectory generation will lead to inaccuracies while following the street.

#### 2.1 Other vehicles
A basic consideration of other vehicles is checking for cars in front. If the distance to the car in front (on the same lane) falls below a certain threshold, the desired speed will be reduced depending on the distance and the delta speed. I.e. if a car suddenly switches to our lane with a lower speed, our car would brake harder than if we slowly approach a car on our lane. This does not directly influence the action selection. If there are cars on adjacent lanes within a certain distance, either in front or behind the ego vehicle, a lane change towards the respective lane will be prevented.

#### 2.2 Cost function
To decide which of the three actions is the optimal one to perform, a cost function is being used. For each action, a distinct cost is being calcualuted and then assigned to it. While assigning, the cost is rounded towards the nearest integer. This prevents jumping between lanes if there is jittering within the cost values or only a minimal gain for the car. The action having the lowest cost will then be selected. The cost function considers three critera:
- Delta between current speed and target speed
- Delta between new lane and desired lane
- Delta between current speed and nearby cars in front within the target lane

If there is only a small delta towards the target speed, a lane change will be considered unnecessary. If the delta speed keeps increasing, the lane change will be considered at some point. For this, the delta speed between the car and vehicles on the possible target lanes will also be taken into account. Only cars that are in front of the ego vehicle and within a certain range will be evaluated. Thus, the cost function will help to avoid lane changes if this would lead to ending up behind an even slower car. The cost function also will drag the car into the (parameterized) desired lane. As default, the center lane has been chosen, but any other lane can be chosen.

### Conclusion
The created path planner works pretty well for the given scenario. However, some points for further improvement could be considered:
- Create actions for double lane changes
- Predict the other vehicles' positions in the future (i.e. for free space checking before lane change)
