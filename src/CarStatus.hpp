#ifndef CAR_STATUS_H
#define CAR_STATUS_H

#include <chrono>
#include "PathPlannerAction.hpp"

typedef std::chrono::milliseconds milliseconds;
typedef std::chrono::high_resolution_clock Clock;

class CarStatus {
    public:
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
        int lane = 1;
        double desired_velocity_mph = 0;
        PathPlannerAction* current_action = nullptr;
        Clock::time_point last_lane_change = Clock::now();
};

#endif // CAR_STATUS_H
