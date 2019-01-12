#ifndef LANE_CHANGE_INFO_H
#define LANE_CHANGE_INFO_H

class LaneChangeInfo {
    public:
        double adjacent_car_delta_speed;

        LaneChangeInfo(double adjacent_car_delta_speed) {
            this->adjacent_car_delta_speed = adjacent_car_delta_speed;
        }
};

#endif // LANE_CHANGE_INFO_H
