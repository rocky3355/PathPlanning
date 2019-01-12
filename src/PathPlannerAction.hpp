#ifndef PATH_PLANNER_ACTION_H
#define PATH_PLANNER_ACTION_H

class PathPlannerAction {
    public:
        double cost;
        int lane_shift;
        
        // Round the costs to filter out jitters. If values are the same, straight will be preferred
		// as this action is the first one in the list
        PathPlannerAction(int lane_shift, double cost) {
            this->cost = std::round(cost);
            this->lane_shift = lane_shift;
        }

        bool operator<(const PathPlannerAction& other) {
            return cost < other.cost;
        }
};

#endif // PATH_PLANNER_ACTION_H
