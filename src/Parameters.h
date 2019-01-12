#ifndef PARAMETERS_H
#define PARAMETERS_H

#define LANE_COST                                   3.0
#define SPEED_COST                                  2.0
#define BRAKE_SPEED_FACTOR                          0.01
#define BRAKE_DIST_FACTOR                           0.003
#define NUMBER_OF_ROUGH_POINTS                      5
#define TIME_TO_REACH_POINT                         0.02
#define MS_TO_MPH                                   2.237
#define MPH_TO_MS                                   1.0 / MS_TO_MPH
#define NUMBER_OF_PATH_POINTS                       50
#define ROUGH_STEP_SIZE                             30.0
#define TARGET_VELOCITY_MPH                         49.5
#define LANE_WIDTH                                  4.0
#define MAX_SPEED_DIFF_PER_STEP                     0.2
#define MIN_FREE_SPACE_LANE_CHANGE                  10.0
#define TIME_UNTIL_NEXT_LANE_CHANGE_MS              5000
#define ADJECENT_MAX_DISTANCE_FOR_CONSIDERATION     60.0
#define LANE_CHANGE_ROUGH_STEP_SIZE_FACTOR          1.7
#define DESIRED_LANE                                1
#define MIN_LANE                                    0
#define MAX_LANE                                    2

#endif // PARAMETERS_H
