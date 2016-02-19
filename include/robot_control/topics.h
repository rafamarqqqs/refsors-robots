#ifndef _TOPICS_H_
#define _TOPICS_H_

#include <cstring>

#define VEL(type) (strcmp(type, "larger_robot") == 0 ? "/larger_robot/cmd_vel" : "/smaller_robot/cmd_vel")
#define SCAN(type) (strcmp(type, "larger_robot") == 0 ? "/larger_robot/scan" : "/smaller_robot/front/scan")
#define LASER(type) (strcmp(type, "larger_robot") == 0 ? "/larger_robot/laser_measures" : "/smaller_robot/laser_measures")
#define POSE(type) (strcmp(type, "larger_robot") == 0 ? "/larger_robot/pose" : "/smaller_robot/pose")
#define MOBILE(type) (strcmp(type, "larger_robot") == 0 ? "husky" : "jackal")

#endif