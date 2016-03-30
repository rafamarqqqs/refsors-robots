#ifndef _TOPICS_H_
#define _TOPICS_H_

#include <cstring>

#define LARGER_ROBOT "larger_robot"
#define SMALLER_ROBOT "smaller_robot"

#define VEL_TOPIC(type) (strcmp(type, LARGER_ROBOT) == 0 ? "/larger_robot/cmd_vel" : "/smaller_robot/cmd_vel")
#define SCAN_TOPIC(type) (strcmp(type, LARGER_ROBOT) == 0 ? "/larger_robot/scan" : "/smaller_robot/front/scan")
#define LASER_TOPIC(type) (strcmp(type, LARGER_ROBOT) == 0 ? "/larger_robot/laser_measures" : "/smaller_robot/laser_measures")
#define POSE_TOPIC(type) (strcmp(type, LARGER_ROBOT) == 0 ? "/larger_robot/pose" : "/smaller_robot/pose")
#define MODEL_NAME(type) (strcmp(type, LARGER_ROBOT) == 0 ? "husky" : "jackal")
#define LASER_DISPLACEMENT(type) (strcmp(type, LARGER_ROBOT) == 0 ? 0.2206 : 0.120)

#define CONTAINERS_MAP "/home/rafael/SORS_Application/src/robot_control/maps/containers.map"
#define PATH_PLANNING_MAP "/home/rafael/SORS_Application/src/robot_control/maps/containers_path_planning.map"

#define KNOWLEDGE_NODE "Knowledge"
#define GET_MAP_SERVICE "/Knowledge/getMap"
#define ADD_TO_MAP_SERVICE "/Knowledge/addToMap"
#define GET_POSITIONS_SERVICE "/Knowledge/getPositions"
#define GET_NEW_GOAL_SERVICE "/Knowledge/getNewGoal"

#define PATH_PLANNING_NODE "Path planning"
#define DEFINE_GLOBAL_PATH_SERVICE "/PathPlanning/defineGlobalPath"

#define LASER_NODE "Laser"

#define LOCALIZATION_NODE "Localization"
#define MODEL_STATES_TOPIC "/gazebo/model_states"

#define MAPPING_NODE "Mapping"
#define CREATE_MAP_ACTION "Mapping/createMap"

#define NAVIGATION_NODE "Navigation"
#define SEARCH_ACTION "Navigation/search"

#define CONTROL_NODE "Control"

/*
#define VEL(type) "/Pioneer3AT/cmd_vel"
#define SCAN(type) "/Pioneer3AT/laserscan"
#define LASER(type) "/Pioneer3AT/laser_measures"
#define POSE(type) "/Pioneer3AT/pose"
#define MOBILE(type) "Pioneer3AT"
*/

#endif