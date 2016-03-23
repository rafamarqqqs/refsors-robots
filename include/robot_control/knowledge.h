#include <list>
#include <ros/ros.h>
#include <stdlib.h> 
#include <limits.h>
#include "resources.h"
#include "topics.h"
#include "robot_control/addToMap.h"
#include "robot_control/getPositions.h"
#include "robot_control/getNewGoal.h"
#include "robot_control/defineGlobalPath.h"
#include <actionlib/server/simple_action_server.h>
#include <fstream>
#include <stdlib.h> 
#include <cmath>
#include <vector>

using namespace std;

#ifndef _KNOWLEDGE_H_
#define _KNOWLEDGE_H_

class Knowledge {
public:
	Knowledge(ros::NodeHandle node, double length, double width, double cell_size, double area_size);
    ~Knowledge();

    bool addToMap(robot_control::addToMap::Request& req, robot_control::addToMap::Response& res);
    bool getPositions(robot_control::getPositions::Request& req, robot_control::getPositions::Response& res);
    bool getNewGoal(robot_control::getNewGoal::Request& req, robot_control::getNewGoal::Response& res);

private:
    void writeMap();
	inline bool isCurrentGoal(uint8_t x, uint8_t y);
    inline void writeAreas();
	void initMaps();
	void initAreas();

	uint8_t** areas;

	vector<Goal>* goals;

	char** map;
	uint16_t** map_scans;

	int8_t ids;
	std::vector<_2DPoint>* positions;

	bool mapped;
	double length;
	double width;
	double cell_size;
	int area_size;

	ros::ServiceServer service;
    ros::NodeHandle node;
};

#endif