#include <list>
#include <ros/ros.h>
#include <stdlib.h> 
#include "resources.h"
#include "topics.h"
#include "robot_control/addToMap.h"
#include "robot_control/getPositions.h"
#include "robot_control/defineGlobalPath.h"
#include <actionlib/server/simple_action_server.h>
#include <fstream>
#include <stdlib.h> 
#include <cmath>

#ifndef _KNOWLEDGE_H_
#define _KNOWLEDGE_H_

#define FULL 100
#define UNKNOWN 50
#define EMPTY 0

#define EMPTY_RANGE 30
#define FULL_RANGE 70

#define DISCRETE_ERROR 0.1

#define MAX_RANGE 8

#define INSIDE(x, y) (x + (this -> length/2) >= 0 && y + (this -> width/2) >= 0 && \
				   x < this -> length/2 && y < this -> width/2)

#define TO_CELLS(v) (floor(v/this -> cell_size))

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

class Knowledge {
public:
	Knowledge(ros::NodeHandle node, double length, double width, double cell_size);
    ~Knowledge();

    bool addToMap(robot_control::addToMap::Request& req, robot_control::addToMap::Response& res);
    bool getPositions(robot_control::getPositions::Request& req, robot_control::getPositions::Response& res);

    void writeMap();

private:
	void initMaps();

	char** map;
	unsigned short** map_scans;

	int ids;
	std::vector<_2DPoint>* positions;

	bool mapped;
	double length;
	double width;
	double cell_size;

	ros::ServiceServer service;
    ros::NodeHandle node;
};

#endif