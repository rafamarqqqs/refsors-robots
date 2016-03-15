#include <list>
#include <ros/ros.h>
#include <stdlib.h> 
#include "resources.h"
#include "topics.h"
#include "robot_control/addToMap.h"
#include <actionlib/server/simple_action_server.h>
#include <fstream>
#include <stdlib.h> 
#include <cmath>

#ifndef _KNOWLEDGE_H_
#define _KNOWLEDGE_H_

#define FULL '#'
#define UNKNOWN ':'
#define EMPTY ' '
#define ME 'x'

#define DISCRETE_ERROR 0.1

#define MAX_RANGE 8

#define INSIDE(p) (p.x + (this -> length/2) >= 0 && p.y + (this -> width/2) >= 0 && \
				   p.x < this -> length/2 && p.y < this -> width/2)

#define TO_CELLS(v) (((int)floor(v/this -> cell_size)))

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

#define COMPARE(start, goal, pos) (start > goal ? (pos > goal) : (pos < goal))

class Knowledge {
public:
	Knowledge(ros::NodeHandle node, float length, float width, float cell_size);
    ~Knowledge();

    bool addToMap(robot_control::addToMap::Request& req, robot_control::addToMap::Response& res);

    void writeMap();

private:
	bool checkError(int x, int y);
	void initMap();

	char** map;

	float length;
	float width;
	float cell_size;

	ros::ServiceServer service;
    ros::NodeHandle node;

};

#endif