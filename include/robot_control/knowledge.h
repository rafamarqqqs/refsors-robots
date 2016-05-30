#include <list>
#include <ros/ros.h>
#include <stdlib.h> 
#include <limits.h>
#include "resources.h"
#include "topics.h"
#include "robot_control/addToMap.h"
#include "robot_control/getMap.h"
#include "robot_control/getPositions.h"
#include "robot_control/getNewGoal.h"
#include <actionlib/server/simple_action_server.h>
#include <fstream>
#include <stdlib.h> 
#include <cmath>
#include <vector>
#include <mutex>

using namespace std;

#ifndef _KNOWLEDGE_H_
#define _KNOWLEDGE_H_

class Knowledge {
public:
	Knowledge(ros::NodeHandle node, double length, double width, double cell_size, double area_size);
    ~Knowledge();

    bool addToMap(robot_control::addToMap::Request& req, robot_control::addToMap::Response& res);
    bool getMap(robot_control::getMap::Request& req, robot_control::getMap::Response& res);
    bool getPositions(robot_control::getPositions::Request& req, robot_control::getPositions::Response& res);
    bool getNewGoal(robot_control::getNewGoal::Request& req, robot_control::getNewGoal::Response& res);

private:

	mutex goal_mtx;
	mutex pose_mtx;
	mutex map_mtx;

    void writeMap();
    inline void getLeastExploredQuadrant(int* x_displacement, int* y_displacement, int length, int width);
    inline bool isInTheSameQuadrant(int x, int y);
	inline bool isCurrentGoal(int x, int y);
    inline void writeAreas();
	void initMaps();
	void initAreas();

	int** areas;

	vector<Goal>* goals;

	char** map;
	int** map_scans;

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