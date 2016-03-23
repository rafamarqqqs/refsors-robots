#include <fstream>
#include <ros/ros.h>
#include <cmath>
#include <ctime>
#include <stdlib.h>
#include <list>
#include <iomanip>
#include "resources.h"
#include <algorithm>
#include <vector>
#include "robot_control/getNewGoal.h"
#include <limits.h>

using namespace std;

#ifndef _OCCUPANCY_GRID_H_
#define _OCCUPANCY_GRID_H_

#define MAX_TAIL_SIZE 1

#define EMPTY 0
#define OCCUPIED 1

#define UNDEFINED 1000

#define IS_INSIDE(x, y) (x >= 0 && y >= 0 && x < TO_CELLS(length) && y < TO_CELLS(width))

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

#define TO_CELLS(v) (((int)floor(v/this -> cell_size)))

typedef struct area {
	_2DPoint start;
	_2DPoint end;
	_2DPoint destiny;
	bool occupied;
} Area;

typedef enum lvl {
	NEW,
	RECENT,
	OLD
} Level;

typedef struct {
	double x, y;
	Level level;
} OccupancyTail;

class OccupancyGrid{
public:
	OccupancyGrid(ros::NodeHandle node, double length, double width, double cell_size, int area_size,
		double rep, int nearby, int displacement);
	~OccupancyGrid();

	void calculateOGVector(_2DPoint* robot, double* x, double* y);
	void updatePosition(double x, double y);
	void writeMap(std::string type);
	void getNewGoal(_2DPoint* goal);
	double OGInfluence(double x, double y);
	void calculateTailForce(_2DPoint* robot_pose, double* x, double* y);

private:
	void updateTail(double x, double y);
	void initMap();
	void initAreas();
	void writeAreas(std::string type);
	vector<uint8_t> remakeOccupiedAreas();
	
	int** map;

	Area** areas;

	int last_x, last_y;

	vector<_2DPoint> tail;

	int nearby;
	int displacement;
	double length;
	double width;
	double cell_size;
	double repulsion;
	int area_size;
	
	ros::NodeHandle node;
	ros::ServiceClient client;
};

#endif