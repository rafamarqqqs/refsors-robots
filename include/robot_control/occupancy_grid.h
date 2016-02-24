#include <fstream>
#include <ros/ros.h>
#include <cmath>
#include <ctime>
#include <stdlib.h>
#include <iomanip>
#include "resources.h"
#include <algorithm>

#ifndef _OCCUPANCY_GRID_H_
#define _OCCUPANCY_GRID_H_

#define DISPLACEMENT (M_PI/10)

#define NEARBY 4
#define TAIL_SIZE 0.6
#define TAIL_ANGLE (M_PI/4)

#define IS_INSIDE(x, y) (x >= 0 && y >= 0 && x < TO_CELLS(length) && y < TO_CELLS(width))

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

#define TO_CELLS(v) (((int)(v/this -> cell_size)))

typedef struct OccupancyGridVector {
	float x;
	float y;
} OGVector;

typedef struct map_point{
	int x;
	int y;
} MapPoint;

typedef struct area {
	_2DPoint start;
	_2DPoint end;
	bool occupied;
} Area;

class OccupancyGrid{
public:
	OccupancyGrid(float length, float width, float cell_size, int area_size, float rep);
	~OccupancyGrid();

	OGVector* calculateOGVector(_2DPoint robot);
	void getNewGoal(_2DPoint *goal);
	void updatePosition(float x, float y, float yaw);
	void writeMap(std::string type);
	void writeAreas(std::string type);
	bool OGReady();

private:
	void initMap();
	void initAreas();
	void remakeOccupiedAreas();

	float** map;

	std::vector<Area>* areas;

	float area_size;
	float length;
	float width;
	bool ready;
	float cell_size;
	float repulsion;

};

#endif