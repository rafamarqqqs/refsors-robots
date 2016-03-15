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

using namespace std;

#ifndef _OCCUPANCY_GRID_H_
#define _OCCUPANCY_GRID_H_

#define NEARBY 4
#define DISPLACEMENT 1

#define MAX_TAIL_SIZE 0

#define UNDEFINED 1000

#define IS_INSIDE(x, y) (x >= 0 && y >= 0 && x < TO_CELLS(length) && y < TO_CELLS(width))

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

#define TO_CELLS(v) (((int)floor(v/this -> cell_size)))

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

typedef enum lvl {
	NEW,
	RECENT,
	OLD
} Level;

typedef struct {
	float x, y;
	Level level;
} OccupancyTail;

class OccupancyGrid{
public:
	OccupancyGrid(float length, float width, float cell_size, int area_size, float rep);
	~OccupancyGrid();

	OGVector* calculateOGVector(_2DPoint robot);
	void getNewGoal(_2DPoint *goal);
	void updatePosition(float x, float y);
	void writeMap(std::string type);
	float OGInfluence(float x, float y);
	void setRepulsion(float rep);
	OGVector* calculateTailForce(_2DPoint robot);
	bool isFarAway(_2DPoint* goal, Area area);

private:
	void updateTail(float x, float y);
	void writeAreas(std::string type);
	bool isInTheSameQuadrant(_2DPoint* goal, Area area);
	void initMap();
	void initAreas();
	void remakeOccupiedAreas();

	float** map;

	Area** areas;

	int last_x, last_y;

	vector<_2DPoint> tail;

	float area_size;
	float length;
	float width;
	bool ready;
	float cell_size;
	float repulsion;

};

#endif