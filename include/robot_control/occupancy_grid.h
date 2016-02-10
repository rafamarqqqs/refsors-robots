#include <fstream>
#include <ros/ros.h>
#include <cmath>
#include <stdlib.h>

#ifndef _OCCUPANCY_GRID_H_
#define _OCCUPANCY_GRID_H_

#define NEARBY 2

#define IS_INSIDE(x, y) (x >= 0 && y >= 0 && x < TO_CELLS(length) && y < TO_CELLS(width))

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

#define TO_CELLS(v) (((int)(v/this -> cell_size)))

typedef struct OccupancyGridVector {
	float x;
	float y;
} OGVector;

class OccupancyGrid{
public:
	OccupancyGrid(float length, float width, float cell_size, float rep);
	~OccupancyGrid();

	OGVector calculateOGVector(float x, float y);
	void updatePosition(float x, float y);
	void writeMap();

private:
	void initMap();

	int** map;

	float length;
	float width;
	float cell_size;
	float repulsion;

};

#endif