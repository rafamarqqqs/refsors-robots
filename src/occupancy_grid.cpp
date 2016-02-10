#include "../include/robot_control/occupancy_grid.h"

OccupancyGrid::OccupancyGrid(float length, float width, float cell_size, float rep){
	this -> length = length;
	this -> width = width;
	this -> cell_size = cell_size;
	this -> repulsion = rep;

	initMap();
}

void OccupancyGrid::initMap(){
	int i, j;

	map = (int **) malloc(sizeof(int *)*TO_CELLS(length));

	for(i = 0; i < TO_CELLS(length); i++){
		map[i] = (int *) malloc(sizeof(int) * TO_CELLS(width));
		for(j = 0; j < TO_CELLS(width); j++)
			map[i][j] = 0.0;
		//memset(&(map[i]), TO_CELLS(width)*sizeof(int), 0);
	}
		//map[i] = calloc(TO_CELLS(width), sizeof(int));
}

OccupancyGrid::~OccupancyGrid(){
	int i;

	for(i = 0; i < TO_CELLS(length); i++)
		free(map[i]);

	free(map);
}

void OccupancyGrid::updatePosition(float x, float y){
	int map_x = TO_CELLS(x) + BASE_X;
	int map_y = TO_CELLS(y) + BASE_Y;
	int i, j;

	for(i = map_x - NEARBY; i <= map_x + NEARBY; i++){
		for(j = map_y - NEARBY; j <= map_y + NEARBY; j++){
			if(IS_INSIDE(i, j))
				map[i][j] += repulsion;
		}
	}

	//map[map_x][map_y] += 4;
}

void OccupancyGrid::writeMap(){
	int i, j;

	std::ofstream file("occupancy_grid.map");

	for(i = 0; i < TO_CELLS(length); i++){
		for(j = 0; j < TO_CELLS(width); j++){	
			file << this -> map[i][j];
			file << " ";
		}

		file.put('\n');
	}

	file.close();
}

OGVector OccupancyGrid::calculateOGVector(float x, float y){
	int map_x = TO_CELLS(x) + BASE_X;
	int map_y = TO_CELLS(y) + BASE_Y;
	OGVector ogv;
	int max = INT_MIN;
	float total;
	float average;
	int i;
	int size;

	ogv.x = 0;
	ogv.y = 0;

	

	// N
	for(i = 1; i <= NEARBY; i++){
		if(IS_INSIDE(map_x + i, map_y)){
			total += map[map_x + i][map_y];
			size++;
		}
	}
	
	average = total/size;

	if(size > 0 && average > max){
		ogv.x = 1;
		ogv.y = 0;

		max = average;
	}
	
	size = total = 0;

	

	// NE
	for(i = 1; i <= NEARBY; i++){
		if(IS_INSIDE(map_x + i, map_y - i)){
			total += map[map_x + i][map_y - i];
			size++;
		}
	}

	average = total/size;
	
	if(size > 0 && average > max){
		ogv.x = 1;
		ogv.y = -1;

		max = average;
	}

	size = total = 0;

	

	// E
	for(i = 1; i <= NEARBY; i++){
		if(IS_INSIDE(map_x, map_y - i)){
			total += map[map_x][map_y - i];
			size++;
		}
	}
	
	average = total/size;
	
	if(size > 0 && average > max){
		ogv.x = 0;
		ogv.y = -1;

		max = average;
		total = 0;
	}	

	size = total = 0;

	

	// SE
	for(i = 1; i <= NEARBY; i++){
		if(IS_INSIDE(map_x - i, map_y - i)){
			total += map[map_x - i][map_y - i];
			size++;
		}
	}
	
	average = total/size;
	
	if(size > 0 && average > max){
		ogv.x = -1;
		ogv.y = -1;

		max = average;
	}	

	size = total = 0;
	

	// S
	for(i = 1; i <= NEARBY; i++){
		if(IS_INSIDE(map_x - i, map_y)){
			total += map[map_x - i][map_y];
			size++;
		}
	}
	
	average = total/size;
	
	if(size > 0 && average > max){
		ogv.x = -1;
		ogv.y = 0;

		max = average;
		total = 0;
	}	

	size = total = 0;
	

	// SW
	for(i = 1; i <= NEARBY; i++){
		if(IS_INSIDE(map_x - i, map_y + i)){
			total += map[map_x - i][map_y + i];
			size++;
		}
	}

	average = total/size;
	
	if(size > 0 && average > max){
		ogv.x = -1;
		ogv.y = 1;

		max = average;
	}	

	size = total = 0;
	

	// W
	for(i = 1; i <= NEARBY; i++){
		if(IS_INSIDE(map_x, map_y + i)){
			total += map[map_x][map_y + i];
			size++;
		}
	}

	average = total/size;
	
	if(size > 0 && average > max){
		ogv.x = 0;
		ogv.y = 1;

		max = average;
	}	

	size = total = 0;
	

	// NW
	for(i = 1; i <= NEARBY; i++){
		if(IS_INSIDE(map_x + i, map_y + i)){
			total += map[map_x + i][map_y + i];
			size++;
		}
	}

	average = total/size;
	
	if(size > 0 && average > max){
		ogv.x = 1;
		ogv.y = 1;
	}	


	return ogv;
}