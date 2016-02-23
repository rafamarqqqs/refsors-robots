#include "../include/robot_control/occupancy_grid.h"

OccupancyGrid::OccupancyGrid(float length, float width, float cell_size, int area_size, float rep){
	this -> length = length;
	this -> width = width;
	this -> cell_size = cell_size;
	this -> repulsion = rep;
	this -> ready = false;
	this -> area_size = area_size;

	initAreas();
	initMap();
}

void OccupancyGrid::initAreas(){
	Area aux;

	areas = new std::vector<Area*>();

	for(int i = 0; i < (int) TO_CELLS(length); i++){
		for(int j = 0; j < (int) TO_CELLS(width); j++){
			aux = new Area();

			aux -> start.x = j * area_size;
			aux -> start.y = i * area_size;

			aux -> end.x = (j + 1) * area_size;
			aux -> end.y = (i + 1) * area_size;

			aux -> occupied = false;

			areas -> push_back(aux);
		}
	}
}

void OccupancyGrid::initMap(){
	int i, j;

	map = (float **) malloc(sizeof(float *)*TO_CELLS(length));

	for(i = 0; i < TO_CELLS(length); i++){
		map[i] = (float *) malloc(sizeof(float) * TO_CELLS(width));
		memset(map[i], 0, TO_CELLS(width)*sizeof(float));
	}
}

OccupancyGrid::~OccupancyGrid(){
	int i;

	for(i = 1; i < TO_CELLS(length); i++)
		free(map[i]);

	free(map);

	delete areas;
}

void OccupancyGrid::getNewGoal(_2DPoint* goal){
	srand(time(NULL));
	int new_area_index;

	new_area_index = rand() % ((int) areas -> size());

	while(areas[new_area_index] -> occupied == true){
		new_area_index = rand() % ((int) areas -> size());		
	}

	goal -> x = (areas[new_area_index] -> start.x + areas[new_area_index] -> end.x)/2;
	goal -> y = (areas[new_area_index] -> start.y + areas[new_area_index] -> end.y)/2;
}

bool OccupancyGrid::OGReady(){
	if(ready == true)
		return true;

	int i, j;
	float total;

	for(i = 0; i < TO_CELLS(length); i++){
		for(j = 0; j < TO_CELLS(width); j++){
			if(map[i][j] != 0){
				total += 1.0;
			}
		}
	}

	//ROS_INFO("%f %f %f > 0.005",total, TO_CELLS(length)*TO_CELLS(width)*1.0, total/(TO_CELLS(length)*TO_CELLS(width)*1.0));

	if((1.0*total)/(TO_CELLS(length)*TO_CELLS(width)*1.0) > 0.005){
		this -> ready = true;
		return true;
	}

	return false;
}

void OccupancyGrid::updatePosition(float x, float y, float yaw){
	int map_x;
	int map_y;
	float tail_x, tail_y;
	float i;

	for(i = (-1)*TAIL_ANGLE; i <= TAIL_ANGLE; i+= TAIL_ANGLE){

		float angle = Resources::angleSum(yaw, i);

		tail_x = x - (cos(angle) * TAIL_SIZE);
		tail_y = y - (sin(angle) * TAIL_SIZE);

		map_x = (int) floor(TO_CELLS(tail_x) + BASE_X);
		map_y = (int) floor(TO_CELLS(tail_y) + BASE_Y);

		if(IS_INSIDE(map_x, map_y))
			map[map_x][map_y] += repulsion;
	}
	/*

	for(i = (-2)*TAIL_ANGLE; i <= 2*TAIL_ANGLE; i+= TAIL_ANGLE){

		float angle = Resources::angleSum(yaw, i);

		tail_x = x + (cos(angle) * TAIL_SIZE);
		tail_y = y + (sin(angle) * TAIL_SIZE);

		map_x = (int) floor(TO_CELLS(tail_x) + BASE_X);
		map_y = (int) floor(TO_CELLS(tail_y) + BASE_Y);

		if(IS_INSIDE(map_x, map_y))
			map[map_x][map_y] += (repulsion/3);
	}

	//just for checking
	map_x = (int) floor(TO_CELLS(x) + BASE_X);
	map_y = (int) floor(TO_CELLS(y) + BASE_Y);

	map[map_x][map_y] += repulsion;
	*/
}

void OccupancyGrid::writeMap(std::string type){
	int i, j;

	std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/occupancy_grid_" + type + ".map");

	for(i = 0; i < TO_CELLS(length); i++){
		for(j = 0; j < TO_CELLS(width); j++){
			file << " | ";
			if(map[i][j] == 0)
				file << "    ";
			else
				file << std::setfill(' ') << std::setw(4) << this -> map[i][j];
		}
		file.put('|');
		file.put('\n');
		file.put('\n');
	}
	file.close();
}

OGVector* OccupancyGrid::calculateOGVector(_2DPoint robot){
	float x = robot.x;
	float y = robot.y;
	int map_x = TO_CELLS(x) + BASE_X;
	int map_y = TO_CELLS(y) + BASE_Y;
	OGVector* ogv;
	int max = 30;
	float total;
	float average;
	int i, j;
	int size;

	ogv = new OGVector();

	ogv -> x = 0;
	ogv -> y = 0;


	// N
	for(i = 1; i < NEARBY; i++){
		if(IS_INSIDE(map_x + i, map_y)){
			total += map[map_x + i][map_y];
			size++;
		}
	}
	
	average = total/size;

	if(size > 0 && average > max){
		ogv -> x = -1;
		ogv -> y = 0;

		max = average;
	}
	
	size = 0;
	total = 0;

	

	// NE
	for(i = 1; i < NEARBY; i++){
		for(j = 1; j < NEARBY; j++){
			if(IS_INSIDE(map_x + i, map_y - j)){
				total += map[map_x + i][map_y - j];
				size++;
			}
		}
	}

	average = total/size;
	
	if(size > 0 && average > max){
		ogv -> x = -1;
		ogv -> y = 1;

		max = average;
	}

	size = 0;
	total = 0;

	

	// E
	for(i = 1; i < NEARBY; i++){
		if(IS_INSIDE(map_x, map_y - i)){
			total += map[map_x][map_y - i];
			size++;
		}
	}
	
	average = total/size;
	
	if(size > 0 && average > max){
		ogv -> x = 0;
		ogv -> y = 1;

		max = average;
	}	

	size = 0;
	total = 0;

	

	// SE
	for(i = 1; i < NEARBY; i++){
		for(j = 1; j < NEARBY; j++){
			if(IS_INSIDE(map_x - i, map_y - j)){
				total += map[map_x - i][map_y - j];
				size++;
			}
		}
	}
	
	average = total/size;
	
	if(size > 0 && average > max){
		ogv -> x = 1;
		ogv -> y = 1;

		max = average;
	}	

	size = 0;
	total = 0;
	

	// S
	for(i = 1; i < NEARBY; i++){
		if(IS_INSIDE(map_x - i, map_y)){
			total += map[map_x - i][map_y];
			size++;
		}
	}
	
	average = total/size;
	
	if(size > 0 && average > max){
		ogv -> x = 1;
		ogv -> y = 0;

		max = average;
	}	

	size = 0;
	total = 0;
	

	// SW
	for(i = 1; i < NEARBY; i++){
		for(j = 1; j < NEARBY; j++){
			if(IS_INSIDE(map_x - i, map_y + j)){
				total += map[map_x - i][map_y + j];
				size++;
			}
		}
	}

	average = total/size;
	
	if(size > 0 && average > max){
		ogv -> x = 1;
		ogv -> y = -1;

		max = average;
	}	

	size = 0;
	total = 0;
	

	// W
	for(i = 1; i < NEARBY; i++){
		if(IS_INSIDE(map_x, map_y + i)){
			total += map[map_x][map_y + i];
			size++;
		}
	}

	average = total/size;
	
	if(size > 0 && average > max){
		ogv -> x = 0;
		ogv -> y = -1;

		max = average;
	}	

	size = 0;
	total = 0;
	

	// NW
	for(i = 1; i < NEARBY; i++){
		for(j = 1; j < NEARBY; j++){
			if(IS_INSIDE(map_x + i, map_y + j)){
				total += map[map_x + i][map_y + j];
				size++;
			}
		}
	}

	average = total/size;
	
	if(size > 0 && average > max){
		ogv -> x = -1;
		ogv -> y = -1;
	}

	return ogv;
}