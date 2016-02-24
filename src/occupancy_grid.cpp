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

	areas = new std::vector<Area>;

	for(int i = 0; i < (int) TO_CELLS(length)/area_size; i++){
		for(int j = 0; j < (int) TO_CELLS(width)/area_size; j++){
			aux.start.x = i * area_size;
			aux.start.y = j * area_size;

			aux.end.x = (i + 1) * area_size;
			aux.end.y = (j + 1) * area_size;

			aux.occupied = false;

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

	while(areas -> at(new_area_index).occupied == true){
		new_area_index = rand() % ((int) areas -> size());		
	}

	goal -> x = (((areas -> at(new_area_index).start.x + areas -> at(new_area_index).end.x)/2) * cell_size) - (length/2);
	goal -> y = (((areas -> at(new_area_index).start.y + areas -> at(new_area_index).end.y)/2) * cell_size) - (width/2);

//	ROS_INFO("got new goal %3.2f %3.2f", goal -> x, goal -> y);
	remakeOccupiedAreas();
}

void OccupancyGrid::remakeOccupiedAreas(){
	std::vector<Area>::iterator it = areas -> begin();
	int size;
	int occupied;

	while(it != areas -> end()){
		occupied = size = 0;

		for(int i = (*it).start.x; i < (*it).end.x; i++){
			for(int j = (*it).start.y; j < (*it).end.y; j++){
				if(IS_INSIDE(i, j)){
					size++;

					if(map[i][j] != 0)
						occupied++;
				}
			}
		}

		ROS_INFO("%3.2f", occupied/size);

		if(occupied/size > 0.6)
			(*it).occupied = true;

		it++;
	}
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
	int map_x = (int) (TO_CELLS(x) + BASE_X);
	int map_y = (int) (TO_CELLS(y) + BASE_Y);

	for(int i = - area_size; i < area_size; i++){
		for(int j = - area_size; j < area_size; j++){
			if(IS_INSIDE(map_x + i, map_y + j)){
				map[map_x + i][map_y + j] += repulsion;
			}
		}
	}
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

	writeAreas(type);
}

void OccupancyGrid::writeAreas(std::string type){
	int counter = 0;
	std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/areas_" + type + ".map");

	std::vector<Area>::iterator it = areas -> begin();

	while(it != areas -> end()){

		file << "|";
		
		file << std::setfill(' ') << ((*it).occupied == true ? '1' : '0');
		
		it++;
		counter++;
		
		if(counter % ((int) (TO_CELLS(length)/area_size)) == 0){
			file << "|";
			file << "\n";
		}
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