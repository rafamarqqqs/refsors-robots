#include "../include/robot_control/occupancy_grid.h"

OccupancyGrid::OccupancyGrid(float length, float width, float cell_size, int area_size, float rep){
	this -> length = length;
	this -> width = width;
	this -> cell_size = cell_size;
	this -> repulsion = rep;
	this -> ready = false;
	this -> area_size = area_size;

	last_x = UNDEFINED;
	last_y = UNDEFINED;

	initAreas();
	initMap();
}

void OccupancyGrid::setRepulsion(float rep){
	repulsion = rep;
}

void OccupancyGrid::initAreas(){
	int i, j;

	areas = (Area **) malloc(sizeof(Area *)*TO_CELLS(length));

	for(i = 0; i < floor(TO_CELLS(length)/area_size); i++){
		
		areas[i] = (Area *) malloc(sizeof(Area) * TO_CELLS(width));

		for(j = 0; j < floor(TO_CELLS(width)/area_size); j++){
			areas[i][j].start.x = (i * area_size) - (length/2);
			areas[i][j].start.y = (j * area_size) - (width/2);

			areas[i][j].end.x = ((i + 1) * area_size) - (length/2);
			areas[i][j].end.y = ((j + 1) * area_size) - (width/2);

			areas[i][j].occupied = false;
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

	for(i = 0; i < TO_CELLS(length); i++)
		free(map[i]);

	free(map);

	for(i = 0; i < floor(TO_CELLS(length)/area_size); i++)
		free(areas[i]);

	free(areas);

}

bool OccupancyGrid::isInTheSameQuadrant(_2DPoint* goal, Area area){
	float x = (area.start.x + area.end.x)/2;
	float y = (area.start.y + area.end.y)/2;
	float x_middle = 0;
	float y_middle = 0;

	if(x > x_middle && goal -> x > x_middle){
		if(y > y_middle && goal -> y > y_middle)
			return true;
		else if(y < y_middle && goal -> y < y_middle)
			return true;
		else
			return false;
	}
	else if(x < x_middle && goal -> x < x_middle){
		if(y > y_middle && goal -> y > y_middle)
			return true;
		else if(y < y_middle && goal -> y < y_middle)
			return true;
		else
			return false;
	}
	else
		return false;
}

bool OccupancyGrid::isFarAway(_2DPoint* goal, Area area){
	int x = (area.start.x + area.end.x)/2;
	int y = (area.start.y + area.end.y)/2;

	float distance = pow(pow(x - goal -> x, 2) + pow(y - goal -> y, 2), 0.5);

	return distance > 10 ? true : false;
}

void OccupancyGrid::getNewGoal(_2DPoint* goal){
	float x = 0;
	float y = 0;
	int y_index;
	int x_index;

	srand(time(NULL));

	x_index = rand() % ((int) floor(TO_CELLS(length)/area_size));
	y_index = rand() % ((int) floor(TO_CELLS(width)/area_size));

	while(areas[x_index][y_index].occupied == true ||
		isFarAway(goal, areas[x_index][y_index])
		 /*isInTheSameQuadrant(goal, areas[x_index][y_index])*/){
		x_index = rand() % ((int) floor(TO_CELLS(length)/area_size));
		y_index = rand() % ((int) floor(TO_CELLS(width)/area_size));
	}

	goal -> x = (areas[x_index][y_index].start.x + areas[x_index][y_index].end.x)/2;
	goal -> y = (areas[x_index][y_index].start.y + areas[x_index][y_index].end.y)/2;

	/*
	for(int i = 0; i < TO_CELLS(length); i++){
		for(int j = 0; j < TO_CELLS(width); j++){
			if(map[i][j] != 0){
				aux.x = ((i - BASE_X) * cell_size) - goal -> x;
				aux.y = ((j - BASE_Y) * cell_size) - goal -> y;

				norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);

				x += aux.x/norm;
				y += aux.y/norm;
			}
		}
	}

	goal -> x = x*0.1;
	goal -> y = y*0.1;
	*/

	remakeOccupiedAreas();
}

void OccupancyGrid::remakeOccupiedAreas(){
	int size;
	int occupied;
	
	for(int _i = 0; _i < floor(TO_CELLS(length)/area_size); _i++){
		for(int _j = 0; _j < floor(TO_CELLS(width)/area_size); _j++){

			occupied = size = 0;

			for(int i = areas[_i][_j].start.x; i < areas[_i][_j].end.x; i++){
				for(int j = areas[_i][_j].start.y; j < areas[_i][_j].end.y; j++){
					if(IS_INSIDE(i, j)){
						size++;

						if(map[i][j] != 0)
							occupied++;
					}
				}
			}

			//ROS_INFO("%3.2f", (float) (1.0*occupied)/(1.0*size));

			if(((float) (1.0*occupied)/(1.0*size)) >= 0.25)
				areas[_i][_j].occupied = true;
		}
	}
}

float OccupancyGrid::OGInfluence(float x, float y){
	int map_x = (int) (TO_CELLS(x) + BASE_X);
	int map_y = (int) (TO_CELLS(y) + BASE_Y);
	int i, j;
	float total = 0, local_total = 0;
	int size = 0, local_size = 0;
	float local_average, average;
	float percentage = 0;

	for(i = 0; i < TO_CELLS(length); i++){
		for(j = 0; j < TO_CELLS(width); j++){
			if(IS_INSIDE(i, j) && map[i][j] != 0){
				total += map[i][j];
				size++;
			}
		}
	}

	if(size != 0)
		average = total/(size*1.0);

	for(i = map_x - 2; i < map_x + 2; i++){
		for(j = map_y - 2; j < map_y + 2; j++){
			if(IS_INSIDE(i, j) && map[i][j] != 0){
				local_total += map[i][j];
				local_size++;
			}
		}
	}

	if(local_size != 0)
		local_average = local_total/(local_size*1.0);

	if(average != 0 && local_average != 0)
		percentage = local_average/average;

	//ROS_INFO("LA: %3.2f GA: %3.2f", local_average, average);

	if(percentage - 0.5 > 0.4)
		return percentage - 0.5;
	else
		return 0;
}

OGVector* OccupancyGrid::calculateTailForce(_2DPoint robot){
	vector<_2DPoint>::iterator it;
	OGVector* ogTail = new OGVector();
	_2DPoint aux;
	float norm;
	ogTail -> x = 0; 
	ogTail -> y = 0; 


	for(it = tail.begin(); it != tail.end(); it++){
		aux.x = (*it).x - robot.x;
		aux.y = (*it).y - robot.y;

		norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);

		ogTail -> x += (aux.x/norm);
		ogTail -> y += (aux.y/norm);

		//ROS_INFO("%d tail %3.2f %3.2f", i, ogTail -> x, ogTail -> y);
	}

	return ogTail;
}

void OccupancyGrid::updatePosition(float x, float y){
	int map_x = (int) (TO_CELLS(x) + BASE_X);
	int map_y = (int) (TO_CELLS(y) + BASE_Y);
	/*
	std::list<_2DPoint>::iterator it;

	it = wall_points -> begin();

	while(it != wall_points -> end()){
		map[(int) (TO_CELLS((*it).x) + BASE_X)][(int) (TO_CELLS((*it).y) + BASE_Y)] += repulsion;
		it++;
	}

	*/


	for(int i = map_x - DISPLACEMENT; i <= map_x + DISPLACEMENT; i++){
		for(int j = map_y - DISPLACEMENT; j <= map_y + DISPLACEMENT; j++){
			if(IS_INSIDE(i, j)){
				map[i][j] += repulsion;
			}
		}
	}
	
	//map[map_x][map_y] += repulsion;

	updateTail(x, y);
}

void OccupancyGrid::updateTail(float x, float y){
	if(tail.size() == 0)
		return;

	int map_x = (int) (TO_CELLS(x) + BASE_X);
	int map_y = (int) (TO_CELLS(y) + BASE_Y);

	if(last_x == UNDEFINED && last_y == UNDEFINED){
		last_x = map_x;
		last_y = map_y;
	}
	else if(last_x == map_x && last_y == map_y)
		return;

	_2DPoint aux;

	if(tail.size() == MAX_TAIL_SIZE)
		tail.erase(tail.begin());
	
	aux.x = x;
	aux.y = y;
	tail.push_back(aux);

	last_x = x;
	last_y = y;
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

	for(int i = 0; i < floor(TO_CELLS(length)/area_size); i++){
		for(int j = 0; j < floor(TO_CELLS(width)/area_size); j++){

		file << "|";
		
		file << std::setfill(' ') << (areas[i][j].occupied == true ? '1' : '0');
		
		counter++;
		
			if(counter % ((int) (TO_CELLS(length)/area_size)) == 0){
				file << "|";
				file << "\n";
			}
		}
	}

	file.close();
}


OGVector* OccupancyGrid::calculateOGVector(_2DPoint robot){
	float x = robot.x;
	float y = robot.y;
	int map_x = (int) (TO_CELLS(x) + BASE_X);
	int map_y = (int) (TO_CELLS(y) + BASE_Y);
	OGVector* ogv;
	int max = 0;
	float total;
	float average;
	int i, j;
	int size;

	ogv = new OGVector();

	ogv -> x = 0;
	ogv -> y = 0;


	// NW
	for(i = 1, j = 1; i < NEARBY; i++, j++){
		if(IS_INSIDE(map_x + i, map_y + j)){
			total += map[map_x + i][map_y + j];
			size++;
		}
	}

	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		ogv -> x = -1;
		ogv -> y = -1;

		max = average;
	}

	size = 0;
	total = 0;


	// N
	for(i = 1; i < NEARBY; i++){
		if(IS_INSIDE(map_x + i, map_y)){
			total += map[map_x + i][map_y];
			size++;
		}
	}
	
	average = total/(1.0*size);

	if(size > 0 && average > max){
		ogv -> x = -1;
		ogv -> y = 0;

		max = average;
	}
	
	size = 0;
	total = 0;

	

	// NE
	for(i = 1, j = 1; i < NEARBY; i++, j++){
		if(IS_INSIDE(map_x + i, map_y - j)){
			total += map[map_x + i][map_y - j];
			size++;
		}
	}

	average = total/(1.0*size);
	
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
	
	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		ogv -> x = 0;
		ogv -> y = 1;

		max = average;
	}	

	size = 0;
	total = 0;

	

	// SE
	for(i = 1, j = 1; i < NEARBY; i++, j++){
		if(IS_INSIDE(map_x - i, map_y - j)){
			total += map[map_x - i][map_y - j];
			size++;
		}
	}
	
	average = total/(1.0*size);
	
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
	
	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		ogv -> x = 1;
		ogv -> y = 0;

		max = average;
	}	

	size = 0;
	total = 0;
	

	// SW
	for(i = 1, j = 1; i < NEARBY; i++, j++){
		if(IS_INSIDE(map_x - i, map_y + j)){
			total += map[map_x - i][map_y + j];
			size++;
		}
	}

	average = total/(1.0*size);
	
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

	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		ogv -> x = 0;
		ogv -> y = -1;
	}	

	return ogv;
}