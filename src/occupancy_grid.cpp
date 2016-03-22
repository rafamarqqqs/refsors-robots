#include "../include/robot_control/occupancy_grid.h"

OccupancyGrid::OccupancyGrid(double len, double wid, double cell_s, int area_s, double rep,
													int near, int disp):
	length(len), width(wid), cell_size(cell_s), area_size(area_s), repulsion(rep), nearby(near),
	displacement(disp),	last_x(UNDEFINED), last_y(UNDEFINED) {
	
	initMap();
	initAreas();
}

void OccupancyGrid::initMap(){
	int i;

	map = (int **) malloc(sizeof(int *)*TO_CELLS(length));

	for(i = 0; i < TO_CELLS(length); i++){
		map[i] = (int *) malloc(sizeof(int) * TO_CELLS(width));
		memset(map[i], 0, TO_CELLS(width)*sizeof(int));
	}
}


void OccupancyGrid::initAreas(){
	int i, j;

	areas = (Area **) malloc(sizeof(Area *) * floor(length/area_size));

	for(i = 0; i < floor(length/area_size); i++){
		
		areas[i] = (Area *) malloc(sizeof(Area) * floor(width/area_size));

		for(j = 0; j < floor(width/area_size); j++){
			areas[i][j].start.x = (j * area_size) - (width/2);
			areas[i][j].start.y = (i * area_size) - (length/2);

			areas[i][j].end.x = ((j + 1) * area_size) - (width/2);
			areas[i][j].end.y = ((i + 1) * area_size) - (length/2);

			areas[i][j].destiny.x = (areas[i][j].start.x + areas[i][j].end.x)/2;
			areas[i][j].destiny.y = (areas[i][j].start.y + areas[i][j].end.y)/2;

			areas[i][j].occupied = false;
		}
	}
}


OccupancyGrid::~OccupancyGrid(){
	int i;

	for(i = 0; i < TO_CELLS(length); i++)
		free(map[i]);

	free(map);

	for(i = 0; i < floor(length/area_size); i++)
		free(areas[i]);

	free(areas);
}

/*
bool OccupancyGrid::isInTheSameQuadrant(_2DPoint* goal, Area area){
	double x = (area.start.x + area.end.x)/2;
	double y = (area.start.y + area.end.y)/2;
	double x_middle = 0;
	double y_middle = 0;

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
}*/

bool OccupancyGrid::isFarAway(_2DPoint* goal, Area area){
	return pow(pow(area.destiny.x - goal -> x, 2) + pow(area.destiny.y - goal -> y, 2), 0.5) > 10 
	? true : false;
}

void OccupancyGrid::getNewGoal(_2DPoint* goal){
	double x = 0;
	double y = 0;
	int y_index;
	int x_index;

	srand(time(NULL));

	x_index = rand() % ((int) floor(length/area_size));
	y_index = rand() % ((int) floor(width/area_size));

	//ROS_INFO("%d %d", x_index, y_index);
	
	if(goal -> x != INT_MAX && goal -> y != INT_MAX){
		while(areas[x_index][y_index].occupied == true /*||
			isFarAway(goal, areas[x_index][y_index])*/
			 /*isInTheSameQuadrant(goal, areas[x_index][y_index])*/){
			x_index = rand() % ((int) floor(length/area_size));
			y_index = rand() % ((int) floor(width/area_size));
		}
	}

	//ROS_INFO("got new goal");

	goal -> x = areas[x_index][y_index].destiny.x;
	goal -> y = areas[x_index][y_index].destiny.y * (-1.0);

	remakeOccupiedAreas();
	//ROS_INFO("remade areass");
}

void OccupancyGrid::remakeOccupiedAreas(){
	int size;
	int occupied;
	int map_x, map_y;
	
	for(int _i = 0; _i < floor(length/area_size); _i++){
		for(int _j = 0; _j < floor(width/area_size); _j++){

			occupied = size = 0;

			for(float i = areas[_i][_j].start.x; i < areas[_i][_j].end.x; i += cell_size){
				for(float j = areas[_i][_j].start.y; j < areas[_i][_j].end.y; j += cell_size){
					
					map_x = (int)floor(TO_CELLS(i)) + BASE_X;
					map_y = (int)floor(TO_CELLS(j)) + BASE_Y;

					if(IS_INSIDE(map_x, map_y)){
						size++;

						if(map[map_x][map_y] != 0)
							occupied++;
					}
				}
			}

			if(size != 0 && ((double) (1.0*occupied)/(1.0*size)) >= 0.25)
				areas[_i][_j].occupied = true;
		}
	}
}

double OccupancyGrid::OGInfluence(double x, double y){
	int map_x = (int) (TO_CELLS(x) + BASE_X);
	int map_y = (int) (TO_CELLS(y) + BASE_Y);
	int i, j;
	double total = 0, local_total = 0;
	int size = 0, local_size = 0;
	double local_average, average;
	double percentage = 0;

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

void OccupancyGrid::calculateTailForce(_2DPoint* robot_pose, double* x, double* y){
	vector<_2DPoint>::iterator it;
	_2DPoint aux;
	double norm;
	*x = 0; 
	*y = 0; 

	for(it = tail.begin(); it != tail.end(); it++){
		aux.x = (*it).x - robot_pose -> x;
		aux.y = (*it).y - robot_pose -> y;

		norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);

		*x += (aux.x/norm);
		*y += (aux.y/norm);
	}
}

void OccupancyGrid::updatePosition(double x, double y){
	int map_x = (int) (TO_CELLS(x) + BASE_X);
	int map_y = (int) (TO_CELLS(y) + BASE_Y);

	for(int i = map_x - displacement; i <= map_x + displacement; i++){
		for(int j = map_y - displacement; j <= map_y + displacement; j++){
			if(IS_INSIDE(i, j)){
				map[i][j] += repulsion;
			}
		}
	}

	updateTail(x, y);
}

void OccupancyGrid::updateTail(double x, double y){
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

	//writeAreas(type);
}

void OccupancyGrid::writeAreas(std::string type){
	int counter = 0;
	std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/areas_" + type + ".map");

	for(int i = 0; i < floor(length/area_size); i++){
		for(int j = 0; j < floor(width/area_size); j++){

		file << "|";
		
		file << std::setfill(' ') << (areas[i][j].occupied == true ? '1' : '0');
		
		}

		file << "|";
		file << "\n";
	}

	for(int i = 0; i < floor(length/area_size); i++){
		for(int j = 0; j < floor(width/area_size); j++){
		
		file << "|";
		
		file << areas[i][j].start.x << areas[i][j].start.y << areas[i][j].end.x << areas[i][j].end.y
			<< "(" << areas[i][j].destiny.x << areas[i][j].destiny.y << ")";
		
		}
	
		file << "|";
		file << "\n";
	}

	file << "\n";
	file << floor(length/area_size) << " " << floor(width/area_size) << endl;
	file.close();
}


void OccupancyGrid::calculateOGVector(_2DPoint* robot_pose, double* x, double* y){
	int map_x = (int) (TO_CELLS(robot_pose -> x) + BASE_X);
	int map_y = (int) (TO_CELLS(robot_pose -> y) + BASE_Y);
	int max = 0;
	double total;
	double average;
	int i, j;
	int size;

	*x = 0;
	*y = 0;

	// NW
	for(i = 1, j = 1; i < nearby; i++, j++){
		if(IS_INSIDE(map_x + i, map_y + j)){
			total += map[map_x + i][map_y + j];
			size++;
		}
	}

	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		*x = -1;
		*y = -1;

		max = average;
	}

	size = 0;
	total = 0;


	// N
	for(i = 1; i < nearby; i++){
		if(IS_INSIDE(map_x + i, map_y)){
			total += map[map_x + i][map_y];
			size++;
		}
	}
	
	average = total/(1.0*size);

	if(size > 0 && average > max){
		*x = -1;
		*y = 0;

		max = average;
	}
	
	size = 0;
	total = 0;

	

	// NE
	for(i = 1, j = 1; i < nearby; i++, j++){
		if(IS_INSIDE(map_x + i, map_y - j)){
			total += map[map_x + i][map_y - j];
			size++;
		}
	}

	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		*x = -1;
		*y = 1;

		max = average;
	}

	size = 0;
	total = 0;

	

	// E
	for(i = 1; i < nearby; i++){
		if(IS_INSIDE(map_x, map_y - i)){
			total += map[map_x][map_y - i];
			size++;
		}
	}
	
	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		*x = 0;
		*y = 1;

		max = average;
	}	

	size = 0;
	total = 0;

	

	// SE
	for(i = 1, j = 1; i < nearby; i++, j++){
		if(IS_INSIDE(map_x - i, map_y - j)){
			total += map[map_x - i][map_y - j];
			size++;
		}
	}
	
	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		*x = 1;
		*y = 1;

		max = average;
	}	

	size = 0;
	total = 0;
	

	// S
	for(i = 1; i < nearby; i++){
		if(IS_INSIDE(map_x - i, map_y)){
			total += map[map_x - i][map_y];
			size++;
		}
	}
	
	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		*x = 1;
		*y = 0;

		max = average;
	}	

	size = 0;
	total = 0;
	

	// SW
	for(i = 1, j = 1; i < nearby; i++, j++){
		if(IS_INSIDE(map_x - i, map_y + j)){
			total += map[map_x - i][map_y + j];
			size++;
		}
	}

	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		*x = 1;
		*y = -1;

		max = average;
	}	

	size = 0;
	total = 0;
	

	// W
	for(i = 1; i < nearby; i++){
		if(IS_INSIDE(map_x, map_y + i)){
			total += map[map_x][map_y + i];
			size++;
		}
	}

	average = total/(1.0*size);
	
	if(size > 0 && average > max){
		*x = 0;
		*y = -1;
	}	
}