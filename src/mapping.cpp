#include "../include/robot_control/mapping.h"

Mapper::Mapper(ros::NodeHandle n, float length, float width, float cell_size){
	int i, j;

	this -> node = n;
	this -> length = length;
	this -> width = width;
	this -> cell_size = cell_size;

	//this -> map = new std::vector<std::vector<CellValue> >(length/this -> cell_size);
	this -> map = new std::vector<std::vector<CellValue>* >(length/this -> cell_size);
	
	for(i = 0; i < length/this -> cell_size; i++){
		this -> map -> at(i) = new std::vector<CellValue>(width/this -> cell_size);
		
		for(j = 0; j < width/this -> cell_size; j++)
			this -> map -> at(i) -> at(j) = EMPTY;
	}

	laser_sub = node.subscribe("/larger_robot/base_scan/scan", 1, &Mapper::handleLaser, this);
}

Mapper::~Mapper(){
	delete this -> map;
}

void Mapper::handleLaser(const sensor_msgs::LaserScan::ConstPtr &data){
	int i;
	float x, y;
	_2DPoint aux;

	//for(i = 0; i < (int) floor((data -> angle_max*2)/data -> angle_increment); i++){
	//	aux
	//}


}

void Mapper::createMap(){

}

void Mapper::addToMap(_2DPoint point, CellValue value){
	int x = 0;
	int y = 0;

	if(fmod(point.x, this -> cell_size) <= this -> cell_size/4 &&
		(int) floor((point.x/this -> cell_size) + BASE_X))
		x = 1;
	if(fmod(point.y, this -> cell_size) <= this -> cell_size/4 &&
	   (int) floor((point.y/this -> cell_size) + BASE_Y) > 0)
		y = 1;

	this -> map -> at((int) floor((point.x/this -> cell_size) - x + BASE_X)) 
				-> at((int) floor((point.y/this -> cell_size) - y + BASE_Y)) = value;
}



std::vector<std::vector<CellValue>* >* Mapper::getMap(){
	return this -> map;
}

void Mapper::writeMap(){
	int i, j;

	std::ofstream file("containers.map");

	for(i = 0; i < this -> map -> size(); i++){
		for(j = 0; j < this -> map -> at(i) -> size(); j++){
			file.put(this -> map -> at(i) -> at(j));
		}

		file.put('\n');
	}
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "mapping");

    ros::NodeHandle node;

    Mapper *mapper = new Mapper(node, 40.0, 40.0, 0.5);

    ros::spinOnce();

    ROS_INFO("Mapper started.");

    mapper -> createMap();

    delete mapper;
}