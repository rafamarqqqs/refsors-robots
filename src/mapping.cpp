#include "../include/robot_control/mapping.h"

Mapper::Mapper(ros::NodeHandle n, float length, float width, float cell_size){
	int i, j;

	this -> node = n;
	this -> length = length;
	this -> width = width;
	this -> cell_size = cell_size;

	this -> robot = (Robot *) malloc(sizeof(Robot));
	//this -> robotAux = (Robot *) malloc(sizeof(Robot));

	this -> laser = new Laser();

	this -> map = new std::vector<CellValue>(TO_CELLS(length) * TO_CELLS(width));
	
	for(i = 0; i < TO_CELLS(length) * TO_CELLS(width); i++){
			this -> map -> at(i) = EMPTY;
	}

	laser_sub = node.subscribe("/larger_robot/base_scan/scan", 1, &Laser::handleSubscription, this -> laser);
	//pose_sub = node.subscribe("/larger_robot/pose", 1, &Mapper::handlePose, this);
	gazebo_pose_sub = node.subscribe("/gazebo/model_states", 1, &Mapper::handleGazeboModelState, this);

}

Mapper::~Mapper(){
	delete this -> map;
	free(this -> robot);
	free(this -> laser);
}

void Mapper::handlePose(const geometry_msgs::Pose::ConstPtr& data){
	//this -> robot -> position.x = data -> position.x;
	//this -> robot -> position.y = data -> position.y;
	//this -> robot -> yaw = tf::getYaw(data -> orientation);
}

void Mapper::handleGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr& data){
    int i = 0;
	
    while(data -> name[i] != "mobile_base"){
        i++;
    }

	this -> robot -> position.x = data -> pose[i].position.x;
	this -> robot -> position.y = data -> pose[i].position.y;
	this -> robot -> yaw = tf::getYaw(data -> pose[i].orientation);
}

void Mapper::calculateDistances(float real_x, float real_y){
	std::list<LaserPoint>::iterator it;
	std::list<LaserPoint> ranges;
	_2DPoint aux;
	float theta;
	int i = 0;

	ranges = this -> laser -> getRanges();
	it = ranges.begin();
	ROS_INFO("SIZE %d", (int) ranges.size());

	while(it != ranges.end() && ros::ok()){
		if((*it).range < 6){

			theta = Resources::angleSum(this -> robot -> yaw, (*it).angle);
				
			aux.x = real_x +  (cos(theta) * (*it).range); 
							   - (X_DISPLACEMENT * cos(this -> robot -> yaw)); 
			aux.y = real_y + (sin(theta) * (*it).range);
							   - (X_DISPLACEMENT * sin(this -> robot -> yaw));


			ROS_INFO("(%d) %3.2f %3.2f %3.2f", i, theta, aux.x, aux.y);

			addToMap(aux, FULL);

			/*
			aux.x = robot -> position.x;
			aux.y = robot -> position.y;

			addToMap(aux, ME);
			*/
		}

		i++;
		it++;
	}
}

void Mapper::createMap(){
	float last_x = 1000, last_y = 1000;
	float y_diff, x_diff;

	ROS_INFO("Waiting for laser");
	
	while(laser -> isReady() == false && ros::ok()){
		ros::spinOnce();
	}
	
	ROS_INFO("Laser ready");

	while(ros::ok()){

		while(this -> laser -> getStatus() == false && ros::ok()){
			ros::spinOnce();
		}

		x_diff = fabs(this -> robot -> position.x - last_x);
		y_diff = fabs(this -> robot -> position.y - last_y);

		if(x_diff >= 0.5 || y_diff >= 0.5){

			calculateDistances(this -> robot -> position.x, this -> robot -> position.y);

			last_x = this -> robot -> position.x;
			last_y = this -> robot -> position.y;

			writeMap();
		}

		laser -> setStatus(false);
	}
}

void Mapper::addToMap(_2DPoint point, CellValue value){
	if(!INSIDE(point))
		return;

	this -> map -> at(((int) TO_CELLS(point.x) + BASE_X)*TO_CELLS(this -> length) + 
					  ((int) TO_CELLS(point.y) + BASE_Y)) = value;
}



std::vector<CellValue>* Mapper::getMap(){
	return this -> map;
}

void Mapper::writeMap(){
	int i, j;

	std::ofstream file("containers.map");

	for(i = 0; i < TO_CELLS(length); i++){
		for(j = 0; j < TO_CELLS(width); j++)	
			file.put(this -> map -> at((i*TO_CELLS(length)) + j));

		file.put('\n');
	}

	file.close();
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