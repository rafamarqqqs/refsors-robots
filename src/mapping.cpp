#include "../include/robot_control/mapping.h"

Mapper::Mapper(ros::NodeHandle n, float length, float width, float cell_size, char *type):
    createMapServer(n, "createMap", boost::bind(&Mapper::createMap, (Mapper *) this, _1), false) {
	int i, j;

	this -> flag = strcmp(type, "smaller_robot") == 0 ? false : true;

	this -> node = n;
	this -> length = length;
	this -> width = width;
	this -> cell_size = cell_size;

	this -> robot = (Robot *) malloc(sizeof(Robot));

	this -> laser_ready = false;
	
	laser_sub = node.subscribe(LASER(type), 1, &Mapper::handleLaser, this);
	pose_sub = node.subscribe(POSE(type), 1, &Mapper::handlePose, this);
	
	initMap();

	createMapServer.start();
}

Mapper::~Mapper(){
	delete map;
	free(this -> robot);
}

void Mapper::initMap(){
	int i, j;

	map = (char **) malloc(sizeof(char *)*TO_CELLS(length));

	for(i = 0; i < TO_CELLS(length); i++)
		map[i] = (char *) malloc(sizeof(char) * TO_CELLS(width));
	
	for(i = 0; i < TO_CELLS(length); i++){
		for(j = 0; j < TO_CELLS(width); j++)
			map[i][j] = UNKNOWN;

	}
}

void Mapper::handlePose(const geometry_msgs::Pose::ConstPtr& data){
	this -> robot -> position.x = data -> position.x;
	this -> robot -> position.y = data -> position.y;
	this -> robot -> yaw = tf::getYaw(data -> orientation);
}

void Mapper::handleLaser(const robot_control::laserMeasures::ConstPtr& data){
    if(data -> range.size() == 0 || data -> angle.size() == 0)
        return;


    this -> range = data -> range;
    this -> angle = data -> angle;
    this -> front = data -> front;

    this -> laser_ready = true;

}


void Mapper::calculateDistances(_2DPoint real_pose){
	_2DPoint aux;
	float theta;

	std::vector<float>::iterator range_it = range.begin();
    std::vector<float>::iterator angle_it = angle.begin();

	while(range_it != range.end()){
		if((*range_it) < 25){

			theta = Resources::angleSum(this -> robot -> yaw, (*angle_it));
				
			aux.x = real_pose.x +  (cos(theta) * (*range_it)); 
			aux.y = real_pose.y + (sin(theta) * (*range_it));

			//ROS_INFO("(%d) %3.2f %3.2f %3.2f", i, theta, aux.x, aux.y);

			addToMap(aux, FULL, real_pose, cos(theta)*0.5, sin(theta)*0.5, (*range_it));

			/*
			aux.x = robot -> position.x;
			aux.y = robot -> position.y;

			addToMap(aux, ME, real_pose, 0, 0, 0);
			*/
		}

		range_it++;
		angle_it++;
	}
}

void Mapper::createMap(const robot_control::createMapGoalConstPtr &goal){
	float last_x = 1000, last_y = 1000;
	float y_diff, x_diff;

	//ROS_INFO("mapper waiting for laser");

	ros::Rate r(10);

    ROS_INFO("navigator waiting for laser");
    while(laser_ready == false && ros::ok()){
        r.sleep();
    }

	while(ros::ok()){

		x_diff = fabs(this -> robot -> position.x - last_x);
		y_diff = fabs(this -> robot -> position.y - last_y);

		if(x_diff >= 0.5 || y_diff >= 0.5){

			_2DPoint aux;

			aux.x = this -> robot -> position.x;
			aux.y = this -> robot -> position.y;

			calculateDistances(aux);

			last_x = this -> robot -> position.x;
			last_y = this -> robot -> position.y;

			if(flag)
				writeMap();
		}

		r.sleep();
	}
}

void Mapper::addToMap(_2DPoint point, char value, _2DPoint real_pose, float x_inc, float y_inc, float range){
	if(!INSIDE(point))
		return;

	int x = BASE_X + TO_CELLS(point.x);
	int y = BASE_Y + TO_CELLS(point.y);
	float aux_x = real_pose.x;
	float aux_y = real_pose.y;
	int map_x, map_y;

	while(COMPARE(real_pose.x, point.x, aux_x) && COMPARE(real_pose.y, point.y, aux_y)){
		aux_x += x_inc;
		aux_y += y_inc;


		map_x = ((int) TO_CELLS(aux_x)) + BASE_X;
		map_y = ((int) TO_CELLS(aux_y)) + BASE_Y;

		if(map[map_x][map_y] != FULL/* && map[map_x][map_y] != ME*/)
			map[map_x][map_y] = EMPTY;

	}

	map[x][y] = value;
}



char** Mapper::getMap(){
	return map;
}

void Mapper::writeMap(){
	int i, j;

	std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/containers.map");

	for(i = 0; i < TO_CELLS(length); i++){
		for(j = 0; j < TO_CELLS(width); j++){
			file.put(map[i][j]);
			//VISUALIZATION PURPOSES, REMOVE WHEN READY
			//file.put(map[i][j]);
		}

		file.put('\n');
	}

	file.close();
}

int main(int argc, char **argv) {
	if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "Mapping");

    ros::NodeHandle node;

    Mapper *mapper = new Mapper(node, 40.0, 40.0, 0.5, argv[1]);

    ROS_INFO("Mapper started.");

    ros::spin();

    return 0;
}