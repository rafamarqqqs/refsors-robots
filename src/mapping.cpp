#include "../include/robot_control/mapping.h"

Mapper::Mapper(ros::NodeHandle n, float length, float width, float cell_size, char *type):
    createMapServer(n, "createMap", boost::bind(&Mapper::createMap, (Mapper *) this, _1), false) {
	int i, j;

	std::string aux(type);

	this -> type = aux;

	this -> node = n;
	this -> length = length;
	this -> width = width;
	this -> cell_size = cell_size;

	this -> robot = (Robot *) malloc(sizeof(Robot));

	this -> laser_ready = false;
	
	laser_sub = node.subscribe(LASER(type), 1, &Mapper::handleLaser, this);
	pose_sub = node.subscribe(POSE(type), 1, &Mapper::handlePose, this);

	client = n.serviceClient<robot_control::addToMap>("/Knowledge/addToMap");
	
	createMapServer.start();
}

Mapper::~Mapper(){
	free(this -> robot);
}

void Mapper::handlePose(const geometry_msgs::PoseStamped::ConstPtr& data){
	this -> robot -> position.x = data -> pose.position.x;
	this -> robot -> position.y = data -> pose.position.y;
	this -> robot -> yaw = tf::getYaw(data -> pose.orientation);
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
	int interval = RANGES/MEASURES;
	bool last_measure = true;
	robot_control::addToMap srv;

	std::vector<float>::iterator range_it = range.begin();
    std::vector<float>::iterator angle_it = angle.begin();

	while(range_it != range.end()){
			theta = Resources::angleSum(this -> robot -> yaw, (*angle_it));
				
			aux.x = real_pose.x + (cos(theta) * (*range_it)); 
			aux.y = real_pose.y + (sin(theta) * (*range_it));

			srv.request.wall_x = aux.x;
			srv.request.wall_y = aux.y;

			srv.request.start_x = real_pose.x;
			srv.request.start_y = real_pose.y;

			srv.request.inc_x = cos(theta)*0.25;
			srv.request.inc_y = sin(theta)*0.25;

			srv.request.range = (*range_it);

			client.call(srv);
			
			//addToMap(aux, FULL, real_pose, cos(theta)*0.1, sin(theta)*0.1, (*range_it));

			/*
			aux.x = robot -> position.x;
			aux.y = robot -> position.y;

			addToMap(aux, ME, real_pose, 0, 0, 0);
			*/

		if(!last_measure)
			break;
		
		for(int i = 0; i < interval; i++){
			range_it++;
			angle_it++;
		}

		if(range_it == range.end() && last_measure){
			last_measure = false;
			range_it--;
			angle_it--;	
		}
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

			//writeMap();
		}

		r.sleep();
	}
}

int main(int argc, char **argv) {
	if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "Mapping");

    ros::NodeHandle node;

    Mapper *mapper = new Mapper(node, 40.0, 40.0, 0.25, argv[1]);

    ROS_INFO("Mapper started.");

    ros::spin();

    return 0;
}