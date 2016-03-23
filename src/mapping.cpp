#include "../include/robot_control/mapping.h"

Mapper::Mapper(ros::NodeHandle n, double len, double wid, char *_type):
    createMapServer(n, "createMap", boost::bind(&Mapper::createMap, (Mapper *) this, _1), false),
    type(_type), node(n), length(len), width(wid) {
	
	this -> robot = (Robot *) malloc(sizeof(Robot));

	laser_sub = node.subscribe(LASER(_type), 100, &Mapper::handleLaser, this);
	pose_sub = node.subscribe(POSE(_type), 100, &Mapper::handlePose, this);

	client = n.serviceClient<robot_control::addToMap>("/Knowledge/addToMap");
	
	createMapServer.start();
}

Mapper::~Mapper(){
	free(this -> robot);
}

void Mapper::handlePose(const geometry_msgs::PoseStamped::ConstPtr& data){
	this -> robot -> position.x = data -> pose.position.x;
	this -> robot -> position.y = data -> pose.position.y;
	tf::Quaternion q(data -> pose.orientation.x, data -> pose.orientation.y, data -> pose.orientation.z, data -> pose.orientation.w);
	tf::Matrix3x3(q).getRPY(robot -> roll, robot -> pitch, robot -> yaw);
}

void Mapper::handleLaser(const robot_control::laserMeasures::ConstPtr& data){
    this -> range = data -> range;
    this -> angle = data -> angle;
}


void Mapper::calculateDistances(double real_x_pose, double real_y_pose){

    if(fabs(robot -> roll) > DISCRETE_ERROR || fabs(robot -> pitch) > DISCRETE_ERROR)
    	return;

	int interval = RANGES/MEASURES;
	bool last_measure = true;
	robot_control::addToMap srv;
	double theta;

	std::vector<double>::iterator range_it = range.begin();
    std::vector<double>::iterator angle_it = angle.begin();

	while(range_it != range.end()){
		theta = Resources::angleSum(this -> robot -> yaw, (*angle_it));
				
		srv.request.wall_x = real_x_pose + (cos(theta) * (*range_it));;
		srv.request.wall_y = real_y_pose + (sin(theta) * (*range_it));;

		srv.request.start_x = real_x_pose;
		srv.request.start_y = real_y_pose;

		srv.request.inc_x = cos(theta)*0.25;
		srv.request.inc_y = sin(theta)*0.25;

		srv.request.range = (*range_it);

		client.call(srv);
			
		if(!last_measure)
			break;
		
		for(int i = interval; i--;){
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
	double last_x = 1000, last_y = 1000;
	double y_diff, x_diff;

	ros::Rate r(10);

    ROS_INFO("navigator waiting for laser");
    while(!LASER_STARTED && ros::ok()){
        r.sleep();
    }

	while(ros::ok()){

		//x_diff = fabs(this -> robot -> position.x - last_x);
		//y_diff = fabs(this -> robot -> position.y - last_y);

		//if(x_diff >= 0.25 || y_diff >= 0.25){

			calculateDistances(robot -> position.x, robot -> position.y);

			//last_x = this -> robot -> position.x;
			//last_y = this -> robot -> position.y;

			//writeMap();
		//}

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

    Mapper *mapper = new Mapper(node, MAP_LENGTH, MAP_WIDTH, argv[1]);

    ROS_INFO("Mapper started.");

    ros::spin();

    delete mapper;

    return 0;
}