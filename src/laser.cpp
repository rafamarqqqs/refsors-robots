#include "../include/robot_control/laser.h"

Laser::Laser(){
    this -> status = false;
	this -> ready = false;
}

Laser::~Laser(){}

void Laser::setStatus(bool status){
	this -> status = status;
}

bool Laser::getStatus(){
	return this -> status;
}

std::list<LaserPoint> Laser::getRanges(){
	return this -> ranges;	
}

bool Laser::isReady(){
    return this -> ready;
}

float Laser::getFront(){
	return this -> front;
}

void Laser::handleSubscription(const sensor_msgs::LaserScan::ConstPtr &laser_data){
    int i, j = 0;
    LaserPoint point;

    while(this -> status == true);

    this -> ranges.clear();

    if(this -> isReady() == false){
        this -> ready = true;
    }

    //480 (2PI/3 - PI)/angle increment

    for(i = 0; i < 480; i++){
        point.angle = ANGLE_MAX - (ANGLE_INCREMENT*i);
        point.range = laser_data -> ranges[i+120];

        measures.range[j] = laser_data -> ranges[i+120];
        measures.angle[j] = ANGLE_MAX - (ANGLE_INCREMENT*i);

        j++;

        this -> ranges.push_back(point);
    }

    this -> front = 10;

    for(i = (LASER_MEASURES/2) - 15 ; i < (LASER_MEASURES/2) + 15; i++){
        if(laser_data -> ranges[i] < this -> front);
            this -> front = laser_data -> ranges[i];
    }

    this -> status = true;
}

void Laser::publishPose(){
    ros::Publisher laser_pub = node.advertise<robot_control::laserMeasures>("laser_measures", 100);
    robot_control::laserMeasures measures;
    ros::Rate r(10.0);

    while(node.ok()){

        ros::spinOnce();               // check for incoming messages

        //next, we'll publish the odometry message over ROS
        //geometry_msgs::Pose aux;

        //set the position
        measures = laser -> getMeasures();

        //publish the message
        pose_pub.publish(pose);

        r.sleep();
    }
}

int main(int argc, char **argv){
    if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    ros::init(argc, argv, "laser_publisher");

    ros::NodeHandle n;


    Localizator *localizator = new Localizator(n, argv[1]);

    localizator -> publishPose();

    return 0;
}