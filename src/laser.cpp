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
    int i;
    LaserPoint point;

    while(this -> status == true);
        //return;
    //ROS_INFO("%3.2f %3.2f %3.2f %3.2f", laser_data -> angle_min, laser_data -> angle_max, laser_data -> angle_increment,
      //                              (laser_data -> angle_max - laser_data -> angle_min)/laser_data -> angle_increment);
    this -> ranges.clear();

    //Store some info for later use by other functions
    if(this -> isReady() == false){
        this -> ready = true;
    }

    //ROS_INFO("%.3f %.3f", RAD_45/ANGLE_INCREMENT, MEASURES - RAD_45/ANGLE_INCREMENT);

    //480 (2PI/3 - PI)/angle increment

    for(i = 0; i < 480; i++){
        
        point.angle = ANGLE_MAX - (ANGLE_INCREMENT*i);
        point.range = laser_data -> ranges[i+120];
        //ROS_INFO("%d %3.2f", i, point.angle);

        this -> ranges.push_back(point);
    }


    //point.angle = ANGLE_MIN + (ANGLE_INCREMENT*i);
    //point.range = laser_data -> ranges[639];
    //this -> ranges.push_back(point);

    this -> front = 10;

    for(i = (LASER_MEASURES/2) - 15 ; i < (LASER_MEASURES/2) + 15; i++){
        if(laser_data -> ranges[i] < this -> front);
            this -> front = laser_data -> ranges[i];
    }

    this -> status = true;
}