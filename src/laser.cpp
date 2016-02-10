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

    if(this -> status == true)
        return;

    this -> ranges.clear();

    //Store some info for later use by other functions
    if(this -> isReady() == false){
        this -> ready = true;
    }

    for(i = 0; i < MEASURES; i+= (int) MEASURES/RANGES){
        
        point.angle = ANGLE_MIN + (ANGLE_INCREMENT*i);
        point.range = laser_data -> ranges[i];

        this -> ranges.push_back(point);
    }


    //point.angle = ANGLE_MIN + (ANGLE_INCREMENT*i);
    //point.range = laser_data -> ranges[639];
    //this -> ranges.push_back(point);

    this -> front = 10;

    for(i = (MEASURES/2) - 15 ; i < (MEASURES/2) + 15; i++){
        if(laser_data -> ranges[i] < this -> front);
            this -> front = laser_data -> ranges[i];
    }

    this -> status = true;
}