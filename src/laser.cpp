#include "../include/robot_control/laser.h"

Laser::Laser(){
	this -> angle_increment = -1;
	this -> angle_max = -1;
	this -> angle_min = -1;
	this -> status = false;
}

Laser::Laser(float angle_increment, float angle_max, float angle_min){
	this -> angle_increment = angle_increment;
	this -> angle_max = angle_max;
	this -> angle_min = angle_min;
	this -> status = false;
}

Laser::~Laser(){}

void Laser::setAngleIncrement(float angle_increment){
	this -> angle_increment = angle_increment;
}
    
void Laser::setAngleMax(float angle_max){
	this -> angle_max = angle_max;
}
    
void Laser::setAngleMin(float angle_min){
	this -> angle_min = angle_min;
}

void Laser::setStatus(bool status){
	this -> status = status;
}

float Laser::getAngleIncrement(){
	return this -> angle_increment;
}

float Laser::getAngleMax(){
	return this -> angle_max;
}

float Laser::getAngleMin(){
	return this -> angle_min;
}

bool Laser::getStatus(){
	return this -> status;
}

std::list<float> Laser::getRanges(){
	return this -> ranges;	
}

float Laser::getFront(){
	return this -> front;
}

void Laser::handleSubscription(const sensor_msgs::LaserScan::ConstPtr &laser_data){
    int i;

    while(this -> status == true);

    this -> ranges.clear();

    //Store some info for later use by other functions
    if(LASER_NOT_INITIALIZED(this)){
        this -> setAngleIncrement(laser_data -> angle_increment);
        this -> setAngleMin(laser_data -> angle_min);
        this -> setAngleMax(laser_data -> angle_max);
    }


    for(i = 0; i <= RANGES; i++){
        this -> ranges.push_back(laser_data -> ranges[(floor(INCREMENT/angle_increment*i))]);
    }

    this -> front = 10;

    for(i = 340; i < 380; i++){
    	if(laser_data -> ranges[i] < this -> front);
    		this -> front = laser_data -> ranges[i];
    }

    this -> status = true;
}