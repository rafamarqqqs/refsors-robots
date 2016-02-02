#include "../include/robot_control/laser.h"

Laser::Laser(){
	this -> angle_increment = -1;
	this -> angle_max = -1;
	this -> angle_min = -1;
	this -> measures = -1;
	this -> status = false;
}

Laser::Laser(float angle_increment, float angle_max, float angle_min, int measures){
	this -> angle_increment = angle_increment;
	this -> angle_max = angle_max;
	this -> angle_min = angle_min;
	this -> measures = measures;
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

void Laser::setMeasures(int measures){
	this -> measures = measures;
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

int Laser::getMeasures(){
	return this -> measures;
}

void Laser::subscribeTo(ros::NodeHandle node, const std::string topic, int rate, Laser *laser, ros::Subscriber sub){
	sub = node.subscribe(topic, rate, &Laser::handleSubscription, laser);
}

bool Laser::getStatus(){
	return this -> status;
}

float Laser::getRadius(){
	return this -> angle_max - this -> angle_min;
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
        this -> setMeasures((int) ((angle_max - angle_min)/RANGES));
    	//ROS_INFO("%d %.3f %.3f %.3f", this -> measures, this -> angle_min, this -> angle_max, this -> angle_increment);
    	//this -> base = 1.0472/laser_data -> angle_increment;//floor(BASE(laser_data -> angle_min, laser_data -> angle_increment));
    	//ROS_INFO("%.3f %.3f %.3f", fabs(fabs(laser_data -> angle_min) - RAD_90), laser_data -> angle_increment, this -> base);
    }


    for(i = 0; i <= RANGES; i++){
    	//ROS_INFO("%d %d", (int) (floor(this -> getMeasures()/RANGES)*i), this -> base);
    	//ROS_INFO("%.3f -> %.4f", floor((INCREMENT/angle_increment*i) + (this -> base)), 
    	//					laser_data -> ranges[(floor(INCREMENT/angle_increment*i) + floor(this -> base))]);
        this -> ranges.push_back(laser_data -> ranges[(floor(INCREMENT/angle_increment*i))]);
        //ROS_INFO("%.3f -> %.3f", floor(INCREMENT/angle_increment*i), (INCREMENT*i) - angle_max);
    	//ROS_INFO("%.3f %.3f", (INCREMENT*i) + this -> base, laser_data -> ranges[(floor(INCREMENT/angle_increment*i) + floor(this -> base))]);
    }

    this -> front = 10;

    for(i = 340; i < 380; i++){
    	if(laser_data -> ranges[i] < this -> front);
    		this -> front = laser_data -> ranges[i];
    }

    this -> status = true;
}