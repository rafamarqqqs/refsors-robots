#include "../include/robot_control/localization.h"

Localization::Localization(){
	this -> position = (_2DPoint*) malloc(sizeof(_2DPoint));
}

Localization::Localization(float yaw, float x, float y){
	this -> position = (_2DPoint*) malloc(sizeof(_2DPoint));
	this -> position -> x = x;
	this -> position -> y = y;
	this -> yaw = yaw;
}

Localization::~Localization(){
	free(this -> position);
}

_2DPoint* Localization::getPosition(){
	return this -> position;
}

float Localization::getYaw(){
	return this -> yaw;
}

void Localization::getTfTransforms(){
    listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);

    this -> position -> x = transform.getOrigin().x();
    this -> position -> y = transform.getOrigin().y();

    tf::Quaternion q(transform.getRotation().x(), 
	    			 transform.getRotation().y(),
	    			 transform.getRotation().z(),
	    			 transform.getRotation().w());

    this -> yaw = tf::getYaw(q);
}