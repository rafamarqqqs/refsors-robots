#include "../include/robot_control/resources.h"

float Resources::angleSum(float angle_a, float angle_b){
	angle_a = normalizeAngle(angle_a);
	angle_b = normalizeAngle(angle_b);
	
	float sum = angle_a + angle_b;
    
    sum = fmod(sum,2*M_PI);

    if(sum < M_PI)
	    return sum;
  	
  	return sum - (2*M_PI);
}


float Resources::angleDiff(float angle_a, float angle_b){
    float diff1 = angle_a - angle_b;
    float diff2 = diff1 - 2*M_PI;

    if(fabs(diff1) <= fabs(diff2))
        return diff1;

    return diff2;
}

float Resources::transformYaw(float yaw){
	yaw = fmod(yaw, 2*M_PI);

	if(yaw < 0)
		return yaw + (2*M_PI);

	return yaw;
}

float Resources::normalizeAngle(float angle){
	if(angle < 0){
		angle += (2*M_PI);
	}

	return angle;
}