#include "../include/robot_control/resources.h"

float Resources::angleSum(float angle_a, float angle_b){
	float sum = angle_a + angle_b;
    sum = fmod(sum,2*M_PI);

    if(sum <= 0)
        return sum;
    else
        return sum - (2*M_PI);
}


float Resources::angleDiff(float angle_a, float angle_b){
    float diff1 = angle_a - angle_b;
    float diff2 = diff1 - 2*M_PI;

    if(fabs(diff1) <= fabs(diff2))
        return diff1;
    else
        return diff2;
}