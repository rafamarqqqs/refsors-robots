#include <cmath>

#ifndef _RESOURCES_H_
#define _RESOURCES_H_

class Resources {
public:

	inline static double angleSum(double angle_a, double angle_b){
		angle_a = normalizeAngle(angle_a);
		angle_b = normalizeAngle(angle_b);
		
		double sum = angle_a + angle_b;
	    
	    sum = fmod(sum,2*M_PI);

	    if(sum <= M_PI)
		    return sum;
	  	
	  	return sum - (2*M_PI);
	}


	inline static double angleDiff(double angle_a, double angle_b){
	    double diff1 = angle_a - angle_b;
	    double diff2 = diff1 - 2*M_PI;

	    if(fabs(diff1) <= fabs(diff2))
	        return diff1;

	    return diff2;
	}

	inline static double transformYaw(double yaw){
		yaw = fmod(yaw, 2*M_PI);

		if(yaw < 0)
			return yaw + (2*M_PI);

		return yaw;
	}

	inline static double normalizeAngle(double angle){
		if(angle < 0){
			angle += (2*M_PI);
		}

		return angle;
	}

};

typedef struct point {
	double angle;
	double range;
} LaserPoint;

typedef struct _2dpoint {
    double x;
    double y;
} _2DPoint;

typedef struct robot {
	_2DPoint position;
	double yaw;
	double pitch;
	double roll;
} Robot;

#endif