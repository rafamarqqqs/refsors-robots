#include <cmath>

#ifndef _RESOURCES_H_
#define _RESOURCES_H_

class Resources {
public:

	static double angleSum(double angle_a, double angle_b);

	static double angleDiff(double angle_a, double angle_b);

	static double transformYaw(double yaw);

	static double normalizeAngle(double angle);

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
} Robot;

#endif