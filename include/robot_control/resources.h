#include <cmath>

#ifndef _RESOURCES_H_
#define _RESOURCES_H_

#define MEASURES 480

class Resources {
public:

	static float angleSum(float angle_a, float angle_b);

	static float angleDiff(float angle_a, float angle_b);

	static float transformYaw(float yaw);

	static float normalizeAngle(float angle);

};

typedef struct point {
	float angle;
	float range;
} LaserPoint;

typedef struct _2dpoint {
    float x;
    float y;
} _2DPoint;

typedef struct robot {
	_2DPoint position;
	float yaw;
} Robot;

#endif