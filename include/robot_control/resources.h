#include <cmath>

#ifndef _RESOURCES_H_
#define _RESOURCES_H_

class Resources {
public:

	static float angleSum(float angle_a, float angle_b);

	static float angleDiff(float angle_a, float angle_b);

	static float transformYaw(float yaw);

	static float normalizeAngle(float angle);

};

typedef char CellValue;

#define FULL '*'
#define EMPTY ' '

typedef struct point {
	float angle;
	float range;
} LaserPoint;

typedef struct _2dpoint {
    float x;
    float y;
} _2DPoint;

typedef struct _2dvector {
	_2DPoint start;
	_2DPoint end;
} _2DVector;

#endif