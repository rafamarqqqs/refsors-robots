#include <cmath>
#include <ros/ros.h>

#ifndef _RESOURCES_H_
#define _RESOURCES_H_

#define ANGLE_BASE (2*RAD_45/ANGLE_INCREMENT)
#define LASER_MEASURES 720
#define ANGLE_MAX 1.57
#define ANGLE_MIN -1.57
#define ANGLE_INCREMENT (4.71239/LASER_MEASURES)
#define FRONT_SIZE(type) (strcmp(type, "larger_robot") ? 15 : 9)
#define MIN_FRONT(type) (strcmp(type, "larger_robot") ? 8 : 3)
#define RAD_45 0.785398

#define MAP_FULL 100
#define MAP_UNKNOWN 50
#define MAP_EMPTY 0

#define EMPTY_RANGE 30
#define FULL_RANGE 70

#define CELL_SIZE 0.5

#define DISCRETE_ERROR 0.1

#define MAX_RANGE 8

#define LASER_PI_RANGES 480
#define MEASURES 8

#define IS_POINT_INSIDE(x, y) (x + (this -> length/2) >= 0 && y + (this -> width/2) >= 0 && \
				   x < this -> length/2 && y < this -> width/2)

#define IS_CELL_INSIDE(x, y) (x >= 0 && y >= 0 && x < TO_CELLS(length) && y < TO_CELLS(width))
#define TO_CELLS(v) (((int)floor(v/this -> cell_size)))

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

#define MAX_TAIL_SIZE 1

#define EMPTY 0
#define OCCUPIED 1

#define UNDEFINED 1000

#define RANGES 8

#define MAP_LENGTH 40.0
#define MAP_WIDTH 40.0
#define SMALLER_ROBOT_CELL_SIZE 1.0
#define LARGER_ROBOT_CELL_SIZE 1.0
#define REPULSION 30

#define AREA_SIZE 8

#define ROBOTS_MIN_DIST 1.5;

#define LOCALIZATION_STARTED ((pose.position.x != 0 || pose.position.y != 0) ? true : false)
#define LASER_STARTED ((range.size() != 0 && angle.size() != 0) ? true : false)

#define SQUARE(x) (x*x)
#define TO_THE_FOURTH(x) (x*x*x)

#define QROBOTS 0.5
#define QGOAL 0.8
#define QWALL 0.8
#define QOG 0.8
#define QTAIL 0.4
#define MAX_LIN_SPEED 0.7
#define MAX_ANG_SPEED 0.9

#define TIME_LIMIT (3 * 60)

#define ERROR 3.0

#define REACHED_DESTINATION(g, p) (fabs(g -> x - p.x) < ERROR && fabs(g -> y - p.y) < ERROR)

#define REACHED_TIME_LIMIT(start, now) ((now - start) > TIME_LIMIT ? true : false)

#define ORTOGONAL_WEIGTH 10
#define DIAGONAL_WEIGTH 14

#define CTE 10

#define TO_UNKNOWN_CELLS(v, c) (floor(v/c))

#define UNKNOWN_CELL_BASE_X(c) (floor((this -> length/c) / 2))
#define UNKNOWN_CELL_BASE_Y(c) (floor((this -> width/c) / 2))

typedef struct node {
	int x, y;
	int F, G, H;
	node* parent;
} Node;


typedef struct drivingInfo {
    double rotation;
    double velocity;
} DrivingInfo;

typedef enum lvl {
	NEW,
	RECENT,
	OLD
} Level;

typedef struct {
	double x, y;
	Level level;
} OccupancyTail;

typedef struct goal {
	uint8_t x, y;
} Goal;

typedef struct point {
	double angle;
	double range;
} LaserPoint;

typedef struct _2dpoint {
    double x;
    double y;
} _2DPoint;

typedef struct area {
	_2DPoint start;
	_2DPoint end;
	_2DPoint destiny;
	bool occupied;
} Area;

typedef struct robot {
	_2DPoint position;
	double yaw;
	double pitch;
	double roll;
} Robot;

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

#endif