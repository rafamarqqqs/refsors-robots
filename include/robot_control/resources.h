#include <cmath>
#include <ros/ros.h>
#include "topics.h"

#ifndef _RESOURCES_H_
#define _RESOURCES_H_

#define ANGLE_BASE (2*RAD_45/ANGLE_INCREMENT)
#define LASER_MEASURES 720
#define ANGLE_MAX 1.57
#define ANGLE_MIN -1.57
#define ANGLE_INCREMENT (4.71239/LASER_MEASURES)
#define FRONT_SIZE(type) (strcmp(type, "larger_robot") ? 15 : 12)
#define MIN_FRONT(type) (strcmp(type, "larger_robot") ? 8 : 2.5)
#define RAD_45 0.785398

#define MAP_FULL 100
#define MAP_UNKNOWN 50
#define MAP_EMPTY 0
#define WALKABLE 0

#define EMPTY_RANGE 33
//#define EMPTY_RANGE 20
#define FULL_RANGE 66
//#define FULL_RANGE 80

#define MAX_SCANS 150

#define LASER_MAX_RANGE 10
#define LASER_MIN_RANGE 0.15

#define CELL_SIZE 0.5

#define DISCRETE_ERROR 0.1

#define MAX_RANGE 8

#define LASER_PI_MEASURES 480

#define MAP_MEASURES 10
#define NAV_MEASURES 8

#define IS_POINT_INSIDE(x, y) (x + (this -> length/2) >= 0 && y + (this -> width/2) >= 0 && \
				   x < this -> length/2 && y < this -> width/2)

#define IS_CELL_INSIDE(x, y) (x >= 0 && y >= 0 && x < TO_CELLS(length) && y < TO_CELLS(width))
#define TO_CELLS(v) ((int)floor(v/cell_size))

#define IS_UNKNOWN_CELL_INSIDE(x, y, c) (x >= 0 && y >= 0 && x < TO_UNKNOWN_CELLS(length, c) && y < TO_UNKNOWN_CELLS(width, c))
#define TO_UNKNOWN_CELLS(v, c) ((int)floor(v/c))

#define UNKNOWN_CELL_BASE_X(c) ((int)floor((this -> length/c) / 2))
#define UNKNOWN_CELL_BASE_Y(c) ((int)floor((this -> width/c) / 2))

#define BASE_X ((int)floor((length/cell_size) / 2))
#define BASE_Y ((int)floor((width/cell_size) / 2))

#define MAX_TAIL_SIZE 3

#define EMPTY 0
#define OCCUPIED 1

#define OCCUPIED_FACTOR 0.4

#define UNDEFINED 1000

#define MAP_LENGTH 40.0
#define MAP_WIDTH 40.0
#define SMALLER_ROBOT_CELL_SIZE 1.0
#define LARGER_ROBOT_CELL_SIZE 1.0
#define REPULSION 30

#define AREA_SIZE 8

#define ROBOTS_MIN_DIST 1.5;

#define LOCALIZATION_STARTED ((robot -> position.x != 0 || robot -> position.y != 0) ? true : false)
#define LASER_STARTED ((range.size() != 0 && angle.size() != 0) ? true : false)

#define SQUARE(x) (x*x)
#define TO_THE_FOURTH(x) (x*x)

#define QROBOTS 0.5
#define QGOAL 0.8
#define QWALL 0.8
#define QOG 0.8
#define QTAIL 0.4
#define MAX_LIN_SPEED 0.7
#define MAX_ANG_SPEED 0.9

#define TIME_LIMIT (1 * 60)

#define ERROR 3.0

#define REACHED_DESTINATION(g, p) (fabs(g -> x - p.x) < ERROR && fabs(g -> y - p.y) < ERROR)

#define REACHED_TIME_LIMIT(start, now) ((now - start) > TIME_LIMIT ? true : false)
#define REACHED_GLOBAL_TIME_LIMIT(start, now) ((now - start) > 5*60 ? true : false)

#define ORTOGONAL_WEIGTH 10
#define DIAGONAL_WEIGTH 14

#define CTE 10


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
	int x, y;
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
	double front;
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