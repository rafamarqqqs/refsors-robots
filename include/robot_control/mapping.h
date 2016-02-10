#include "resources.h"
#include <vector>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <list>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include "topics.h"
#include <fstream>
#include "laser.h"

#ifndef _MAPPING_H_
#define _MAPPING_H_

#define INSIDE(p) (p.x + (this -> length/2) >= 0 && p.y + (this -> width/2) >= 0 && \
				   p.x < this -> length/2 && p.y < this -> width/2)

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

typedef char CellValue;

#define FULL '*'
#define EMPTY ' '
#define ME 'X'

#define GET_SIGNAL(yaw) (fabs(yaw) > M_PI/2 ? -1 : 1)

#define BASE (M_PI/4)

#define TO_CELLS(v) (((int)(v/this -> cell_size)))

class Mapper {
public:
	Mapper(ros::NodeHandle n, float length, float width, float cell_size);
	~Mapper();


	std::vector<CellValue>* getMap();

	void createMap();

private:
	void writeMap();

	void addToMap(_2DPoint point, CellValue value);
	
	void handlePose(const geometry_msgs::Pose::ConstPtr &data);

	void calculateDistances(float real_x, float real_y);

	float length;
	float width;
	float cell_size;
	
	Robot* robot;
	//Robot* robotAux;
	Laser* laser;
	
	std::vector<CellValue>* map;
	ros::NodeHandle node;
	std::list<_2DPoint> points;

	float real_x;
	float real_y;

	ros::Subscriber laser_sub;
	ros::Subscriber pose_sub;

	int status;
};


#endif