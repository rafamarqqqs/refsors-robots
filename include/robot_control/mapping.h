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
#include <actionlib/server/simple_action_server.h>
#include "robot_control/createMapAction.h"

#ifndef _MAPPING_H_
#define _MAPPING_H_

#define max(a, b) (a > b ? a : b)
#define min(a, b) (a > b ? b : a)

#define COMPARE(start, goal, pos) (start > goal ? (pos > goal) : (pos < goal))

#define INSIDE(p) (p.x + (this -> length/2) >= 0 && p.y + (this -> width/2) >= 0 && \
				   p.x < this -> length/2 && p.y < this -> width/2)

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

#define FULL '#'
#define UNKNOWN ':'
#define EMPTY ' '
#define ME 'x'

#define GET_SIGNAL(yaw) (fabs(yaw) > M_PI/2 ? -1 : 1)

#define BASE (M_PI/4)

#define TO_CELLS(v) (((int)(v/this -> cell_size)))

typedef actionlib::SimpleActionServer<robot_control::createMapAction> CreateMapAction;

class Mapper {
public:
	Mapper(ros::NodeHandle n, float length, float width, float cell_size, char *type);
	~Mapper();

	char** getMap();

	void createMap(const robot_control::createMapGoalConstPtr &goal);

private:
	void writeMap();

	void addToMap(_2DPoint point, char value, _2DPoint real_pose, float x_inc, float y_inc, float range);

	void initMap();
	
	void handlePose(const geometry_msgs::Pose::ConstPtr &data);

	void calculateDistances(_2DPoint real_pose);

	float length;
	float width;
	float cell_size;
	
	Robot* robot;
	//Robot* robotAux;
	Laser* laser;
	
	char** map;
	ros::NodeHandle node;
	std::list<_2DPoint> points;

	float real_x;
	float real_y;

	CreateMapAction createMapServer;

	ros::Subscriber laser_sub;
	ros::Subscriber pose_sub;

	int status;
};


#endif