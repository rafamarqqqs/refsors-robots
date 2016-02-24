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
#include <actionlib/server/simple_action_server.h>
#include "robot_control/addToMap.h"
#include "robot_control/createMapAction.h"
#include "robot_control/laserMeasures.h"

#ifndef _MAPPING_H_
#define _MAPPING_H_

#define INSIDE(p) (p.x + (this -> length/2) >= 0 && p.y + (this -> width/2) >= 0 && \
				   p.x < this -> length/2 && p.y < this -> width/2)

#define TO_CELLS(v) (((int)floor(v/this -> cell_size)))

typedef actionlib::SimpleActionServer<robot_control::createMapAction> CreateMapAction;

class Mapper {
public:
	Mapper(ros::NodeHandle n, float length, float width, float cell_size, char *type);
	~Mapper();

 	void createMap(const robot_control::createMapGoalConstPtr &goal);

private:
	void addToMap(_2DPoint point, char value, _2DPoint real_pose, float x_inc, float y_inc, float range);

	void handleLaser(const robot_control::laserMeasures::ConstPtr& data);

	void handlePose(const geometry_msgs::Pose::ConstPtr& data);

	void calculateDistances(_2DPoint real_pose);

	float length;
	float width;
	float cell_size;
	
	Robot* robot;
	
    std::vector<float> range;
    std::vector<float> angle;
    float front;

    bool laser_ready;

	std::string type;

	ros::NodeHandle node;
	std::list<_2DPoint> points;

	ros::ServiceClient client;

	float real_x;
	float real_y;

	CreateMapAction createMapServer;

	ros::Subscriber laser_sub;
	ros::Subscriber pose_sub;

	int status;
};

#endif