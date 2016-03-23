#include "resources.h"
#include <vector>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <list>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include "topics.h"
#include <fstream>
#include <tf/LinearMath/Transform.h>
#include <actionlib/server/simple_action_server.h>
#include "robot_control/addToMap.h"
#include "robot_control/createMapAction.h"
#include "robot_control/laserMeasures.h"
#include <string.h>

#ifndef _MAPPING_H_
#define _MAPPING_H_

typedef actionlib::SimpleActionServer<robot_control::createMapAction> CreateMapAction;

class Mapper {
public:
	Mapper(ros::NodeHandle n, double length, double width, char *type);
	~Mapper();

 	void createMap(const robot_control::createMapGoalConstPtr &goal);

private:
	void handleLaser(const robot_control::laserMeasures::ConstPtr& data);

	void handlePose(const geometry_msgs::PoseStamped::ConstPtr& data);

	void calculateDistances(double real_x_pose, double real_y_pose);

	double length;
	double width;
	
	Robot* robot;
	
    std::vector<double> range;
    std::vector<double> angle;

	std::string type;

	ros::ServiceClient client;

	CreateMapAction createMapServer;

	ros::Subscriber laser_sub;
	ros::Subscriber pose_sub;

	ros::NodeHandle node;
};

#endif