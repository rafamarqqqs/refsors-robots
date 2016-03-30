#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdlib.h> 
#include <cmath>
#include "laser.h"
#include "topics.h"
#include <gazebo_msgs/ModelStates.h>

class Localizator {
public:
	Localizator(ros::NodeHandle n, char* type);
	virtual ~Localizator();

	void publishPose(char* type);

private:
	void handleGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr& data);
	
	geometry_msgs::PoseStamped getPose();

	geometry_msgs::PoseStamped pose;

	ros::Subscriber gazebo_pose_sub;

	double laser_displacement;

	ros::NodeHandle node;

	std::string robot_name;

	bool position_ready;
	int id;
};