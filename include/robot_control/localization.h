#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
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
	Localizator(ros::NodeHandle n);
	virtual ~Localizator();

	geometry_msgs::Pose getPose();

	void handleGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr& data);

private:
	geometry_msgs::Pose pose;

	ros::Subscriber gazebo_pose_sub;

	tf::StampedTransform odom_transform;
    tf::TransformListener odom_listener;
};