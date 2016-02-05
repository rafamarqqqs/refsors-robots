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

class Localizator {
public:
	Localizator();
	~Localizator();

	void getYaw();
	void getPosition();
	geometry_msgs::Pose getPose();

private:
	geometry_msgs::Pose pose;

	tf::StampedTransform odom_transform;
    tf::TransformListener odom_listener;

	tf::StampedTransform imu_transform;
    tf::TransformListener imu_listener;
};