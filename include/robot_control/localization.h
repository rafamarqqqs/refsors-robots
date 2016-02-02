#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdlib.h> 
#include <cmath>
#include "laser.h"

class Localization {
public:
	Localization();
	Localization(float yaw, float x, float y);
	~Localization();

	void getTfTransforms();
	float getYaw();
	_2DPoint* getPosition();

private:
	_2DPoint* position;
	float yaw;
	tf::StampedTransform transform;
    tf::TransformListener listener;
};