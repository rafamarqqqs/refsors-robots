#include <list>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h> 
#include "resources.h"
#include "topics.h"
#include "robot_control/laserMeasures.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdlib.h> 
#include <cmath>

//#include <sensor_msgs/PointCloud.h>
//#include "laser_geometry/laser_geometry.h"

#ifndef _LASER_H_
#define _LASER_H_

#define ANGLE_BASE (2*RAD_45/ANGLE_INCREMENT)
#define LASER_MEASURES 720
#define ANGLE_MAX 1.57
#define ANGLE_MIN -1.57
#define ANGLE_INCREMENT (4.71239/LASER_MEASURES)
#define FRONT_SIZE(type) (strcmp(type, "larger_robot") ? 15 : 9)
#define RAD_45 0.785398

class Laser {
public:
	Laser(ros::NodeHandle node, char* type);
    virtual ~Laser();

    void handleSubscription(const sensor_msgs::LaserScan::ConstPtr& laser_data);

    void publishMeasures(char* type);

private:
    inline robot_control::laserMeasures getMeasures();
    robot_control::laserMeasures measures;
    std::list<LaserPoint> ranges;
    int front_size;
    ros::NodeHandle node;
    ros::Subscriber laser_sub;
};

#endif