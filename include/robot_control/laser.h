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

#ifndef _LASER_H_
#define _LASER_H_

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
    int min_front;
    ros::NodeHandle node;
    ros::Subscriber laser_sub;
};

#endif