#include <list>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h> 
#include "resources.h"
//#include <sensor_msgs/PointCloud.h>
//#include "laser_geometry/laser_geometry.h"

#ifndef _LASER_H_
#define _LASER_H_

#define X_DISPLACEMENT 0.35
#define Y_DISPLACEMENT 0

#define ANGLE_BASE (2*RAD_45/ANGLE_INCREMENT)
#define LASER_MEASURES 720
#define MEASURES 480
#define ANGLE_MAX 1.57
#define ANGLE_MIN -1.57
#define ANGLE_INCREMENT (4.71239/LASER_MEASURES)
#define RAD_45 0.785398

class Laser {
public:
	Laser();
    Laser(float angle_increment, float angle_min, float angle_max);
    virtual ~Laser();

    void handleSubscription(const sensor_msgs::LaserScan::ConstPtr &laser_data);

    void setStatus(bool status);
    bool getStatus();
    bool isReady();

    std::list<LaserPoint> getRanges(); 
    float getFront();

private:
    std::list<LaserPoint> ranges;
    bool status;
    bool ready;
    float front;
};

#endif