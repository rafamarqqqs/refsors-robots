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

//#define BASE 0.785398
#define INCREMENT (3.14/RANGES)
//#define INCREMENT ((4.71239 - (BASE*2))/RANGES)
#define RANGES 8

#define LASER_NOT_INITIALIZED(laser) (laser -> angle_increment == -1 && \
                                      laser -> angle_min == -1 &&       \
                                      laser -> angle_max == -1)

class Laser {
public:
	Laser();
    Laser(float angle_increment, float angle_min, float angle_max);
    virtual ~Laser();

    void handleSubscription(const sensor_msgs::LaserScan::ConstPtr &laser_data);

    void setAngleIncrement(float angle_increment);
    void setAngleMax(float angle_max);
    void setAngleMin(float angle_min);
    void setStatus(bool status);

    float getAngleIncrement();
    float getAngleMax();
    float getAngleMin();
    std::list<LaserPoint> getRanges(); 
    bool getStatus();
    float getRadius();
    float getFront();

private:
    std::list<LaserPoint> ranges;
    float angle_increment;
    float angle_min;
    float angle_max;
    bool status;
    float front;
};

#endif