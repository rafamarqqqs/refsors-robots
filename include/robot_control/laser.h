#include <list>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h> 

#ifndef _LASER_H_
#define _LASER_H_

#define INCREMENT 0.523599
#define RANGES 8

#define LASER_NOT_INITIALIZED(laser) (laser -> angle_increment == -1 && \
                                      laser -> angle_min == -1 &&       \
                                      laser -> angle_max == -1)

typedef struct _2dpoint {
    float x;
    float y;
} _2DPoint;

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
    std::list<float> getRanges(); 
    bool getStatus();
    float getRadius();
    float getFront();

private:
    std::list<float> ranges;
    float angle_increment;
    float angle_min;
    float angle_max;
    bool status;
    float front;
};

#endif