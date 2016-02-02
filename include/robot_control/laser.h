#include <list>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h> 

#ifndef _LASER_H_
#define _LASER_H_


#define RAD_10 0.174533
#define INCREMENT 0.523599
#define RANGES 8

#define LASER_NOT_INITIALIZED(laser) (laser -> angle_increment == -1 && \
                                      laser -> angle_min == -1 &&       \
                                      laser -> angle_max == -1 &&       \
                                      laser -> measures == -1)

typedef struct _2dpoint {
    float x;
    float y;
} _2DPoint;

typedef struct _2dvector {
	_2DPoint point;
	float norm;
} _2DVector;

class Laser {
public:
	Laser();
    Laser(float angle_increment, float angle_min, float angle_max, int measures);
    virtual ~Laser();

    void handleSubscription(const sensor_msgs::LaserScan::ConstPtr &laser_data);

    void subscribeTo(ros::NodeHandle node, const std::string topic, int rate, Laser *laser, ros::Subscriber sub);

    //int getIndex(float min, float max, float angle, float base);

    void setAngleIncrement(float angle_increment);
    void setAngleMax(float angle_max);
    void setAngleMin(float angle_min);
    void setMeasures(int measures);
    void setStatus(bool status);

    float getAngleIncrement();
    float getAngleMax();
    float getAngleMin();
    std::list<float> getRanges(); 
    int getMeasures();
    bool getStatus();
    float getRadius();
    float getFront();

    float base;

private:
    std::list<float> ranges;
    float angle_increment;
    float angle_min;
    float angle_max;
    int measures;
    bool status;
    float front;
};

#endif