#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdlib.h> 
#include <cmath>
#include "laser.h"
#include "resources.h"
//#include "localization.h"
//#include "mapping.h"



#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#define QGOAL 1.0
#define QWALL 1.0
#define MAX_LIN_SPEED 0.6
#define MAX_ANG_SPEED 0.5

#define ERROR 1.5

/*
    SECTORS
*/

#define REACHED_DESTINATION(g, p) (fabs(g -> x - p.x) < ERROR && fabs(g -> y - p.y) < ERROR)

typedef struct drivingInfo {
    float rotation;
    float velocity;
} DrivingInfo;

class Navigator {
public:
    Navigator(ros::NodeHandle node);
    virtual ~Navigator();

    bool driveTo(_2DPoint *goal);
    _2DPoint *createGoal(float x, float y);

private:
    DrivingInfo defineDirection(_2DPoint *goal);

    void handleIMU(const sensor_msgs::Imu::ConstPtr& data);
    
    void handleOdom(const nav_msgs::Odometry::ConstPtr& data);

    void handlePose(const geometry_msgs::Pose::ConstPtr& data);

    void stop();
    void driveForward();
    void drive(DrivingInfo info);
    std::list<_2DPoint>* calculateDistances(_2DPoint* robot, float yaw);
    float calculateAngle(_2DPoint *goal, std::list<_2DPoint>* wall_points, _2DPoint *robot);

    Laser *laser;
    //Localizator *localizator;
    geometry_msgs::Pose pose;

    ros::Publisher velocity_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber pose_sub;
    ros::NodeHandle node;

    tf::StampedTransform transform;
    tf::TransformListener listener;
};

#endif