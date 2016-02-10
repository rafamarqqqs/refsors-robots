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
#include "occupancy_grid.h"
#include <gazebo_msgs/ModelStates.h>



#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#define QGOAL 1.0
#define QWALL 1.0
#define QOG 1.0
#define MAX_LIN_SPEED 0.7
#define MAX_ANG_SPEED 0.5

#define ERROR 1.5

#define START_OG_CALCULATION(x, y) (fabs(x) > 0.5 || fabs(y) > 0.5)

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

    void handlePose(const geometry_msgs::Pose::ConstPtr& data);

    void handleGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr& data);

    void stop();
    void driveForward();
    void drive(DrivingInfo info);
    std::list<_2DPoint>* calculateDistances(Robot* robot);
    float calculateAngle(_2DPoint *goal, std::list<_2DPoint>* wall_points, _2DPoint robot);

    Laser* laser;
    OccupancyGrid *og;
    geometry_msgs::Pose pose;

    ros::Publisher velocity_pub;

    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber gazebo_pose_sub;

    ros::NodeHandle node;
};

#endif