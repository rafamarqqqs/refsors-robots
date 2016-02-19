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
#include "resources.h"
#include "occupancy_grid.h"
#include "topics.h"
#include <gazebo_msgs/ModelStates.h>
#include <actionlib/server/simple_action_server.h>
#include "robot_control/laserMeasures.h"
#include "robot_control/searchAction.h"

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#define MIN_RANGE 1.8

#define DANGER_ZONE 1.0
#define DANGER_ANGLE (M_PI/3)

#define RANGES 8

#define MAP_LENGTH 40.0
#define MAP_WIDTH 40.0
#define CELL_SIZE 0.5
#define REPULSION 1

#define QGOAL 1.0
#define QWALL 1.0
#define QOG 1.0
#define MAX_LIN_SPEED 0.4
#define MAX_ANG_SPEED 0.5

#define ERROR 1.5

#define START_OG_CALCULATION(x, y) (fabs(x) > 0.5 || fabs(y) > 0.5)

#define REACHED_DESTINATION(g, p) (fabs(g -> x - p.x) < ERROR && fabs(g -> y - p.y) < ERROR)

typedef struct drivingInfo {
    float rotation;
    float velocity;
} DrivingInfo;

typedef actionlib::SimpleActionServer<robot_control::searchAction> SearchAction; 

class Navigator {
public:
    Navigator(ros::NodeHandle node, char *type);
    virtual ~Navigator();

    void search(const robot_control::searchGoalConstPtr &goal);
    bool driveTo(_2DPoint *goal);
    _2DPoint *createGoal(float x, float y);

private:
    DrivingInfo defineDirection(_2DPoint *goal);

    void handlePose(const geometry_msgs::Pose::ConstPtr& data);

    void handleLaser(const robot_control::laserMeasures::ConstPtr& data);

    void stop();
    void driveForward();
    void drive(DrivingInfo info);
    std::list<_2DPoint>* calculateDistances(Robot* robot);
    float calculateAngle(_2DPoint *goal, std::list<_2DPoint>* wall_points, Robot* robot);

    std::list<LaserPoint> remakeRanges();

    //Laser* laser;
    OccupancyGrid *og;
    geometry_msgs::Pose pose;

    std::vector<float> range;
    std::vector<float> angle;
    float front;

    bool laser_ready;

    ros::Publisher velocity_pub;

    SearchAction searchServer;

    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;

    ros::ServiceServer service;

    ros::NodeHandle node;
};

#endif