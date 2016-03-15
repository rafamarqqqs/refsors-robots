#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#define RANGES 8

#define MAP_LENGTH 40.0
#define MAP_WIDTH 40.0
#define CELL_SIZE 1
#define REPULSION 30
#define AREA_SIZE 5

#define MEASURES 480

#define QGOAL 0.8
#define QWALL 0.3
#define QOG 0.8
#define QTAIL 0.4
#define MAX_LIN_SPEED 0.4
#define MAX_ANG_SPEED 0.5

//seconds
#define TIME_LIMIT (5 * 60)

#define ERROR 4.0
#define MIN_DIST 6

#define START_OG_CALCULATION(x, y) (fabs(x) > 0.5 || fabs(y) > 0.5)

#define REACHED_DESTINATION(g, p) (fabs(g -> x - p.x) < ERROR && fabs(g -> y - p.y) < ERROR)

#define REACHED_TIME_LIMIT(start, now, g, p) (difftime(now, start) > TIME_LIMIT ? true : false)
/*
#define REACHED_TIME_LIMIT(start, now, g, p) ((difftime(now, start) > TIME_LIMIT) && \
                                                 (fabs(g -> x - p.x) < MIN_DIST && fabs(g -> y - p.y) < MIN_DIST) \
                                                 ? true : false)
*/
typedef struct drivingInfo {
    float rotation;
    float velocity;
} DrivingInfo;

//typedef message_filters::sync_policies::ApproximateTime<robot_control::laserMeasures, geometry_msgs::PoseStamped> SyncPolicy;

typedef actionlib::SimpleActionServer<robot_control::searchAction> SearchAction; 

class Navigator {
public:
    Navigator(ros::NodeHandle node, char *type);
    virtual ~Navigator();

    void search(const robot_control::searchGoalConstPtr &goal);
    _2DPoint *createGoal(float x, float y);

private:
    DrivingInfo defineDirection(_2DPoint *goal);

    void handlePose(const geometry_msgs::PoseStamped::ConstPtr& data);

    /*void syncCallback(const robot_control::laserMeasures::ConstPtr& laser_data, 
        const geometry_msgs::PoseStamped::ConstPtr& pose_data);*/

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

    float min_range;

    std::string type;

    bool laser_ready;
    bool localization_ready;

    ros::Publisher velocity_pub;

    SearchAction searchServer;

    //message_filters::Synchronizer<SyncPolicy> sync;

    //message_filters::Subscriber<robot_control::laserMeasures> laser_sub;
    //message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;

    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;

    ros::NodeHandle node;
};

#endif