#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <stdlib.h> 
#include <cmath>
#include "resources.h"
#include "occupancy_grid.h"
#include "topics.h"
#include <limits.h>
#include <gazebo_msgs/ModelStates.h>
#include <actionlib/server/simple_action_server.h>
#include "robot_control/getPositions.h"
#include "robot_control/laserMeasures.h"
#include "robot_control/searchAction.h"
#include <vector>

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

typedef actionlib::SimpleActionServer<robot_control::searchAction> SearchAction; 

class Navigator {
public:
    Navigator(ros::NodeHandle node, char *type);
    virtual ~Navigator();

    void search(const robot_control::searchGoalConstPtr &goal);

private:
    inline DrivingInfo* defineDirection(_2DPoint* goal);

    void handlePose(const geometry_msgs::PoseStamped::ConstPtr& data);

    void handleLaser(const robot_control::laserMeasures::ConstPtr& data);

    inline void stop();
    inline void driveForward();
    inline void drive(DrivingInfo* info);
    inline std::list<_2DPoint>* calculateDistances(Robot* robot);
    inline double calculateAngle(_2DPoint* goal, std::list<_2DPoint>* wall_points, Robot* robot);

    inline std::list<LaserPoint>* remakeRanges();

    //Laser* laser;
    OccupancyGrid *og;
    geometry_msgs::Pose pose;

    std::vector<double> range;
    std::vector<double> angle;
    
    float qwall;
    float qgoal;
    float qog;
    float qtail;
    float max_lin_speed;
    float max_ang_speed;
    float min_dist;
    float critical_wall_dist;
    double front;

    std::string type;

    int8_t id;

    ros::Publisher velocity_pub;

    SearchAction searchServer;

    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;

    ros::ServiceClient client;

    ros::NodeHandle node;
};

#endif