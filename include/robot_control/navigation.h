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
#include <geometry_msgs/Quaternion.h>
#include <actionlib/server/simple_action_server.h>
#include "robot_control/getPositions.h"
#include "robot_control/laserMeasures.h"
#include "robot_control/searchAction.h"
#include "robot_control/driveToAction.h"
#include "robot_control/alignWithBombAction.h"
#include "robot_control/getBombDisplacement.h"
#include <vector>

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

typedef actionlib::SimpleActionServer<robot_control::searchAction> SearchAction;
typedef actionlib::SimpleActionServer<robot_control::alignWithBombAction> AlignWithBombAction;
typedef actionlib::SimpleActionServer<robot_control::driveToAction> DriveToAction; 

class Navigator {
public:
    Navigator(ros::NodeHandle node, char *type);
    virtual ~Navigator();

    void search(const robot_control::searchGoalConstPtr& goal);
    void searchPreempted();
    void driveToPreempted();
    void driveTo(const robot_control::driveToGoalConstPtr& goal);
    void alignWithBomb(const robot_control::alignWithBombGoalConstPtr& goal);

private:
    inline void defineDirection();

    void handlePose(const geometry_msgs::PoseStamped::ConstPtr& data);

    void handleLaser(const robot_control::laserMeasures::ConstPtr& data);

    inline void getGoalAttraction(double* x_component, double* y_component);

    inline void getWallsRepulsion(double* x_component, double* y_component, std::list<_2DPoint>* wall_points);
    inline void getRobotsRepulsion(double* x_component, double* y_component);
    inline void stop();
    inline void driveForward();
    inline void drive();
    inline std::list<_2DPoint>* calculateDistances();
    inline double calculateAngle(std::list<_2DPoint>* wall_points);

    inline std::list<LaserPoint>* remakeRanges();

    OccupancyGrid *og;

    Robot* robot;

    std::vector<double> range;
    std::vector<double> angle;
    
    double qwall;
    double qrobots;
    double qgoal;
    double qog;
    double qtail;
    double max_lin_speed;
    double max_ang_speed;
    double min_dist;
    double critical_wall_dist;
    double front;
    double navigation_error;
    double min_repulsion;
    _2DPoint* goal;

    bool found;
    bool driving;

    std::string type;

    DrivingInfo* driving_info;

    int8_t id;

    ros::Publisher velocity_pub;

    SearchAction searchServer;
    DriveToAction driveToServer;
    AlignWithBombAction alignWithBombServer;

    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;

    ros::ServiceClient client;

    ros::NodeHandle node;
};

#endif