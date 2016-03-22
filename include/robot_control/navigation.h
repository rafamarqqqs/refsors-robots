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

#define RANGES 8

#define MAP_LENGTH 40.0
#define MAP_WIDTH 40.0
#define SMALLER_ROBOT_CELL_SIZE 1.0
#define LARGER_ROBOT_CELL_SIZE 1.0
#define REPULSION 30

#define AREA_SIZE 8

#define ROBOTS_MIN_DIST 3;

#define LASER_DISPLACEMENT(type) (strcmp(type, "larger_robot") == 0 ? 0.371 : 0.1075)

#define LOCALIZATION_STARTED ((pose.position.x != 0 || pose.position.y != 0) ? true : false)
#define LASER_STARTED ((range.size() != 0 && angle.size() != 0) ? true : false)

#define MEASURES 480

#define SQUARE(x) (x*x)
#define TO_THE_FOURTH(x) (x*x*x)

#define QROBOTS 1.0
#define QGOAL 0.8
#define QWALL 0.8
#define QOG 0.8
#define QTAIL 0.4
#define MAX_LIN_SPEED 0.7
#define MAX_ANG_SPEED 0.9

//seconds
#define TIME_LIMIT (3 * 60)

#define ERROR 3.0
#define MIN_DIST 8

#define REACHED_DESTINATION(g, p) (fabs(g -> x - p.x) < ERROR && fabs(g -> y - p.y) < ERROR)

#define REACHED_TIME_LIMIT(start, now) ((now - start) > TIME_LIMIT ? true : false)

typedef struct drivingInfo {
    double rotation;
    double velocity;
} DrivingInfo;

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

    int id;

    ros::Publisher velocity_pub;

    SearchAction searchServer;

    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;

    ros::ServiceClient client;

    ros::NodeHandle node;
};

#endif