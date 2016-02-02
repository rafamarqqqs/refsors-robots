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
#include "localization.h"

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#define MAX_X(n, x) (fabs(n) < x ? n : (n/n)*x)

#define SQUARE(n) (n*n)

/*
    SECTORS
*/

#define GET_DESTINATION_SECTOR(g) (g -> destiny.x >= 0 && g -> destiny.y >= 0 ? FIRST_SECTOR : \
                         (g -> destiny.x <= 0 && g -> destiny.y >= 0 ? SECOND_SECTOR : \
                         (g -> destiny.x <= 0 && g -> destiny.y <= 0 ? THIRD_SECTOR : FOURTH_SECTOR)))

#define GET_SECTOR(g, p) (p -> x >= g -> destiny.x && p -> y >= g -> destiny.y ? \
                        FIRST_SECTOR : \
                         (p -> x <= g -> destiny.x && p -> y >= g -> destiny.y ? \
                        SECOND_SECTOR : \
                         (p -> x <= g -> destiny.x && p -> y <= g -> destiny.y ? \
                        THIRD_SECTOR : FOURTH_SECTOR)))

#define REACHED_DESTINATION(g, p) (g -> destination_sector == GET_SECTOR(g, p))

enum Sector {
    FIRST_SECTOR,
    SECOND_SECTOR,
    THIRD_SECTOR,
    FOURTH_SECTOR
};

typedef struct goal {
    _2DPoint component;
    _2DPoint destiny;
    Sector destination_sector;
} Goal;

typedef struct drivingInfo {
    float rotation;
    float velocity;
} DrivingInfo;

class Navigator {
public:
    Navigator(ros::NodeHandle node);
    virtual ~Navigator();

    bool driveTo(Goal *goal);
    Goal *createGoal(float x, float y);

private:
    void handleTf(const tf::tfMessage::ConstPtr &tf_data);

    DrivingInfo defineDirection(Goal *goal);

    void stop();
    void driveForward();
    void drive(DrivingInfo info);
    std::list<_2DPoint>* calculateDistances(_2DPoint* robot, float yaw);

    Laser *laser;
    Localization *localization;

    ros::Publisher velocity_pub;
    ros::Subscriber laser_sub;
    ros::NodeHandle node;
};

#endif