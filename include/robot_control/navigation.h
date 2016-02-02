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

typedef struct position {
    geometry_msgs::Pose pose;
    float yaw;
} Position;

class Navigator {
public:
    Navigator(ros::NodeHandle node);
    virtual ~Navigator();

    bool driveTo(Goal *goal);
    Goal *createGoal(float x, float y);

private:
    //void handleScan(const sensor_msgs::LaserScan::ConstPtr &laser_data);
    //void handleSubscription(const sensor_msgs::LaserScan::ConstPtr &laser_data);

    void handleTf(const tf::tfMessage::ConstPtr &tf_data);

    void handleOrientation(const sensor_msgs::Imu::ConstPtr &imu_data);
    
    void handlePosition(const nav_msgs::Odometry::ConstPtr &odometry_data);

    DrivingInfo defineDirection(Goal *goal);

    bool acquire_laser_lock();
    bool release_laser_lock();

    void stop();
    void fallback();
    void driveForward();
    void drive(DrivingInfo info);

    Laser *laser;
    Position *position;
    geometry_msgs::Quaternion quaternion_orientation;
    Localization *localization;

    float direction_x, direction_y;

    ros::Publisher velocity_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;
    ros::NodeHandle node;
};

#endif