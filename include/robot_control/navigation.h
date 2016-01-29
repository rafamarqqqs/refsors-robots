#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <stdlib.h> 
#include <math.h>
#include <tf/transform_datatypes.h>

#define SQUARE(n) (n*n)

/*
    SECTORS
*/

#define GET_DESTINATION_SECTOR(g) (g -> destiny.x >= 0 && g -> destiny.y >= 0 ? FIRST_SECTOR : \
                         (g -> destiny.x <= 0 && g -> destiny.y >= 0 ? SECOND_SECTOR : \
                         (g -> destiny.x <= 0 && g -> destiny.y <= 0 ? THIRD_SECTOR : FOURTH_SECTOR)))

#define GET_SECTOR(g, p) (p.position.x >= g -> destiny.x && p.position.y >= g -> destiny.y ? \
                        FIRST_SECTOR : \
                         (p.position.x <= g -> destiny.x && p.position.y >= g -> destiny.y ? \
                        SECOND_SECTOR : \
                         (p.position.x <= g -> destiny.x && p.position.y <= g -> destiny.y ? \
                        THIRD_SECTOR : FOURTH_SECTOR)))

#define REACHED_DESTINATION(g, p) (g -> destination_sector == GET_SECTOR(g, p))

/*
    ATTRACTION COEFFICIENTS
*/

#define GOAL_ATTRACTION 120
#define COEFFICIENT(d) (d < 1 ? 0.8 : (d < 2 ? 0.5 : (d < 10 ? 0.3 : 0.1)))
#define X_COEFFICIENT 0.2
#define Y_COEFFICIENT 1.2

#define SIDE_BONUS(angle, radius) (((radius-angle)/radius > 0.85 || (radius-angle)/radius < 0.15) ? 5 : 1)

#define SAFE_ZONE_BONUS(min, max, angle) ((angle >= min && angle <= max ? 1 : 0.9))

/*
    ROTATION ATTEMPTS
*/

#define FIRST_ATTEMPT() (this -> rotation -> attempts == 0)
#define INCREASE_ATTEMPTS() (this -> rotation -> attempts++)
#define REMAKE_ATTEMPTS() (this -> rotation -> attempts = 0)
#define MAX_ATTEMPTS() (this -> rotation -> attempts == 5 ? true : false)

#define NONE 1.0
#define LOW 1.5
#define MEDIUM 2.0
#define HIGH 3.0
#define EXTREME 4.0

#define GET_STRENGTH() (this -> rotation -> attempts == 0 ? NONE : \
                        (this -> rotation -> attempts == 1 ? LOW : \
                        (this -> rotation -> attempts == 2 ? MEDIUM : \
                        (this -> rotation -> attempts == 3 ? HIGH : EXTREME))))

#define ROTATION_SPEED 0.15
#define ROTATE_OK(angle, yaw, way) (way == CLOCKWISE ? (angle < yaw) : (angle > yaw))

#define FALLBACK -0.2 * GET_STRENGTH()

/*
    SAFETY CONFIGS
*/

#define ROBOT_FRONT_SIZE 0.670
#define MAX_SAFE_DISTANCE (sqrt(SQUARE(ROBOT_FRONT_SIZE/2) + 1))
#define SAFETY_ANGLE (2*asin((ROBOT_FRONT_SIZE/2)/MAX_SAFE_DISTANCE))
#define SAFE_DISTANCE(angle) (1/(cos((SAFETY_ANGLE/2) - angle)))

enum Sector {
    FIRST_SECTOR,
    SECOND_SECTOR,
    THIRD_SECTOR,
    FOURTH_SECTOR
};

enum RotationWay {
    CLOCKWISE,
    COUNTERCLOCKWISE
};

typedef struct rotation {
    float yaw;
    int attempts;
} Rotation;


/*
    SAFE CONDITIONS

    Robot with the laser ranges.
    This figure explains the safe conditions



                             1m     |
                           -------  |
    xxxxxxxxxxxxxxxxxxxxxxx  /   / /|
    x                     x /  / / /|
    x                     x/ /  / / |
    x                     x\ \  \ \ |
    x                     x \  \ \ \|
    xxxxxxxxxxxxxxxxxxxxxxx  \   \ \|
                                    |
                                    |
*/

typedef struct safety {
    float min_safe_angle;
    float max_safe_angle;
    float min_angle_safe_distance;
    float max_angle_safe_distance;
    float laser_min_safe_angle_measure;
    float laser_max_safe_angle_measure;
} SafetySpecs;

typedef struct _2dpoint {
    float x;
    float y;
} _2DPoint;

typedef struct goal {
    _2DPoint component;
    _2DPoint destiny;
    Sector destination_sector;
} Goal;

typedef struct laser {
    float angle_max;
    float angle_min;
    float angle_increment;
    float ranges;
} LaserInfo;

class Navigator {
public:
    Navigator(ros::NodeHandle node);
    virtual ~Navigator();

    bool driveTo(Goal *goal);
    Goal *createGoal(float x, float y);


private:
    void handleScan(const sensor_msgs::LaserScan::ConstPtr &laser_data);

    void handleGPS(const sensor_msgs::NavSatFix::ConstPtr &gps_data);

    void handleOrientation(const sensor_msgs::Imu::ConstPtr &orientation_data);
  
    void handlePosition(const nav_msgs::Odometry::ConstPtr &odometry_data);

    bool defineRotation(Goal *goal);

    bool defineDirection(Goal *goal);

    bool acquire_laser_lock();
    bool release_laser_lock();

    void initSafetyConfigs();
    bool safetyCheckIsOk();

    void stop();
    void fallback();
    void driveForward();

    geometry_msgs::Quaternion quaternion_orientation;
    geometry_msgs::Pose pose;
    //sensor_msgs::LaserScan::ConstPtr laser_data;

    SafetySpecs *safe_distances;
    Rotation *rotation;
    LaserInfo laser_info;

    float direction_x, direction_y;
    bool laser_ok;

    ros::Publisher velocity_pub;
    ros::Subscriber laser;
    ros::Subscriber orientation;
    //ros::Subscriber gps;
    ros::Subscriber position;
    ros::NodeHandle node;

};