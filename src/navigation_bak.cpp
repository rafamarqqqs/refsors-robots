#include "../include/robot_control/navigation.h"

//Constructor

Navigator::Navigator(ros::NodeHandle n){
    node = n;

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>("/larger_robot/cmd_vel", 100);

    this -> safe_distances = (SafetySpecs *) malloc(sizeof(SafetySpecs));
    this -> laser_ok = false;

    this -> rotation = (Rotation *) malloc(sizeof(Rotation));
    this -> rotation -> attempts = 0;
    
    //Subscribing to sensors
    laser = node.subscribe("/larger_robot/scan", 100, &Navigator::handleScan, this);
    
    //gps = node.subscribe("/navsat/fix", 100, &Navigator::handleGPS, this);
    orientation = node.subscribe("/imu/data", 100, &Navigator::handleOrientation, this);
    position = node.subscribe("/larger_robot/odometry/filtered", 100, &Navigator::handlePosition, this);
}

//Destructor

Navigator::~Navigator(){
    free(this -> safe_distances);
    free(this -> rotation);
}

/*

    SAFETY CHECK

*/

void Navigator::initSafetyConfigs(){
    //Initialize safety configurations
    this -> safe_distances -> min_safe_angle = (this -> laser_info.angle_max - this -> laser_info.angle_min - SAFETY_ANGLE)/2;
    this -> safe_distances -> max_safe_angle = this -> safe_distances -> min_safe_angle + SAFETY_ANGLE;
    this -> safe_distances -> min_angle_safe_distance = SAFE_DISTANCE(SAFETY_ANGLE);
    this -> safe_distances -> max_angle_safe_distance = SAFE_DISTANCE(0);
}

bool Navigator::safetyCheckIsOk(){
    int i;
    
    //Verify if safety conditions are satisfied while the laser measures changes
    if(this -> safe_distances -> laser_min_safe_angle_measure < this -> safe_distances -> min_angle_safe_distance || 
       this -> safe_distances -> laser_max_safe_angle_measure < this -> safe_distances -> max_angle_safe_distance)
        return false;

    return true;
}

/*
    
    LASER SENSOR

*/

void Navigator::handleScan(const sensor_msgs::LaserScan::ConstPtr &laser_data){
    //Store some info for later use by other functions
    this -> laser_info.angle_min = laser_data -> angle_min;
    this -> laser_info.angle_max = laser_data -> angle_max;

    int i;
    float x_component = 0, y_component = 0;
    float angle = laser_data -> angle_min;
    float d;
    int start;
    float radius = laser_data -> angle_max - laser_data -> angle_min;

    //Calculates the number of measures of the laser sensor
    int ranges = (int) floor(radius/laser_data -> angle_increment);

    //Store all safe distances for each laser in the SAFETY_ANGLE
    for(i = 0; i < ranges; i++){
        if(i*laser_data -> angle_increment > this -> safe_distances -> min_safe_angle){
            this -> safe_distances -> laser_min_safe_angle_measure = laser_data -> ranges[i-1];
            break;
        }
    }

    for(; i < ranges; i++){
        if(i*laser_data -> angle_increment > this -> safe_distances -> max_safe_angle){
            this -> safe_distances -> laser_max_safe_angle_measure = laser_data -> ranges[i];
            break;
        }
    }

    //Gets the distance from each sensor, decomposes it, scales and sums all components in the same direction
    for(i = 0; i < ranges; i++){
        d = laser_data -> ranges[i];
        x_component += fabs(30 - d) * cos(angle) * COEFFICIENT(d) * GET_STRENGTH() * X_COEFFICIENT *
                           SIDE_BONUS(angle - laser_data -> angle_min, radius) * SAFE_ZONE_BONUS(this -> 
                           safe_distances -> min_safe_angle, this -> safe_distances -> max_safe_angle,
                           angle);
            
        y_component += fabs(30 - d) * sin(angle) * COEFFICIENT(d) * GET_STRENGTH() * Y_COEFFICIENT *
                           SIDE_BONUS(angle - laser_data -> angle_min, radius) * SAFE_ZONE_BONUS(this -> 
                           safe_distances -> min_safe_angle, this -> safe_distances -> max_safe_angle,
                           angle);

        angle += laser_data -> angle_increment;
    }

    this -> direction_x = x_component;
    this -> direction_y = y_component;

    //Sets the laser_ok flag to true
    //Functions can now safely use data that is changed in this function
    this -> laser_ok = true;
}

void Navigator::handleGPS(const sensor_msgs::NavSatFix::ConstPtr &gps_data){
    //this -> longitude = gps_data -> longitude;
    //this -> latitude = gps_data -> latitude;
}

void Navigator::handleOrientation(const sensor_msgs::Imu::ConstPtr &orientation_data){
}

/*

    ODOMETRY

*/

void Navigator::handlePosition(const nav_msgs::Odometry::ConstPtr &odometry_data){
    //Gets the estimated position and Z axis rotation of the robot
    this -> pose = odometry_data -> pose.pose;
    this -> rotation -> yaw = tf::getYaw(odometry_data -> pose.pose.orientation);
}


/*

    ROTATION - defines the angle to rotate the robot, and rotate it.
    It succeeds when all the laser measures in the front of the robot (safety zone) are no longer closer 
    than 1m from the object

*/

bool Navigator::defineRotation(Goal *goal){
    geometry_msgs::Twist msg;

    //Trigonometry
    float hypotenuse = sqrt(SQUARE(goal -> component.x) + SQUARE(goal -> component.y));
    float angle = acos(goal -> component.x/hypotenuse);

    //Gets the right way to rotate
    RotationWay r = angle > this -> rotation -> yaw ? CLOCKWISE : COUNTERCLOCKWISE;

    //Pulls back the robot a little
    this -> fallback();

    //Msg that tells the robot to rotate in the Z axis
    msg.angular.z = (r == CLOCKWISE ? ROTATION_SPEED : (-1)*ROTATION_SPEED);

    if(!MAX_ATTEMPTS()){

        ROS_INFO("ROTATING: %.4f %.4f\n", angle, this -> rotation -> yaw);

        //Rotates until the robot yaw is correct with the given angle
        while(!ROTATE_OK(angle, this -> rotation -> yaw, r) && ros::ok()){
            velocity_pub.publish(msg);
            ros::spinOnce();
        }
    }
    else{
        //If all attempts do not succeed in rotating the robot, it rotates indefinitly until
        //the conditions are satisfied

        ROS_INFO("UNDEFINED ROTATION\n");
        
        float _hypotenuse = sqrt(SQUARE(goal -> destiny.x) + SQUARE(goal -> destiny.y));
        float _angle = acos(goal -> destiny.x/hypotenuse);
        RotationWay _r = angle > this -> rotation -> yaw ? CLOCKWISE : COUNTERCLOCKWISE;
        msg.angular.z = (_r == CLOCKWISE ? ROTATION_SPEED : (-1)*ROTATION_SPEED);

        while(!safetyCheckIsOk() && ros::ok()){
            velocity_pub.publish(msg);
            ros::spinOnce();
        }
        
    }

    return true;
}

/*

    DIRECTION - sums the attraction vector of the goal with the components acquired by the laser, after
    scaling it accordingly

*/


bool Navigator::defineDirection(Goal *goal){
    ros::spinOnce();
    
    goal -> component.x = direction_x + (goal -> destiny.x - this -> pose.position.x) * GOAL_ATTRACTION;
    goal -> component.y = direction_y + (goal -> destiny.y - this -> pose.position.y) * GOAL_ATTRACTION;

    ROS_INFO("components(%.2f %.2f)\n", goal -> component.x, goal -> component.y);
}

/*
    Stops the robot
*/

void Navigator::stop(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    velocity_pub.publish(msg);
}

/*
    Drives forward
*/

void Navigator::driveForward(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0.5;
    velocity_pub.publish(msg);
}

/*
    Pushes the robot back
*/

void Navigator::fallback(){
    geometry_msgs::Twist msg;

    msg.linear.x = FALLBACK;
    velocity_pub.publish(msg);
    
    ros::Duration(1).sleep();

    msg.linear.x = 0;
    velocity_pub.publish(msg);
}

/*

    Drive to a given goal

*/

bool Navigator::driveTo(Goal *goal){
    geometry_msgs::Twist msg;

    //Waits for the laser to set the initial values
    while(!laser_ok){
        ros::spinOnce();
    }

    //0 attempts to rotate the robot
    REMAKE_ATTEMPTS();

    //Initialize safety configurations
    this -> initSafetyConfigs();
    
    while(!REACHED_DESTINATION(goal, this -> pose) && ros::ok()){
        ROS_INFO("Turning (attempt %d) (%.2f %.2f)\n", this -> rotation -> attempts, this -> pose.position.x, this -> pose.position.y);
        
        //Stops, gets the direction, rotates accordingly
        this -> stop();
        this -> defineDirection(goal);
        this -> defineRotation(goal);

        //More one attempt to turn the robot
        INCREASE_ATTEMPTS();

        //If rotated succesfully
        if(safetyCheckIsOk() && !REACHED_DESTINATION(goal, this -> pose) && ros::ok()){
            REMAKE_ATTEMPTS();

            ROS_INFO("Driving (%.2f %.2f)\n", this -> pose.position.x, this -> pose.position.y);
            ROS_INFO("%d %d", goal -> destination_sector, GET_SECTOR(goal, this -> pose));

            //Drives forward until the safety check is not ok
            while(safetyCheckIsOk() && ros::ok()){
                this -> driveForward();
                ros::spinOnce();
            }
        }
    }

    this -> stop();

    free(goal);

    ROS_INFO("Reached destination");

return true;
}

/*
    Creates a goal object
*/

Goal *Navigator::createGoal(float x, float y){
    Goal *g = (Goal *) malloc(sizeof(Goal));

    g -> destiny.x = x;
    g -> destiny.y = y;

    //Axis attraction components
    g -> component.x = 0;
    g -> component.y = 0;

    //Destination sector with center (g -> destiny.x, g -> destiny.y)
    g -> destination_sector = GET_DESTINATION_SECTOR(g);

    return g;
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "navigation");

    ros::NodeHandle node;

    Navigator *nav = new Navigator(node);
    ros::spinOnce();

    ROS_INFO("Navigator started.");

    Goal *goal = nav -> createGoal(4, 4);

    nav -> driveTo(goal);

    delete nav;
}