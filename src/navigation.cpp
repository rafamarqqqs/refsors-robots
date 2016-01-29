#include "../include/robot_control/navigation.h"

void Navigator::initSafetySpecs(){
    this -> safe_distances -> min_safe_angle = (this -> laser_info.angle_max - this -> laser_info.angle_min - SAFETY_ANGLE)/2;
    this -> safe_distances -> max_safe_angle = this -> safe_distances -> min_safe_angle + SAFETY_ANGLE;
    this -> safe_distances -> min_angle_safe_distance = SAFE_DISTANCE(SAFETY_ANGLE);
    this -> safe_distances -> max_angle_safe_distance = SAFE_DISTANCE(0);
}

bool Navigator::safetyCheckIsOk(){
    int i;
    
    //ROS_INFO("%.3f %.3f %.3f %.3f", this -> safe_distances -> min_angle_safe_distance, this -> safe_distances -> max_angle_safe_distance,
        //this -> safe_distances -> laser_min_safe_angle_measure, this -> safe_distances -> laser_max_safe_angle_measure);
    if(this -> safe_distances -> laser_min_safe_angle_measure < this -> safe_distances -> min_angle_safe_distance || 
       this -> safe_distances -> laser_max_safe_angle_measure < this -> safe_distances -> max_angle_safe_distance)
        return false;

    return true;
}

void Navigator::handleScan(const sensor_msgs::LaserScan::ConstPtr &laser_data){
    this -> laser_info.angle_min = laser_data -> angle_min;
    this -> laser_info.angle_max = laser_data -> angle_max;

    int i;
    float x_component = 0, y_component = 0;
    float angle = laser_data -> angle_min;
    float d;
    int start;
    float radius = laser_data -> angle_max - laser_data -> angle_min;

    int ranges = (int) floor(radius/laser_data -> angle_increment);

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

    this -> laser_ok = true;
}

void Navigator::handleGPS(const sensor_msgs::NavSatFix::ConstPtr &gps_data){
    //this -> longitude = gps_data -> longitude;
    //this -> latitude = gps_data -> latitude;
}

void Navigator::handleOrientation(const sensor_msgs::Imu::ConstPtr &orientation_data){
}

void Navigator::handlePosition(const nav_msgs::Odometry::ConstPtr &odometry_data){
    this -> pose = odometry_data -> pose.pose;
    this -> rotation -> yaw = tf::getYaw(odometry_data -> pose.pose.orientation);
    //ROS_INFO("%.2f %.2f %.2f %.2f", this -> front_distance, this -> pose.position.x, this -> pose.position.y, this -> pose.position.z);
}

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

Navigator::~Navigator(){
    free(this -> safe_distances);
}

bool Navigator::defineRotation(Goal *goal){
    geometry_msgs::Twist msg;

    float hypotenuse = sqrt(SQUARE(goal -> component.x) + SQUARE(goal -> component.y));
    float angle = acos(goal -> component.x/hypotenuse);
    RotationWay r = angle > this -> rotation -> yaw ? CLOCKWISE : COUNTERCLOCKWISE;

    this -> fallback();

    msg.angular.z = (r == CLOCKWISE ? ROTATION_SPEED : (-1)*ROTATION_SPEED);

    if(!MAX_ATTEMPTS()){

        ROS_INFO("ROTATING: %.4f %.4f\n", angle, this -> rotation -> yaw);
        while(!ROTATE_OK(angle, this -> rotation -> yaw, r) && ros::ok()){
            velocity_pub.publish(msg);
            ros::spinOnce();
        }
    }
    else{
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

    msg.angular.z = 0;
    velocity_pub.publish(msg);

    return true;
}

bool Navigator::defineDirection(Goal *goal){
    ros::spinOnce();
    
    goal -> component.x = (goal -> destiny.x - this -> pose.position.x) * GOAL_ATTRACTION;
    goal -> component.y = (goal -> destiny.y - this -> pose.position.y) * GOAL_ATTRACTION;
    
    ROS_INFO("goal(%.2f %.2f)", goal -> component.x, goal -> component.y);
    
    goal -> component.x = direction_x + (goal -> destiny.x - this -> pose.position.x) * GOAL_ATTRACTION;
    goal -> component.y = direction_y + (goal -> destiny.y - this -> pose.position.y) * GOAL_ATTRACTION;

    ROS_INFO("components(%.2f %.2f)\n", goal -> component.x, goal -> component.y);
}

void Navigator::stop(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    velocity_pub.publish(msg);
}

void Navigator::driveForward(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0.5;
    velocity_pub.publish(msg);
}

void Navigator::fallback(){
    geometry_msgs::Twist msg;

    msg.linear.x = FALLBACK;
    velocity_pub.publish(msg);
    
    ros::Duration(1).sleep();

    msg.linear.x = 0;
    velocity_pub.publish(msg);
}

bool Navigator::driveTo(Goal *goal){
    geometry_msgs::Twist msg;

    while(!laser_ok){
        ros::spinOnce();
    }

    goal -> destination_sector = GET_DESTINATION_SECTOR(goal);


    REMAKE_ATTEMPTS();

    this -> initSafetySpecs();
    
    while(!REACHED_DESTINATION(goal, this -> pose) && ros::ok()){
        ROS_INFO("Turning (attempt %d) (%.2f %.2f)\n", this -> rotation -> attempts, this -> pose.position.x, this -> pose.position.y);
        
        this -> stop();
        this -> defineDirection(goal);
        this -> defineRotation(goal);

        INCREASE_ATTEMPTS();

        if(safetyCheckIsOk() && !REACHED_DESTINATION(goal, this -> pose) && ros::ok()){
            REMAKE_ATTEMPTS();
            ROS_INFO("Driving (%.2f %.2f)\n", this -> pose.position.x, this -> pose.position.y);
            ROS_INFO("%d %d", goal -> destination_sector, GET_SECTOR(goal, this -> pose));

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

Goal *createGoal(float x, float y){
    Goal *g = (Goal *) malloc(sizeof(Goal));

    g -> destiny.x = x;
    g -> destiny.y = y;

    g -> component.x = 0;
    g -> component.y = 0;

    return g;
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "navigation");

    ros::NodeHandle node;

    Navigator *nav = new Navigator(node);
    ros::spinOnce();

    ROS_INFO("Navigator started.");

    Goal *goal = createGoal(4, 4);

    nav -> driveTo(goal);

    delete nav;
}