#include "../include/robot_control/navigation.h"


//Constructor

Navigator::Navigator(ros::NodeHandle n){
    node = n;

    this -> laser = new Laser();
    //this -> localizator = new Localizator();

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>("/larger_robot/cmd_vel", 100);

    //Subscribing to sensors
    laser_sub = node.subscribe("/larger_robot/base_scan/scan", 1, &Laser::handleSubscription, this -> laser);
    //imu_sub = node.subscribe("/imu/data", 1, &Navigator::handleIMU, this);
    //odom_sub = node.subscribe("/larger_robot/odometry/filtered", 1, &Navigator::handleOdom, this);
    pose_sub = node.subscribe("/larger_robot/pose", 1, &Navigator::handlePose, this);
}

void Navigator::handlePose(const geometry_msgs::Pose::ConstPtr& data){
    this -> pose.position = data -> position;
    this -> pose.orientation = data -> orientation;
}

void Navigator::handleIMU(const sensor_msgs::Imu::ConstPtr& data){

    /*listener.waitForTransform("/base_link", "/imu_link", ros::Time(0), ros::Duration(0));
    listener.lookupTransform("/base_link", "/imu_link", ros::Time(0), transform);

    tf::Quaternion q(transform.getRotation().x(), 
                     transform.getRotation().y(),
                     transform.getRotation().z(),
                     transform.getRotation().w());

    this -> yaw = tf::getYaw(q);*/
    //float aux = tf::getYaw(q);
    //this -> yaw = Resources::angleSum(aux, tf::getYaw(data -> orientation));
}

void Navigator::handleOdom(const nav_msgs::Odometry::ConstPtr& odom){
    //this -> pose = odom -> pose.pose;
    //this -> yaw = tf::getYaw(odom -> pose.pose.orientation);
}

//Destructor

Navigator::~Navigator(){
    delete this -> laser;
    //delete this -> localizator;
}

std::list<_2DPoint>* Navigator::calculateDistances(_2DPoint* robot, float yaw){
    std::list<LaserPoint> ranges = this -> laser -> getRanges();
    std::list<LaserPoint>::iterator it = ranges.begin();
    std::list<_2DPoint>* wall_points = new std::list<_2DPoint>();
    _2DPoint aux;
    float angle;

    while(it != ranges.end()){
        if((*it).range < 8){
            angle = Resources::angleSum((*it).angle, yaw);

            aux.x = robot -> x + ((*it).range * cos(angle)) - X_DISPLACEMENT;
            aux.y = robot -> y + ((*it).range * sin(angle)) - Y_DISPLACEMENT;

            wall_points -> push_back(aux);
        }

        it++;
    }

    return wall_points;
}

float Navigator::calculateAngle(_2DPoint *goal, std::list<_2DPoint>* wall_points, _2DPoint *robot){
    _2DPoint aux;
    std::list<_2DPoint>::iterator it;

    float x_component, y_component;
    float norm;

    //Goal attraction
    aux.x = goal -> x - robot -> x;
    aux.y = goal -> y - robot -> y;
    
    norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);
    
    x_component = (aux.x * QGOAL)/norm;
    y_component = (aux.y * QGOAL)/norm;
    //Wall repulsion
    it = wall_points -> begin();

    while(it != wall_points -> end()){
        aux.x = (*it).x - robot -> x;
        aux.y = (*it).y - robot -> y;

        norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 1.5);

        x_component -= (aux.x * QWALL)/norm;
        y_component -= (aux.y * QWALL)/norm;

        it++;
    }

    return Resources::normalizeAngle(atan2(y_component, x_component));
}

DrivingInfo Navigator::defineDirection(_2DPoint *goal){
    DrivingInfo info;
    std::list<_2DPoint>* wall_points;
    _2DPoint *robot = (_2DPoint *) malloc(sizeof(_2DPoint));
    float yaw, angleDiff, angleForce;

    //robot = localizator -> getPosition();
    //yaw = Resources::transformYaw(localizator -> getYaw());
    //yaw = localizator -> getYaw();
    robot -> x = this -> pose.position.x;
    robot -> y = this -> pose.position.y;
    yaw = tf::getYaw(this -> pose.orientation);

    wall_points = calculateDistances(robot, yaw);

    angleForce = calculateAngle(goal, wall_points, robot);

    angleDiff = Resources::angleDiff(angleForce, Resources::transformYaw(yaw))*MAX_ANG_SPEED/M_PI;

    info.rotation = angleDiff;
    info.velocity = (this -> laser -> getFront())*MAX_LIN_SPEED/7.7;

    delete wall_points;

    this -> laser -> setStatus(false);

    free(robot);

    return info;
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

void Navigator::drive(DrivingInfo info){
    geometry_msgs::Twist msg;
    msg.angular.z = info.rotation;
    msg.linear.x = info.velocity;
    velocity_pub.publish(msg);
}

/*
    Drive to a given goal
*/

bool Navigator::driveTo(_2DPoint *goal){
    DrivingInfo info;
    
    //Waits for the laser to set the initial values
    ROS_INFO("waiting for laser");

    while(!REACHED_DESTINATION(goal, this -> pose.position) && ros::ok()){
        while(this -> laser -> getStatus() == false && ros::ok()){
            ros::spinOnce();
        }

        info = this -> defineDirection(goal);
        this -> drive(info);
        ros::spinOnce();
    }

    this -> stop();

    free(goal);

    ROS_INFO("Reached destination");

return true;
}

/*
    Creates a goal
*/

_2DPoint *Navigator::createGoal(float x, float y){
    _2DPoint *g = (_2DPoint *) malloc(sizeof(_2DPoint));

    g -> x = x;
    g -> y = y;

    return g;
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "navigation");

    ros::NodeHandle node;

    Navigator *nav = new Navigator(node);
    ros::spinOnce();

    ROS_INFO("Navigator started.");

    _2DPoint *goal = nav -> createGoal(-3, 10);

    nav -> driveTo(goal);

    delete nav;
}