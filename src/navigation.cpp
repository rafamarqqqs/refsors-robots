#include "../include/robot_control/navigation.h"

//Constructor

Navigator::Navigator(ros::NodeHandle n){
    node = n;

    this -> laser = new Laser();
    this -> og = new OccupancyGrid(40.0, 40.0, 0.5, 1);

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>("/larger_robot/cmd_vel", 100);

    //Subscribing to sensors
    laser_sub = node.subscribe("/larger_robot/base_scan/scan", 1, &Laser::handleSubscription, this -> laser);
    pose_sub = node.subscribe("/larger_robot/pose", 1, &Navigator::handlePose, this);
    gazebo_pose_sub = node.subscribe("/gazebo/model_states", 1, &Navigator::handleGazeboModelState, this);
}

void Navigator::handlePose(const geometry_msgs::Pose::ConstPtr& data){
    //this -> pose.position = data -> position;
    //this -> pose.orientation = data -> orientation;
}

void Navigator::handleGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr& data){
    int i = 0;

    while(data -> name[i] != "mobile_base"){        
        i++;
    }

    this -> pose = data -> pose[i];
}
//Destructor

Navigator::~Navigator(){
    delete this -> laser;
}

std::list<_2DPoint>* Navigator::calculateDistances(Robot* robot){
    std::list<LaserPoint> ranges = this -> laser -> getRanges();
    std::list<LaserPoint>::iterator it = ranges.begin();
    std::list<_2DPoint>* wall_points = new std::list<_2DPoint>();
    _2DPoint aux;
    float angle;

    while(it != ranges.end()){
        if((*it).range < 8){
            angle = Resources::angleSum((*it).angle, robot -> yaw);

            aux.x = robot -> position.x + ((*it).range * cos(angle))
                                        - (X_DISPLACEMENT * cos(robot -> yaw));
            aux.y = robot -> position.y + ((*it).range * sin(angle))
                                        - (X_DISPLACEMENT * sin(robot -> yaw));

            wall_points -> push_back(aux);
            //ROS_INFO("%3.2f %3.2f %3.2f (%2.2f %2.2f)", (*it).angle, robot -> yaw, angle, robot -> position.x, robot -> position.y);
        }

        for(int i = 0; i < 40; i++)
            it ++;
    }

    return wall_points;
}

float Navigator::calculateAngle(_2DPoint *goal, std::list<_2DPoint>* wall_points, _2DPoint robot){
    _2DPoint aux;
    std::list<_2DPoint>::iterator it;
    OGVector ogv;
    float x_component = 0, y_component = 0;
    float norm;

    //Goal attraction
    /*aux.x = goal -> x - robot -> x;
    aux.y = goal -> y - robot -> y;
    
    norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);
    
    x_component = (aux.x * QGOAL)/norm;
    y_component = (aux.y * QGOAL)/norm;
    */

    //Wall repulsion
    it = wall_points -> begin();

    while(it != wall_points -> end()){
        aux.x = (*it).x - robot.x;
        aux.y = (*it).y - robot.y;

        norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 1.5);

        x_component -= (aux.x * QWALL)/norm;
        y_component -= (aux.y * QWALL)/norm;
        
        it++;
    }
    

    ogv = og -> calculateOGVector(robot.x, robot.y);

    norm = pow(pow(ogv.x, 2) + pow(ogv.y, 2), 1.2);

    x_component -= (ogv.x * QOG)/norm;
    y_component -= (ogv.y * QOG)/norm;
        
    //ROS_INFO("OG-XY: %3.2f %3.2f", x_component, y_component);

    //ROS_INFO("OG: %3.2f %3.2f %3.2f", ogv.x, ogv.y, norm);
    //ROS_INFO("XY:%3.2f %3.2f OG:%3.2f %3.2f", x_component, y_component, -ogv.x, -ogv.y);

    return Resources::normalizeAngle(atan2(y_component, x_component));
}

DrivingInfo Navigator::defineDirection(_2DPoint *goal){
    DrivingInfo info;
    std::list<_2DPoint>* wall_points;
    Robot *robot = (Robot *) malloc(sizeof(Robot));
    float angleDiff, angleForce;

    robot -> position.x = this -> pose.position.x;
    robot -> position.y = this -> pose.position.y;
    robot -> yaw = tf::getYaw(this -> pose.orientation);

    wall_points = calculateDistances(robot);

    angleForce = calculateAngle(goal, wall_points, robot -> position);

    angleDiff = Resources::angleDiff(angleForce, Resources::transformYaw(robot -> yaw))
                                                                        *MAX_ANG_SPEED/M_PI;

    //ROS_INFO("Angle: %3.2f %3.2f", angleDiff, robot -> yaw);

    info.velocity = (this -> laser -> getFront() - 0.3)*MAX_LIN_SPEED/7.7;
    info.rotation = angleDiff;

    this -> laser -> setStatus(false);

    this -> og -> updatePosition(robot -> position.x, robot -> position.y);
    this -> og -> writeMap();

    free(robot);
    delete wall_points;

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
    while(!(laser -> isReady()) && ros::ok()){
        ros::spinOnce();
    }

    while(/*!REACHED_DESTINATION(goal, this -> pose.position) && */ros::ok()){
        while(laser -> getStatus() == false && ros::ok()){
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