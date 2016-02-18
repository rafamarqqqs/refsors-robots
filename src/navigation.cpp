#include "../include/robot_control/navigation.h"

//Constructor

Navigator::Navigator(ros::NodeHandle n){
    node = n;

    this -> laser = new Laser();
    this -> og = new OccupancyGrid(MAP_LENGTH, MAP_WIDTH, CELL_SIZE, REPULSION, -14, 16);

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>(LARGER_ROBOT_VEL, 100);

    ROS_INFO("subscribing to services");

    //Subscribing to sensors
    laser_sub = node.subscribe(LARGER_ROBOT_SCAN, 1, &Laser::handleSubscription, this -> laser);
    pose_sub = node.subscribe(LARGER_ROBOT_POSE, 1, &Navigator::handlePose, this);

    service = node.advertiseService("search", &Navigator::search, this);
}

void Navigator::handlePose(const geometry_msgs::Pose::ConstPtr& data){
    this -> pose.position = data -> position;
    this -> pose.orientation = data -> orientation;
    //ROS_INFO("%3.2f %3.2f %3.2f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
}

//Destructor

Navigator::~Navigator(){
    delete this -> laser;
    delete this -> og;
}

std::list<LaserPoint> Navigator::remakeRanges(std::list<LaserPoint> ranges){
    std::list<LaserPoint> aux;
    std::list<LaserPoint>::iterator it = ranges.begin();

    while(it != ranges.end()){
        aux.push_back((*it));
        //ROS_INFO("%3.2f %3.2f", (*it).angle, (*it).range);
        
        for(int i = 0; i < MEASURES/RANGES; i++)
            it++;
    }

    it--;

    aux.push_back((*it));
    //ROS_INFO("%3.2f %3.2f", (*it).angle, (*it).range);

    return aux;
}

std::list<_2DPoint>* Navigator::calculateDistances(Robot* robot){
    std::list<LaserPoint> ranges = remakeRanges(this -> laser -> getRanges());
    std::list<LaserPoint>::iterator it = ranges.begin();
    std::list<_2DPoint>* wall_points = new std::list<_2DPoint>();
    _2DPoint aux;
    float angle;
    bool last_angle = false;

    while(it != ranges.end()){
        if((*it).range < MIN_RANGE){
            angle = Resources::angleSum((*it).angle, robot -> yaw);

            aux.x = robot -> position.x + ((*it).range * cos(angle));
            aux.y = robot -> position.y + ((*it).range * sin(angle));

            wall_points -> push_back(aux);
            
            //ROS_INFO("%3.2f %3.2f", (*it).angle, (*it).range);
            //ROS_INFO("%3.2f %3.2f %3.2f (%2.2f %2.2f)", (*it).angle, robot -> yaw, angle, robot -> position.x, robot -> position.y);
        }
        
        //ROS_INFO("%3.2f %3.2f", (*it).angle, (*it).range);

        it++;
    }

    return wall_points;
}

float Navigator::calculateAngle(_2DPoint *goal, std::list<_2DPoint>* wall_points, Robot* robot){
    _2DPoint aux;
    std::list<_2DPoint>::iterator it;
    OGVector ogv;
    float x_component = 0;
    float y_component = 0;
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
        aux.x = (*it).x - robot -> position.x;
        aux.y = (*it).y - robot -> position.y;

        norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 1.5);

        /*
        if(pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5) > DANGER_ZONE){
            aux.x *= 0.4;
            aux.y *= 0.4;
        }
        ROS_INFO("%3.2f", fabs(Resources::normalizeAngle(atan((*it).y/(*it).x))));

        if(fabs(Resources::normalizeAngle(atan((*it).y/(*it).x))) > DANGER_ANGLE){
            aux.x *= 0.7;
            aux.y *= 0.7;
        }
        */

        x_component -= (aux.x * QWALL)/norm;
        y_component -= (aux.y * QWALL)/norm;
        
        it++;
    }
    
    if(og -> OGReady()){
        //ROS_INFO("using occupancy grid");

        ogv = og -> calculateOGVector(robot -> position);

        norm = pow(pow(ogv.x, 2) + pow(ogv.y, 2), 0.5);

        x_component += (ogv.x * QOG)/norm;
        y_component += (ogv.y * QOG)/norm;
    }

    //ROS_INFO("OG:%2.2f %2.2f xy:%2.2f %2.2f", (ogv.x * QOG)/norm, (ogv.y * QOG)/norm, x_component, y_component);

    //ROS_INFO("XY:%3.2f %3.2f OG:%3.2f %3.2f %d", x_component, y_component, ogv.x, ogv.y, (int) wall_points -> size());
    
    if((y_component == 0 && x_component == 0) || isnan(y_component) || isnan(x_component))
        return Resources::transformYaw(robot -> yaw);

    //ROS_INFO("xy: %3.2f %3.2f", x_component, y_component);
    //ROS_INFO("%f", Resources::normalizeAngle(atan2(y_component, x_component)));

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

    angleForce = calculateAngle(goal, wall_points, robot);

    angleDiff = Resources::angleDiff(angleForce, Resources::transformYaw(robot -> yaw))*MAX_ANG_SPEED/M_PI;

    info.velocity = (this -> laser -> getFront() - 0.3)*MAX_LIN_SPEED/7.7;
    info.rotation = angleDiff;

    //ROS_INFO("Velocity %3.2f", info.velocity);
    //ROS_INFO("Angle: %3.2f %3.2f", angleDiff, robot -> yaw);

    this -> laser -> setStatus(false);
    this -> og -> updatePosition(robot -> position.x, robot -> position.y, robot -> yaw);
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

    return true;
}

bool Navigator::search(robot_control::search::Request& request, robot_control::search::Request& response){
    ROS_INFO("service called");

    DrivingInfo info;

    //ros::Rate r(20);
    
    //Waits for the laser to set the initial values
    ROS_INFO("waiting for laser");
    while(!(laser -> isReady()) && ros::ok()){
        ros::spinOnce();
    }

    while(ros::ok()){
        while(laser -> getStatus() == false && ros::ok()){
            ros::spinOnce();
        }

        info = this -> defineDirection(NULL);
        this -> drive(info);
        ros::spinOnce();
    }

    this -> stop();

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
    //ros::spinOnce();

    ROS_INFO("Navigator started.");

    

    //_2DPoint *goal = nav -> createGoal(-3, 10);

    //nav -> drive();

    //delete nav;

    ros::spin();

    return 0;
}