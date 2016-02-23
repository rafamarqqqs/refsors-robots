#include "../include/robot_control/navigation.h"

//Constructor

Navigator::Navigator(ros::NodeHandle n, char *type):
    searchServer(n, "search", boost::bind(&Navigator::search, this, _1), false) {

    std::string aux(type);

    this -> type = aux;

    node = n;

    this -> min_range = strcmp(type, "larger_robot") == 0 ? 3 : 2;

    this -> og = new OccupancyGrid(MAP_LENGTH, MAP_WIDTH, CELL_SIZE, REPULSION, AREA_SIZE);

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>(VEL(type), 100);

    //Subscribing to sensors
    laser_sub = node.subscribe(LASER(type), 1, &Navigator::handleLaser, this);
    pose_sub = node.subscribe(POSE(type), 1, &Navigator::handlePose, this);

    laser_ready = false;

    searchServer.start();
}

void Navigator::handlePose(const geometry_msgs::Pose::ConstPtr& data){
    this -> pose.position = data -> position;
    this -> pose.orientation = data -> orientation;
}

void Navigator::handleLaser(const robot_control::laserMeasures::ConstPtr& data){
    if(data -> range.size() == 0 || data -> angle.size() == 0)
        return;

    this -> range = data -> range;
    this -> angle = data -> angle;
    this -> front = data -> front;

    this -> laser_ready = true;

    //ROS_INFO("received laser %3.2f %d %d", this -> front, (int)this -> range.size(), (int)this -> angle.size());
}

//Destructor

Navigator::~Navigator(){
    delete this -> og;
}

std::list<LaserPoint> Navigator::remakeRanges(){
    std::vector<float> range(this -> range);
    std::vector<float> angle(this -> angle);

    LaserPoint aux;

    std::vector<float>::iterator range_it = range.begin();
    std::vector<float>::iterator angle_it = angle.begin();

    std::list<LaserPoint> laser_list;

    while(range_it != range.end()){
        aux.range = (*range_it);
        aux.angle = (*angle_it);

        //ROS_INFO("%3.2f %3.2f", (*range_it), (*angle_it));

        laser_list.push_back(aux);

        for(int i = 0; i < MEASURES/RANGES; i++){
            range_it++;
            angle_it++;
        }
    }
    
    range_it--;
    angle_it--;

    aux.range = (*range_it);
    aux.angle = (*angle_it);

    laser_list.push_back(aux);
    //ROS_INFO("%3.2f %3.2f", (*it).angle, (*it).range);

    return laser_list;
}

std::list<_2DPoint>* Navigator::calculateDistances(Robot* robot){
    std::list<LaserPoint> ranges = remakeRanges();
    std::list<LaserPoint>::iterator it = ranges.begin();
    std::list<_2DPoint>* wall_points = new std::list<_2DPoint>();
    _2DPoint aux;
    float angle;

    while(it != ranges.end()){
        if((*it).range < min_range){
            angle = Resources::angleSum((*it).angle, robot -> yaw);

            aux.x = robot -> position.x + ((*it).range * cos(angle));
            aux.y = robot -> position.y + ((*it).range * sin(angle));

            wall_points -> push_back(aux);
            
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
    OGVector* ogv;
    float x_component;
    float y_component;
    float norm;

    x_component = 0;
    y_component = 0;
    
    //Goal attraction
    aux.x = goal -> x - robot -> position.x;
    aux.y = goal -> y - robot -> position.y;
    
    norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);
    
    x_component = (aux.x * QGOAL)/norm;
    y_component = (aux.y * QGOAL)/norm;
    

    //Wall repulsion
    it = wall_points -> begin();

    while(it != wall_points -> end()){
        aux.x = (*it).x - robot -> position.x;
        aux.y = (*it).y - robot -> position.y;

        norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);

        //x_component -= (aux.x * QWALL * ((pow(min_range, 0.5) - pow(norm, 0.5))/pow(min_range, 0.5)));
        //y_component -= (aux.y * QWALL * ((pow(min_range, 0.5) - pow(norm, 0.5))/pow(min_range, 0.5)));
        x_component -= (aux.x * QWALL)/norm;
        y_component -= (aux.y * QWALL)/norm;

        //ROS_INFO("x %3.2f y %3.2f", (aux.x * QWALL)/norm, (aux.y * QWALL)/norm);
        
        it++;
    }

/*    if(og -> OGReady()){
        //ROS_INFO("using occupancy grid");

        ogv = og -> calculateOGVector(robot -> position);

        norm = pow(pow(ogv -> x, 2) + pow(ogv -> y, 2), 0.5);

        if(norm != 0){

            x_component += (ogv -> x * QOG)/norm;
            y_component += (ogv -> y * QOG)/norm;
            
            //ROS_INFO("og %2.2f %2.2f n %2.2f", (ogv -> x * QOG)/norm, (ogv -> y * QOG)/norm, norm);
        }

        delete ogv;
    }
*/
    //ROS_INFO("final %3.2f %3.2f %3.2f", x_component, y_component, atan2(y_component, x_component));

    //ROS_INFO("XY:%3.2f %3.2f OG:%3.2f %3.2f %d", x_component, y_component, ogv.x, ogv.y, (int) wall_points -> size());
    
    //ROS_INFO("xy: %3.2f %3.2f og: %3.2f %3.2f", x_component, y_component, (ogv.x * QOG)/norm, (ogv.y * QOG)/norm);
    
    if((y_component == 0 && x_component == 0) || isnan(y_component) || isnan(x_component))
        return robot -> yaw;

    //ROS_INFO("Component angle %f", atan2(y_component, x_component));

    return atan2(y_component, x_component);
}

DrivingInfo Navigator::defineDirection(_2DPoint *goal){
    DrivingInfo info;
    std::list<_2DPoint>* wall_points;
    Robot *robot = (Robot *) malloc(sizeof(Robot));
    float angleDiff, angleForce;
    float front = this -> front;

    robot -> position.x = this -> pose.position.x;
    robot -> position.y = this -> pose.position.y;
    robot -> yaw = tf::getYaw(this -> pose.orientation);

    wall_points = calculateDistances(robot);

    angleForce = calculateAngle(goal, wall_points, robot);

    angleDiff = Resources::angleDiff(angleForce, robot -> yaw)*MAX_ANG_SPEED/M_PI;

    info.velocity = (front - 0.3)*MAX_LIN_SPEED/7.7;
    info.rotation = angleDiff;

    //ROS_INFO("Velocity %3.2f", info.velocity);
    //ROS_INFO("Angle: %3.2f %3.2f", angleDiff, robot -> yaw);

    this -> og -> updatePosition(robot -> position.x, robot -> position.y, robot -> yaw);
    this -> og -> writeMap(type);

    free(robot);

    delete wall_points;

    return info;
}

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

bool Navigator::driveTo(_2DPoint *goal){

    return true;
}

void Navigator::search(const robot_control::searchGoalConstPtr &search_goal){
    DrivingInfo info;
    _2DPoint* goal = new _2DPoint();
    time_t start_time, current_time;

    ros::Rate r(10);
    
    //Waits for the laser to set the initial values
    ROS_INFO("navigator waiting for laser");
    while(laser_ready == false && ros::ok()){
        r.sleep();
    }

    while(ros::ok()){
        og -> getNewGoal(goal);

        time(&start_time);
        time(&current_time);

        while(!REACHED_DESTINATION(goal, pose.position) && !REACHED_TIME_LIMIT(start_time, current_time)){
            info = this -> defineDirection(goal);
            this -> drive(info);
            r.sleep();
            time(&current_time);
        }
    }

    delete goal;

    this -> stop();
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

    ros::init(argc, argv, "Navigation");

    if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    ros::NodeHandle node;

    Navigator *nav = new Navigator(node, argv[1]);

    ROS_INFO("Navigator started.");

    ros::spin();

    return 0;
}