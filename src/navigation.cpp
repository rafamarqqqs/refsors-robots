#include "../include/robot_control/navigation.h"

//Constructor

Navigator::Navigator(ros::NodeHandle n, char *type):
    searchServer(n, "search", boost::bind(&Navigator::search, this, _1), false)/*,
    laser_sub(n, LASER(type), 1), pose_sub(n, POSE(type), 1), sync(SyncPolicy(100), laser_sub, pose_sub)*/ {

    std::string aux(type);

    this -> type = aux;

    node = n;

    if(strcmp(type, "larger_robot") == 0){
        qwall = 0.8;
        qog = 1.5;
        max_lin_speed = 1.0;
        max_ang_speed = 1.4;
        min_dist = 6;
    }
    else{
        qwall = 0.6;
        qog = 1.5;
        max_lin_speed = 2.0;
        max_ang_speed = 2.5;
        min_dist = 4;
    }




    this -> og = new OccupancyGrid(MAP_LENGTH, MAP_WIDTH, CELL_SIZE, AREA_SIZE, REPULSION);

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>(VEL(type), 100);

    //sync.registerCallback(boost::bind(&Navigator::syncCallback, this, _1, _2));

    //Subscribing to sensors
    laser_sub = node.subscribe(LASER(type), 1, &Navigator::handleLaser, this);
    pose_sub = node.subscribe(POSE(type), 1, &Navigator::handlePose, this);

    laser_ready = false;
    localization_ready = false;

    searchServer.start();
}
/*
void Navigator::syncCallback(const robot_control::laserMeasures::ConstPtr& laser_data, 
        const geometry_msgs::PoseStamped::ConstPtr& pose_data){
    ROS_INFO("here");
}
*/
void Navigator::handlePose(const geometry_msgs::PoseStamped::ConstPtr& data){

    if(data -> pose.position.x == 0 && data -> pose.position.y == 0 && !localization_ready)
        return;

    this -> pose.position = data -> pose.position;
    this -> pose.orientation = data -> pose.orientation;

    this -> localization_ready = true;
    //ROS_INFO("pose %6.4f", data -> header.stamp.toSec());
}

void Navigator::handleLaser(const robot_control::laserMeasures::ConstPtr& data){
    //ROS_INFO("laser %6.4f", data -> header.stamp.toSec());

    if(data -> range.size() == 0 || data -> angle.size() == 0)
        return;

    this -> range = data -> range;
    this -> angle = data -> angle;
    this -> front = data -> front;
    this -> laser_ready = true;
}

//Destructor

Navigator::~Navigator(){
    delete this -> og;
}

std::list<LaserPoint> Navigator::remakeRanges(){
    std::vector<double> range(this -> range);
    std::vector<double> angle(this -> angle);

    LaserPoint aux;

    std::vector<double>::iterator range_it = range.begin();
    std::vector<double>::iterator angle_it = angle.begin();

    std::list<LaserPoint> laser_list;

    while(range_it != range.end()){
        //ROS_INFO("%3.2f %3.2f", (*angle_it), (*range_it));
        aux.range = (*range_it);
        aux.angle = (*angle_it);

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

    //ROS_INFO("%3.2f %3.2f", (*angle_it), (*range_it));

    laser_list.push_back(aux);

    return laser_list;
}

std::list<_2DPoint>* Navigator::calculateDistances(Robot* robot){
    std::list<LaserPoint> ranges = remakeRanges();
    std::list<LaserPoint>::iterator it = ranges.begin();
    std::list<_2DPoint>* wall_points = new std::list<_2DPoint>();
    _2DPoint aux;
    double angle;

    while(it != ranges.end()){
        if((*it).range < min_dist){
            angle = Resources::angleSum((*it).angle, robot -> yaw);

            aux.x = robot -> position.x + ((*it).range * cos(angle));
            aux.y = robot -> position.y + ((*it).range * sin(angle));

            wall_points -> push_back(aux);
        }
        
        it++;
    }

    return wall_points;
}

double Navigator::calculateAngle(_2DPoint *goal, std::list<_2DPoint>* wall_points, Robot* robot){
    _2DPoint aux;
    std::list<_2DPoint>::iterator it;
    OGVector* ogv;
    OGVector* ogvTail;
    double x_component;
    double y_component;
    double norm;
    double influence = og -> OGInfluence(robot -> position.x, robot -> position.y);

    x_component = 0;
    y_component = 0;
    //Goal attraction

    //SÓ SOMAR GOAL QUANDO O QROG FOR TRUE
    /*

    aux.x = goal -> x - robot -> position.x;
    aux.y = goal -> y - robot -> position.y;
        
    norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);
        
    x_component = (aux.x * QGOAL)/norm;
    y_component = (aux.y * QGOAL)/norm;

    if(influence > 0.1){
        x_component *= 0.5;
        y_component *= 0.5;
    }
    //ROS_INFO("og inf %3.2f", influence);

    x_component = aux.x*pow(norm, 2);
    y_component = aux.y*pow(norm, 2);
    */
    

    //Wall repulsion
    it = wall_points -> begin();

    while(it != wall_points -> end()){
        aux.x = (*it).x - robot -> position.x;
        aux.y = (*it).y - robot -> position.y;

        norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);

        //x_component -= (aux.x * QWALL)/norm;
        //y_component -= (aux.y * QWALL)/norm;
        
        x_component -= (((1/norm) - (1/min_dist)) * (aux.x/pow(norm, 3))) * qwall;
        y_component -= (((1/norm) - (1/min_dist)) * (aux.y/pow(norm, 3))) * qwall;

        it++;
    }


    ogv = og -> calculateOGVector(robot -> position);

    norm = pow(pow(ogv -> x, 2) + pow(ogv -> y, 2), 0.5);

    if(norm != 0){
        //ROS_INFO("og %3.4f %3.4f %d %d %3.2f", (ogv -> x * QOG)/norm, (ogv -> y * QOG)/norm, (int) ogv -> x, (int) ogv -> y, norm);
        x_component += (ogv -> x * qog)/norm;
        y_component += (ogv -> y * qog)/norm;
                
        //x_component += ((1/norm) - (1/min_range)) * (ogv -> x/pow(norm, 3));
        //y_component += ((1/norm) - (1/min_range)) * (ogv -> y/pow(norm, 3));
               
    }

    delete ogv;

    /*ogvTail = og -> calculateTailForce(robot -> position);

    x_component -= ogvTail -> x * QTAIL;
    y_component -= ogvTail -> y * QTAIL;
    */
    //ROS_INFO("tail %3.4f %3.4f", -ogvTail -> x, -ogvTail -> y);

/*
    if(influence > 0.3){
        x_component *= 3;
        y_component *= 3;
        ROS_INFO("REALLY trying to get out of here");
    }
    else if(influence > 0.1){
        ROS_INFO("trying to get out of here");
        //x_component *= 2.0;
        //y_component *= 2.0;
        x_component *= 2;
        y_component *= 2;
    }
*/    
    
    if((y_component == 0 && x_component == 0) || std::isnan(y_component) || std::isnan(x_component))
        return robot -> yaw;

    //ROS_INFO("xy: %3.4f %3.4f", x_component, y_component);

    return atan2(y_component, x_component);
}

DrivingInfo Navigator::defineDirection(_2DPoint *goal){
    DrivingInfo info;
    std::list<_2DPoint>* wall_points;
    Robot *robot = (Robot *) malloc(sizeof(Robot));
    double angleDiff, angleForce;
    double front = this -> front;

    robot -> position.x = this -> pose.position.x;
    robot -> position.y = this -> pose.position.y;
    robot -> yaw = tf::getYaw(this -> pose.orientation);

    wall_points = calculateDistances(robot);

    angleForce = calculateAngle(goal, wall_points, robot);

    angleDiff = Resources::angleDiff(angleForce, robot -> yaw)*max_ang_speed/M_PI;


    info.velocity = (front - 0.3)*max_lin_speed/7;
    info.rotation = angleDiff;

    //og -> setRepulsion((9.7*MAX_LIN_SPEED/7.7) - info.velocity);

    //ROS_INFO("%3.2f", info.velocity);

    this -> og -> updatePosition(robot -> position.x, robot -> position.y);
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

void Navigator::search(const robot_control::searchGoalConstPtr &search_goal){
    DrivingInfo info;
    _2DPoint* goal = new _2DPoint();
    time_t start_time, current_time;

    ros::Rate r(10);
    
    //Waits for the laser to set the initial values
    while((laser_ready == false || localization_ready == false) && ros::ok()){
        r.sleep();
    }

    while(ros::ok()){
        //goal -> x = pose.position.x;
        //goal -> y = pose.position.y;
        og -> getNewGoal(goal);

        time(&start_time);
        time(&current_time);
        
    //ROS_INFO("Driving to %3.2f %3.2f", goal -> x, goal -> y);

        while(!REACHED_DESTINATION(goal, pose.position) && 
              !REACHED_TIME_LIMIT(start_time, current_time, goal, pose.position)){
            info = this -> defineDirection(goal);
            this -> drive(info);
            r.sleep();
            //ROS_INFO("%3.2f %3.2f %3.2f %3.2f", pose.position.x, pose.position.y, goal -> x, goal -> y);
            time(&current_time);
        }
/*
        if(REACHED_DESTINATION(goal, pose.position))
            ROS_INFO("reached destination");
        else
            ROS_INFO("time's up");
*/
    }

    delete goal;

    this -> stop();
}

/*
    Creates a goal
*/

_2DPoint *Navigator::createGoal(double x, double y){
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

    delete nav;

    return 0;
}