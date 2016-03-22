#include "../include/robot_control/navigation.h"

//Constructor

Navigator::Navigator(ros::NodeHandle n, char* _type):
    searchServer(n, "search", boost::bind(&Navigator::search, this, _1), false),
    type(_type), node(n), id(-1) {

    if(strcmp(_type, "larger_robot") == 0){
        qwall = 1.0;
        qog = 1.0;
        qgoal = 1.0;
        qtail = 0;
        max_lin_speed = 0.8;
        max_ang_speed = 1.2;
        min_dist = 8;
        critical_wall_dist = 1.0;
        this -> og = new OccupancyGrid(MAP_LENGTH, MAP_WIDTH, LARGER_ROBOT_CELL_SIZE, AREA_SIZE, REPULSION, 4, 2);
    }
    else{
        qwall = 0.45;
        qog = 0.8;
        qgoal = 1.6;
        qtail = 0.4;
        max_lin_speed = 0.9;
        max_ang_speed = 2.5;
        min_dist = 8;
        critical_wall_dist = 0.25;
        this -> og = new OccupancyGrid(MAP_LENGTH, MAP_WIDTH, SMALLER_ROBOT_CELL_SIZE, AREA_SIZE, REPULSION, 4, 2);
    }

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>(VEL(_type), 100);

    //Subscribing to sensors
    laser_sub = node.subscribe(LASER(_type), 1, &Navigator::handleLaser, this);
    pose_sub = node.subscribe(POSE(_type), 1, &Navigator::handlePose, this);

    client = n.serviceClient<robot_control::getPositions>("/Knowledge/getPositions");

    searchServer.start();
}

void Navigator::handlePose(const geometry_msgs::PoseStamped::ConstPtr& data){
    this -> pose.position = data -> pose.position;
    this -> pose.orientation = data -> pose.orientation;
}

void Navigator::handleLaser(const robot_control::laserMeasures::ConstPtr& data){
    this -> range = data -> range;
    this -> angle = data -> angle;
    this -> front = data -> front;
}

//Destructor

Navigator::~Navigator(){
    this -> og -> writeMap(type);
    delete this -> og;
}

std::list<LaserPoint>* Navigator::remakeRanges(){
    std::vector<double> range(this -> range);
    std::vector<double> angle(this -> angle);

    LaserPoint aux;

    std::vector<double>::iterator range_it = range.begin();
    std::vector<double>::iterator angle_it = angle.begin();

    std::list<LaserPoint>* laser_list = new std::list<LaserPoint>();

    while(range_it != range.end()){
        //ROS_INFO("%3.2f %3.2f", (*angle_it), (*range_it));
        aux.range = (*range_it);
        aux.angle = (*angle_it);

        laser_list -> push_back(aux);

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

    laser_list -> push_back(aux);

    return laser_list;
}

std::list<_2DPoint>* Navigator::calculateDistances(Robot *robot){
    std::list<LaserPoint>* ranges = remakeRanges();
    std::list<LaserPoint>::iterator it = ranges -> begin();
    std::list<_2DPoint>* wall_points = new std::list<_2DPoint>();

    while(it != ranges -> end()){
        if((*it).range < min_dist){
            _2DPoint aux;
            double angle;
            
            angle = Resources::angleSum((*it).angle, robot -> yaw);

            aux.x = (robot -> position.x) + ((*it).range * cos(angle));
            aux.y = (robot -> position.y) + ((*it).range * sin(angle));

            wall_points -> push_back(aux);
        }
        
        it++;
    }

    delete ranges;

    return wall_points;
}

double Navigator::calculateAngle(_2DPoint* goal, std::list<_2DPoint>* wall_points, Robot *robot){
    std::list<_2DPoint>::iterator it;
    std::vector<double>::iterator it_x;
    std::vector<double>::iterator it_y;
    double x_component;
    double y_component;
    double norm;
    double robots_min_dist = ROBOTS_MIN_DIST;
    robot_control::getPositions srv;
    _2DPoint aux;
    
    //Goal attraction
    aux.x = goal -> x - robot -> position.x;
    aux.y = goal -> y - robot -> position.y;
    
    norm = sqrt(SQUARE(aux.x) + SQUARE(aux.y));

    x_component = (aux.x * qgoal)/norm;
    y_component = (aux.y * qgoal)/norm;

    /* wall repulsion */
    it = wall_points -> begin();

    while(it != wall_points -> end()){
        aux.x = (*it).x - robot -> position.x;
        aux.y = (*it).y - robot -> position.y;

        norm = sqrt(SQUARE(aux.x) + SQUARE(aux.y));

        x_component -= (((critical_wall_dist/norm) - (critical_wall_dist/min_dist)) * 
                        (aux.x/TO_THE_FOURTH(norm))) * qwall;
        y_component -= (((critical_wall_dist/norm) - (critical_wall_dist/min_dist)) * 
                        (aux.y/TO_THE_FOURTH(norm))) * qwall;
        
        it++;
    }

    /* occupancy grid force */
    

    //if(og -> OGInfluence(robot -> position.x, robot -> position.y) > 0.3){

        og -> calculateOGVector(&robot -> position, &aux.x, &aux.y);
        norm = sqrt(SQUARE(aux.x) + SQUARE(aux.y));

        if(norm != 0){
            x_component += (aux.x * qog)/norm;
            y_component += (aux.y * qog)/norm;
        }
    //}

    /* tail force
    */

    og -> calculateTailForce(&robot -> position, &aux.x, &aux.y);

    x_component -= aux.x * qtail;
    y_component -= aux.y * qtail;


    /* repulsion to other robots 
    */

    srv.request.my_x = robot -> position.x;
    srv.request.my_y = robot -> position.y;

    if(id == -1){
        srv.request.has_id = false;
    }
    else{
        srv.request.has_id = true;
        srv.request.my_id = id;
    }

    client.call(srv);

    if(id == -1)
        id = srv.response.id;

    for(it_x = srv.response.x.begin(), it_y = srv.response.y.begin();
        it_x != srv.response.x.end() && it_y != srv.response.y.end(); it_x++, it_y++){

        aux.x = (*it_x) - robot -> position.x;
        aux.y = (*it_y) - robot -> position.y;

        norm = sqrt(SQUARE(aux.x) + SQUARE(aux.y));

        if(norm < robots_min_dist){
            //ROS_INFO("%3.2lf %3.2lf", (*it_x), (*it_y));
            x_component -= (aux.x/norm) * QROBOTS;
            y_component -= (aux.y/norm) * QROBOTS;
            //x_component -= (((critical_wall_dist/norm) - (critical_wall_dist/robots_min_dist)) * 
              //              (aux.x/pow(norm, 2)));
            //y_component -= (((critical_wall_dist/norm) - (critical_wall_dist/robots_min_dist)) * 
                //            (aux.y/pow(norm, 2)));            

            robots_min_dist++;
        }
    }

    /* no forces acting */
    if((y_component == 0 && x_component == 0) || std::isnan(y_component) || std::isnan(x_component))
        return robot -> yaw;

    return atan2(y_component, x_component);
}

DrivingInfo* Navigator::defineDirection(_2DPoint* goal){
    DrivingInfo* info = new DrivingInfo();
    std::list<_2DPoint>* wall_points;
    Robot robot;
    double angle_diff, angle_force;
    double front = this -> front;

    robot.position.x = this -> pose.position.x;
    robot.position.y = this -> pose.position.y;
    robot.yaw = tf::getYaw(this -> pose.orientation);

    wall_points = calculateDistances(&robot);

    angle_force = calculateAngle(goal, wall_points, &robot);

    angle_diff = Resources::angleDiff(angle_force, robot.yaw)*max_ang_speed/(M_PI/1.5);

    info -> velocity = (front - 0.15)*max_lin_speed/6.5;

    info -> rotation = angle_diff;

    this -> og -> updatePosition(robot.position.x, robot.position.y);
    //this -> og -> writeMap(type);

    delete wall_points;

    return info;
}

void Navigator::stop(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    velocity_pub.publish(msg);
}

void Navigator::drive(DrivingInfo* info){
    geometry_msgs::Twist msg;
    msg.angular.z = info -> rotation;
    msg.linear.x = info -> velocity;
    velocity_pub.publish(msg);
}

void Navigator::search(const robot_control::searchGoalConstPtr &search_goal){
    _2DPoint* goal = new _2DPoint();
    goal -> x = INT_MAX;
    goal -> y = INT_MAX;
    ros::Time start_time, current_time;

    ros::Rate r(10);
    
    //Waits for the laser to set the initial values
    while((!LASER_STARTED || !LOCALIZATION_STARTED) && ros::ok()){
        r.sleep();
    }

    while(ros::ok()){
        og -> getNewGoal(goal);

        start_time = ros::Time::now();
        current_time = ros::Time::now();
        
        ROS_INFO("%s Driving to %3.2f %3.2f", type.c_str(), goal -> x, goal -> y);

        while(!REACHED_DESTINATION(goal, pose.position) && 
              !REACHED_TIME_LIMIT(start_time.toSec(), current_time.toSec())){

            drive(defineDirection(goal));

            r.sleep();
            //ROS_INFO("%s %3.2f %3.2f %3.2f %3.2f\n", type.c_str(), pose.position.x, pose.position.y, goal -> x, goal -> y);
            current_time = ros::Time::now();
        }
        if(REACHED_DESTINATION(goal, pose.position))
            ROS_INFO("reached destination");
        else
            ROS_INFO("time's up");
    }

    delete goal;

    this -> stop();
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