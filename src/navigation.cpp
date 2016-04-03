#include "../include/robot_control/navigation.h"

//Constructor

Navigator::Navigator(ros::NodeHandle n, char* _type):
    searchServer(n, SEARCH_ACTION, boost::bind(&Navigator::search, this, _1), false),
    driveToServer(n, DRIVE_TO_ACTION, boost::bind(&Navigator::driveTo, this, _1), false),
    type(_type), node(n), id(-1), found(false) {

    if(strcmp(_type, LARGER_ROBOT) == 0){
        qwall = 1.0;
        qog = 0.7;
        qgoal = 0.7;
        qtail = 0.0;
        qrobots = 1.5;
        max_lin_speed = 0.6;
        max_ang_speed = 0.9;
        navigation_error = 1.0;
        //max_lin_speed = 0.6;
        //max_ang_speed = 1.0;
        min_dist = 6;
        critical_wall_dist = 1.0;
        this -> og = new OccupancyGrid(n, MAP_LENGTH, MAP_WIDTH, LARGER_ROBOT_CELL_SIZE, AREA_SIZE, REPULSION, 3, 3);
    }
    else{
        qwall = 0.43;
        qog = 0.4;
        qgoal = 1.25;
        qtail = 0.75;
        qrobots = 1.5;
        max_lin_speed = 0.85;
        max_ang_speed = 1.8;
        navigation_error = 1.0;
        //max_lin_speed = 1.2;
        //max_ang_speed = 1.8;
        min_dist = 0.65;
        critical_wall_dist = 1.0;
        this -> og = new OccupancyGrid(n, MAP_LENGTH, MAP_WIDTH, SMALLER_ROBOT_CELL_SIZE, AREA_SIZE, REPULSION, 2, 2);
    }

    robot = (Robot*) malloc(sizeof(Robot));
    driving_info = (DrivingInfo*) malloc(sizeof(DrivingInfo));
    goal = (_2DPoint*) malloc(sizeof(_2DPoint));

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>(VEL_TOPIC(_type), 10);

    //Subscribing to sensors
    laser_sub = node.subscribe(LASER_TOPIC(_type), 1, &Navigator::handleLaser, this);
    pose_sub = node.subscribe(POSE_TOPIC(_type), 1, &Navigator::handlePose, this);

    client = n.serviceClient<robot_control::getPositions>(GET_POSITIONS_SERVICE);

    searchServer.registerPreemptCallback(boost::bind(&Navigator::searchPreempted, this));

    searchServer.start();
    driveToServer.start();
}

void Navigator::handlePose(const geometry_msgs::PoseStamped::ConstPtr& data){
    this -> robot -> position.x = data -> pose.position.x;
    this -> robot -> position.y = data -> pose.position.y;
    tf::Quaternion q(data -> pose.orientation.x, data -> pose.orientation.y, data -> pose.orientation.z, data -> pose.orientation.w);
    tf::Matrix3x3(q).getRPY(robot -> roll, robot -> pitch, robot -> yaw);
}

void Navigator::handleLaser(const robot_control::laserMeasures::ConstPtr& data){
    this -> range = data -> range;
    this -> angle = data -> angle;
    this -> robot -> front = data -> front;
}

//Destructor

Navigator::~Navigator(){
    this -> og -> writeMap(type);
    free(driving_info);
    free(robot);
    free(goal);
    delete this -> og;
}

inline std::list<LaserPoint>* Navigator::remakeRanges(){
    std::vector<double> range(this -> range);
    std::vector<double> angle(this -> angle);

    LaserPoint aux;

    std::vector<double>::iterator range_it = range.begin();
    std::vector<double>::iterator angle_it = angle.begin();

    std::list<LaserPoint>* laser_list = new std::list<LaserPoint>();

    while(range_it != range.end() && ros::ok()){
        //ROS_INFO("%3.2f %3.2f", (*angle_it), (*range_it));
        aux.range = (*range_it);
        aux.angle = (*angle_it);

        laser_list -> push_back(aux);

        for(int i = 0; i < LASER_PI_MEASURES/NAV_MEASURES; i++){
            range_it++;
            angle_it++;
        }
    }
    
    range_it--;
    angle_it--;

    aux.range = (*range_it);
    aux.angle = (*angle_it);

    //ROS_INFO("%3.2f %3.2f", (*angle_it), (*range_it));

    //ROS_INFO("\n");

    laser_list -> push_back(aux);

    return laser_list;
}

inline std::list<_2DPoint>* Navigator::calculateDistances(){
    std::list<LaserPoint>* ranges = remakeRanges();
    std::list<LaserPoint>::iterator it = ranges -> begin();
    std::list<_2DPoint>* wall_points = new std::list<_2DPoint>();

    while(it != ranges -> end() && ros::ok()){
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

inline void Navigator::getGoalAttraction(double* x_component, double* y_component){
    _2DPoint aux;
    double norm;

    //Goal attraction
    aux.x = goal -> x - robot -> position.x;
    aux.y = goal -> y - robot -> position.y;
    
    norm = pow(SQUARE(aux.x) + SQUARE(aux.y), 0.5);

    *x_component = (aux.x * qgoal)/norm;
    *y_component = (aux.y * qgoal)/norm;
}

inline void Navigator::getWallsRepulsion(double* x_component, double* y_component, std::list<_2DPoint>* wall_points){

    std::list<_2DPoint>::iterator it;
    double norm;
    _2DPoint aux;

    /* wall repulsion */
    it = wall_points -> begin();

    while(it != wall_points -> end() && ros::ok()){
        aux.x = (*it).x - robot -> position.x;
        aux.y = (*it).y - robot -> position.y;

        norm = pow(SQUARE(aux.x) + SQUARE(aux.y), 0.5);

        *x_component -= (((critical_wall_dist/norm) - (critical_wall_dist/min_dist)) * 
                        (aux.x/TO_THE_FOURTH(norm))) * qwall;
        *y_component -= (((critical_wall_dist/norm) - (critical_wall_dist/min_dist)) * 
                        (aux.y/TO_THE_FOURTH(norm))) * qwall;
        
        it++;
    }
}

inline void Navigator::getRobotsRepulsion(double* x_component, double* y_component){
    std::vector<double>::iterator it_x;
    std::vector<double>::iterator it_y;
    _2DPoint aux;
    robot_control::getPositions srv;
    double robots_min_dist = ROBOTS_MIN_DIST;
    double norm;

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

        norm = pow(SQUARE(aux.x) + SQUARE(aux.y), 0.5);

        if(norm < robots_min_dist){
            //ROS_INFO("%3.2lf %3.2lf", (*it_x), (*it_y));
            //*x_component -= (aux.x/norm) * qrobots;
            //*y_component -= (aux.y/norm) * qrobots;
            *x_component -= (((1/norm) - (1/robots_min_dist)) * 
                            (aux.x/pow(norm, 2)));
            *y_component -= (((1/norm) - (1/robots_min_dist)) * 
                            (aux.y/pow(norm, 2)));            

            robots_min_dist++;
            ROS_INFO("Robots: %6.4lf %6.4lf", (aux.x/norm) * qrobots, (aux.y/norm) * qrobots);
        }
    }
}

inline double Navigator::calculateAngle(std::list<_2DPoint>* wall_points){
    double x_component = 0.0;
    double y_component = 0.0;
    double norm;
    _2DPoint aux;
    
    /* goal atraction */
    getGoalAttraction(&x_component, &y_component);

    //ROS_INFO("goal %4.4lf %4.4lf", x_component, y_component);

    /* walls repulsion */
    getWallsRepulsion(&x_component, &y_component, wall_points);

    //ROS_INFO("walls %4.4lf %4.4lf", x_component, y_component);

    /* occupancy grid force */
    og -> calculateOGVector(robot -> position, &aux.x, &aux.y);

    if(og -> OGInfluence(robot -> position.x, robot -> position.y) > 0.7){
        norm = pow(SQUARE(aux.x) + SQUARE(aux.y), 0.5);

        if(norm != 0){
            x_component += (aux.x * qog)/norm;
            y_component += (aux.y * qog)/norm;
        }
    
        //ROS_INFO("og %4.4lf %4.4lf", x_component, y_component);
    }
    
    /* tail force 
    */
    og -> calculateTailForce(robot -> position, &aux.x, &aux.y);

    x_component -= aux.x * qtail;
    y_component -= aux.y * qtail;
    //ROS_INFO("tail %4.4lf %4.4lf", x_component, y_component);

    /* repulsion to other robots */
    getRobotsRepulsion(&x_component, &y_component);

    //ROS_INFO("robot rep %4.4lf %4.4lf", x_component, y_component);
    
    /* no forces acting */
    if((y_component == 0 && x_component == 0) || std::isnan(y_component) || std::isnan(x_component))
        return robot -> yaw;

    return atan2(y_component, x_component);
}

inline void Navigator::defineDirection(){
    std::list<_2DPoint>* wall_points;
    double angle_diff, angle_force;
    /*double front = this -> front;

    robot.position.x = this -> pose.position.x;
    robot.position.y = this -> pose.position.y;
    robot.yaw = tf::getYaw(this -> pose.orientation);*/

    wall_points = calculateDistances();

    angle_force = calculateAngle(wall_points);

    //angle_diff = Resources::angleDiff(angle_force, robot -> yaw)*5;
    angle_diff = Resources::angleDiff(angle_force, robot -> yaw)*max_ang_speed/(M_PI);

    driving_info -> velocity = (robot -> front - 0.15)*max_lin_speed/5.9;

    if(fabs(angle_diff) > M_PI/2)
        driving_info -> rotation = M_PI/2;
    else
        driving_info -> rotation = angle_diff;

    this -> og -> updatePosition(robot -> position.x, robot -> position.y);
    //this -> og -> writeMap(type);
}

inline void Navigator::stop(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    velocity_pub.publish(msg);
}

inline void Navigator::drive(){
    geometry_msgs::Twist msg;
    msg.angular.z = driving_info -> rotation;
    msg.linear.x = driving_info -> velocity;
    velocity_pub.publish(msg);
}

void Navigator::driveTo(const robot_control::driveToGoalConstPtr &driveTo_goal){
    vector<double> aux_x = driveTo_goal -> x;
    vector<double> aux_y = driveTo_goal -> y;
    vector<double>::iterator it_x;
    vector<double>::iterator it_y;

    ros::Rate r(5);

    this -> qog = 0.5;
    this -> qwall = 0.5;
    this -> qrobots = 0.0;
    this -> qtail = 0.1;
    this -> qgoal = 1.0;

    for(it_x = aux_x.begin(),
        it_y = aux_y.begin(); it_x != aux_x.end(); it_x++, it_y++){

        ROS_INFO("Driving to %lf %lf", (*it_x), (*it_y));

        this -> goal -> x = *it_x;
        this -> goal -> y = *it_y;
        
        while(!REACHED_DESTINATION(this -> goal, robot ->position, navigation_error) && ros::ok()){

            defineDirection();
            drive();

            r.sleep();
        }
    }

    this -> stop();

    driveToServer.setSucceeded();
}

void Navigator::searchPreempted(){
    found = true;    
}

void Navigator::search(const robot_control::searchGoalConstPtr &search_goal){
    ros::Time start_time;
    robot_control::searchResult result;
    
    goal -> x = INT_MAX;
    goal -> y = INT_MAX;

    ros::Rate r(5);
    
    //Waits for the laser to set the initial values
    while((!LASER_STARTED || !LOCALIZATION_STARTED) && ros::ok()){
        r.sleep();
    }

    ros::Time g_start_time = ros::Time::now();

    //!REACHED_GLOBAL_TIME_LIMIT(g_start_time.toSec(), ros::Time::now().toSec

    while(found == false && ros::ok()){
        og -> getNewGoal(this -> goal);

        start_time = ros::Time::now();
        
        ROS_INFO("%s Driving to %3.2f %3.2f", type.c_str(), goal -> x, goal -> y);

        while(found == false && !REACHED_DESTINATION(goal, robot ->position, SEARCH_ERROR) && 
              !REACHED_TIME_LIMIT(start_time.toSec(), ros::Time::now().toSec()) && ros::ok()){

            defineDirection();
            drive();

            r.sleep();
            //ROS_INFO("%s %3.2f %3.2f %3.2f %3.2f\n", type.c_str(), robot ->position.x, robot ->position.y, goal -> x, goal -> y);
        }
        if(REACHED_DESTINATION(goal, robot -> position, SEARCH_ERROR))
            ROS_INFO("Reached destination");
        else
            ROS_INFO("Time's up");
    }

    this -> stop();

    result.x = robot -> position.x;
    result.y = robot -> position.y;

    searchServer.setSucceeded(result);
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node

    ros::init(argc, argv, NAVIGATION_NODE);

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