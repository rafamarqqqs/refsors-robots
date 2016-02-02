#include "../include/robot_control/navigation.h"


//Constructor

Navigator::Navigator(ros::NodeHandle n){
    node = n;

    this -> laser = new Laser();
    this -> localization = new Localization();

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>("/larger_robot/cmd_vel", 100);

    //Subscribing to sensors
    laser_sub = node.subscribe("/larger_robot/scan", 1, &Laser::handleSubscription, this -> laser);
}

//Destructor

Navigator::~Navigator(){
    delete this -> laser;
    delete this -> localization;
}

std::list<_2DPoint>* Navigator::calculateDistances(_2DPoint* robot, float yaw){
    std::list<float> ranges = this -> laser -> getRanges();
    std::list<float>::iterator it = ranges.begin();
    std::list<_2DPoint>* wall_points = new std::list<_2DPoint>();
    _2DPoint aux;
    int i = 0;

    while(it != ranges.end()){
        if((*it) < 8){

            float angle = Resources::angleSum((INCREMENT*i) + this -> laser -> getAngleMin(), yaw);

            aux.x = robot -> x + ((*it) * cos(angle));
            aux.y = robot -> y + ((*it) * sin(angle));
            //ROS_INFO("%.3f %.3f %.3f %.3f", (INCREMENT*i) + this -> laser -> getAngleMin(), yaw, (INCREMENT*i) + this -> laser -> getAngleMin() + yaw, (*it));
            wall_points -> push_back(aux);
        }

        it++;
        i++;
    }

    return wall_points;
}

DrivingInfo Navigator::defineDirection(Goal *goal){
    DrivingInfo info;
    
    std::list<_2DPoint>::iterator it;
    std::list<_2DPoint>* wall_points;
    _2DPoint aux;

    float x_component, y_component;

    _2DPoint *robot;
    float yaw;
    float norm;

    localization -> getTfTransforms();
    robot = localization -> getPosition();
    yaw = localization -> getYaw();

    wall_points = calculateDistances(robot, yaw);

    ROS_INFO("Got wall points, calculating attractions/repulsions");

    //Goal attraction
    aux.x = goal -> destiny.x - robot -> x;
    aux.y = goal -> destiny.y - robot -> y;
    
    norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);
    
    x_component = (aux.x/norm);
    y_component = (aux.y/norm);

    
    //Wall repulsion
    it = wall_points -> begin();

    while(it != wall_points -> end()){
        aux.x = (*it).x - robot -> x;
        aux.y = (*it).y - robot -> y;

        norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 1.5);

        x_component -= aux.x/norm;
        y_component -= aux.y/norm;

        it++;
    }

    ROS_INFO("Finished.");

    info.rotation = Resources::angleDiff(atan2(y_component, x_component), yaw)*0.7/M_PI;
    info.velocity = (this -> laser -> getFront() - 0.3)*0.5/7.7;
    //info.velocity = 0.3;

    //ROS_INFO("%f", this -> laser -> getFront());

    ROS_INFO("(%.3f|%.3f) %.2f|%.2f (%.2f,%.2f)",yaw, info.rotation+yaw, x_component, y_component, robot -> x, robot -> y);
    //ROS_INFO("(%.3f %.3f) %.3f [%d/%d-%d/%d]", info.rotation, info.velocity, yaw, (int) goal -> destiny.x, (int) goal -> destiny.y, (int) this -> position -> pose.position.x, (int) this -> position -> pose.position.y);


    this -> laser -> setStatus(false);

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

/*
    Drives forward
*/

void Navigator::driveForward(){
    geometry_msgs::Twist msg;
    msg.linear.x = 1;
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

bool Navigator::driveTo(Goal *goal){
    DrivingInfo info;
    
    //Waits for the laser to set the initial values
    ROS_INFO("waiting for laser");
    

    while(!REACHED_DESTINATION(goal, this -> localization -> getPosition()) && ros::ok()){
        
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

    Goal *goal = nav -> createGoal(-3, 10);

    nav -> driveTo(goal);

    delete nav;
}