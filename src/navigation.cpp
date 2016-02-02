#include "../include/robot_control/navigation.h"


//Constructor

Navigator::Navigator(ros::NodeHandle n){
    node = n;

    this -> laser = new Laser();

    this -> position = (Position *) malloc(sizeof(Position));

    this -> localization = new Localization();

    //Advertising velocity topic
    velocity_pub = node.advertise<geometry_msgs::Twist>("/larger_robot/cmd_vel", 100);

    //Subscribing to sensors
    //this -> laser -> subscribeTo(node, "/larger_robot/scan", 100, this -> laser, sub);
    laser_sub = node.subscribe("/larger_robot/scan", 1, &Laser::handleSubscription, this -> laser);
    odom_sub = node.subscribe("/larger_robot/odometry/filtered", 1, &Navigator::handlePosition, this);
    imu_sub = node.subscribe("/imu/data", 1, &Navigator::handleOrientation, this);
}

//Destructor

Navigator::~Navigator(){
    free(this -> position);
    delete this -> laser;
}


/*

    ODOMETRY

*/

void Navigator::handlePosition(const nav_msgs::Odometry::ConstPtr &odometry_data){
    //Gets the estimated position and Z axis rotation of the robot
    //this -> position -> pose = odometry_data -> pose.pose;
    //this -> position -> yaw = tf::getYaw(odometry_data -> pose.pose.orientation);
}

void Navigator::handleOrientation(const sensor_msgs::Imu::ConstPtr &imu_data){
    //Gets the estimated position and Z axis rotation of the robot
    //this -> position -> yaw = tf::getYaw(imu_data -> orientation);
    //ROS_INFO("%f", this -> position -> yaw);
}

DrivingInfo Navigator::defineDirection(Goal *goal){
    DrivingInfo info;
    
    std::list<float> ranges = this -> laser -> getRanges();
    std::list<_2DPoint> wall_points;
    std::list<float>::iterator it = ranges.begin();
    std::list<_2DPoint>::iterator wall_it;

    float x_component, y_component;
    int i = 0;
    float wall_x, wall_y;

    _2DPoint *robot;
    float yaw = this -> position -> yaw;

    localization -> getTfTransforms();

    robot = localization -> getPosition();
    yaw = localization -> getYaw();

    _2DPoint aux;

    wall_points.clear();

    aux.x = goal -> destiny.x - robot -> x;
    aux.y = goal -> destiny.y - robot -> y;
    
    float norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 0.5);
    
    x_component = (aux.x/norm);
    y_component = (aux.y/norm);
    
    float ang1;
    float ang2;


    while(it != ranges.end()){
        if((*it) < 8){
            ang1 = (INCREMENT*i) + this -> laser -> getAngleMin();
            ang2 = yaw;

            float sum = ang1 + ang2;
            sum = fmod(sum,2*M_PI);

            float angle;

            if(sum <= 0)
                angle = sum;
            else
                angle = sum - (2*M_PI);

            aux.x = robot -> x + ((*it) * cos(angle));
            aux.y = robot -> y + ((*it) * sin(angle));
            //ROS_INFO("%.3f %.3f %.3f %.3f", (INCREMENT*i) + this -> laser -> getAngleMin(), yaw, (INCREMENT*i) + this -> laser -> getAngleMin() + yaw, (*it));
            wall_points.push_back(aux);
        }

        it++;
        i++;
    }
    
    wall_it = wall_points.begin();

    while(wall_it != wall_points.end()){
        aux.x = (*wall_it).x - robot -> x;
        aux.y = (*wall_it).y - robot -> y;

        norm = pow(pow(aux.x, 2) + pow(aux.y, 2), 1.5);

        x_component -= aux.x/norm;
        y_component -= aux.y/norm;
        wall_it++;
    }



    //info.rotation = acos((x_component/sqrt(SQUARE(x_component) + SQUARE(y_component)))) - yaw;

    ang1 = atan2(y_component, x_component);
    ang2 = yaw;
    float diff1 = ang1 - ang2;
    float diff2 = diff1 - 2*M_PI;
    float angDiff;

    if(fabs(diff1) <= fabs(diff2))
        angDiff = diff1;
    else
        angDiff = diff2;

    info.rotation = angDiff*0.5/M_PI;
    //info.velocity = (this -> laser -> getFront() - 0.3)*0.5/7.7;
    info.velocity = 0.3;

    //ROS_INFO("%f", this -> laser -> getFront());

    //if(this -> laser -> getFront() < 0.5)
      //  info.velocity = -0.2;
    
    ROS_INFO("(%.3f|%.3f) %.2f|%.2f (%.2f,%.2f)",yaw, info.rotation+yaw, x_component, y_component, robot -> x, robot -> y);
    //ROS_INFO("(%.3f %.3f) %.3f [%d/%d-%d/%d]", info.rotation, info.velocity, yaw, (int) goal -> destiny.x, (int) goal -> destiny.y, (int) this -> position -> pose.position.x, (int) this -> position -> pose.position.y);


    this -> laser -> setStatus(false);

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

void Navigator::fallback(){
    geometry_msgs::Twist msg;
    msg.linear.x = -0.5;
    msg.angular.z = 0;
    velocity_pub.publish(msg);
//    ros::Duration(1.5).sleep();
    msg.linear.x = 0;
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