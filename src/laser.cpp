#include "../include/robot_control/laser.h"

Laser::Laser(ros::NodeHandle n, char *type): node(n), front_size(FRONT_SIZE(type)), min_front(MIN_FRONT(type)) {

    laser_sub = node.subscribe(SCAN_TOPIC(type), 1, &Laser::handleSubscription, this);
}

Laser::~Laser(){}

void Laser::handleSubscription(const sensor_msgs::LaserScan::ConstPtr& laser_data){
    int i, j = 0;

    measures.range.clear();
    measures.angle.clear();

    //480 (2PI/3 - PI)/angle increment

    measures.header = laser_data -> header;
    //ROS_INFO("%3.2f", (laser_data -> angle_increment*360)/(2*M_PI) * 119);
    for(i = 0; i < 480; i++){
        measures.range.push_back(laser_data -> ranges[i+120]);
        measures.angle.push_back(ANGLE_MAX - (ANGLE_INCREMENT*i));
    }

    //ROS_INFO("%4.4lf", laser_data -> ranges[360]);

    measures.front = min_front;

    for(i = (LASER_MEASURES/2) - front_size; i < (LASER_MEASURES/2) + front_size; i++){
        if(laser_data -> ranges[i] < measures.front);
            measures.front = laser_data -> ranges[i];
    }
}

inline robot_control::laserMeasures Laser::getMeasures(){
    return this -> measures;
}

void Laser::publishMeasures(char* type){
    ros::Publisher laser_pub = node.advertise<robot_control::laserMeasures>(LASER_TOPIC(type), 1);
    ros::Rate r(20.0);

    while(node.ok() && ros::ok()){

        ros::spinOnce();

        laser_pub.publish(getMeasures());

        r.sleep();
    }
}


int main(int argc, char **argv){
    if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    ros::init(argc, argv, LASER_NODE);

    ros::NodeHandle n;

    Laser *laser = new Laser(n, argv[1]);

    ROS_INFO("Laser started.");

    laser -> publishMeasures(argv[1]);

    delete laser;

    return 0;
}