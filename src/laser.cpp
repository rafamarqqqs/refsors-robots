#include "../include/robot_control/laser.h"

Laser::Laser(ros::NodeHandle n, char *type){
    node = n;
    laser_sub = node.subscribe(SCAN(type), 1, &Laser::handleSubscription, this);
}

Laser::~Laser(){}

void Laser::handleSubscription(const sensor_msgs::LaserScan::ConstPtr& laser_data){
    int i, j = 0;

    measures.range.clear();
    measures.angle.clear();

    //480 (2PI/3 - PI)/angle increment

    measures.header = laser_data -> header;

    for(i = 0; i < 480; i++){
        measures.range.push_back(laser_data -> ranges[i+120]);
        measures.angle.push_back(ANGLE_MAX - (ANGLE_INCREMENT*i));
    }

    measures.front = 15;

    for(i = (LASER_MEASURES/2) - 10 ; i < (LASER_MEASURES/2) + 10; i++){
        if(laser_data -> ranges[i] < measures.front);
            measures.front = laser_data -> ranges[i];
    }
}

void Laser::publishMeasures(char* type){
    ros::Publisher laser_pub = node.advertise<robot_control::laserMeasures>(LASER(type), 1000);
    ros::Rate r(20.0);

    while(node.ok()){

        ros::spinOnce();

        laser_pub.publish(getMeasures());

        r.sleep();
    }
}

robot_control::laserMeasures Laser::getMeasures(){
    return this -> measures;
}

int main(int argc, char **argv){
    if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    ros::init(argc, argv, "Laser");

    ros::NodeHandle n;

    Laser *laser = new Laser(n, argv[1]);

    ROS_INFO("Laser started.");

    laser -> publishMeasures(argv[1]);

    return 0;
}