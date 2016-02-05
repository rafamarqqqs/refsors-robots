#include "../include/robot_control/localization.h"

Localizator::Localizator(){
}

Localizator::~Localizator(){
}

void Localizator::getPosition(){
	odom_listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1));
    odom_listener.lookupTransform("/odom", "/base_link", ros::Time(0), odom_transform);

    this -> pose.position.x = odom_transform.getOrigin().x();
    this -> pose.position.y = odom_transform.getOrigin().y();
}

void Localizator::getYaw(){
	imu_listener.waitForTransform("/base_link", "/imu_link", ros::Time(0), ros::Duration(1));
    imu_listener.lookupTransform("/base_link", "/imu_link", ros::Time(0), imu_transform);

    this -> pose.orientation.x = imu_transform.getRotation().x();
    this -> pose.orientation.y = imu_transform.getRotation().y();
    this -> pose.orientation.z = imu_transform.getRotation().z();
    this -> pose.orientation.w = imu_transform.getRotation().w();
}

geometry_msgs::Pose Localizator::getPose(){
	return this -> pose;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pose_publisher");

  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/larger_robot/pose", 1000);

  Localizator localizator;

  ros::Rate r(10.0);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    localizator.getYaw();
    localizator.getPosition();

    //next, we'll publish the odometry message over ROS
    geometry_msgs::Pose pose;
    geometry_msgs::Pose aux;

    //set the position
    aux = localizator.getPose();

    pose.position.x = aux.position.x;
    pose.position.y = aux.position.y;

    pose.orientation.x = aux.orientation.x;
    pose.orientation.y = aux.orientation.y;
    pose.orientation.z = aux.orientation.z;
    pose.orientation.w = aux.orientation.w;

    //publish the message
    pose_pub.publish(pose);

    r.sleep();
  }
}