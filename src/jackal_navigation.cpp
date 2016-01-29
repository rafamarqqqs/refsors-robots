#include "../include/robot_control/jackal_navigation.h"

void JackalNavigator::defineRobotSpeed(const sensor_msgs::LaserScan::ConstPtr &laserData){
  geometry_msgs::Twist msg;

  if(laserData != NULL && laserData -> ranges[320] < 3.0){ 
    msg.linear.x = 2;
    msg.angular.z = 4;
  }
  else{
    msg.linear.x = 1;
    msg.angular.z = 0;
  }

  velocity_pub.publish(msg);
}

JackalNavigator::JackalNavigator(ros::NodeHandle n){
  node = n;
  velocity_pub = node.advertise<geometry_msgs::Twist>("/smaller_robot/cmd_vel", 100);
  laser = node.subscribe("/smaller_robot/front/scan", 100, &JackalNavigator::defineRobotSpeed, this);
}

JackalNavigator::~JackalNavigator(){}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "Jackal_commands");
    
    ros::NodeHandle node;

    JackalNavigator *nav = new JackalNavigator(node);

    ROS_INFO("Jackal navigator started.");

    //Sets the loop to publish at a rate of 10Hz
    ros::Rate rate(10);

      while(ros::ok()) {
        //msg.linear.x=1;//*double(rand())/double(RAND_MAX)-2;
        //msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
        //Delays untill it is time to send another message
        rate.sleep();
        //calls the callback function that defines the new velocity
        ros::spinOnce();
      }
}