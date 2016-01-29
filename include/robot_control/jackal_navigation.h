#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h> 

class JackalNavigator {
public:
  JackalNavigator(ros::NodeHandle node);
  virtual ~JackalNavigator();

private:
  void defineRobotSpeed(const sensor_msgs::LaserScan::ConstPtr &laserData);
  ros::Publisher velocity_pub;
  ros::Subscriber laser;
  ros::NodeHandle node;

};