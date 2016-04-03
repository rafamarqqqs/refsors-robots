#include "topics.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <actionlib/server/simple_action_server.h>
#include "robot_control/processImageAction.h"

typedef actionlib::SimpleActionServer<robot_control::processImageAction> ProcessImageAction; 

class ImageProcessing{
public:
	ImageProcessing(ros::NodeHandle node, char* type);
	~ImageProcessing();

	void handleImage(const sensor_msgs::ImageConstPtr& img);
	bool processImage(const robot_control::processImageGoalConstPtr& image_goal);

private:
	ros::NodeHandle node;
	ProcessImageAction processImageServer;
	ros::Subscriber camera_sub;
	int found;
};