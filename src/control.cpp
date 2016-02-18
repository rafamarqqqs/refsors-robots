#include "ros/ros.h"
#include "robot_control/search.h"
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char *argv[]){
	
	ros::init(argc, argv, "control");

	ros::NodeHandle node;

	/*actionlib::SimpleActionClient<robot_control::searchAction> client("NavSearch", true);

	robot_control::searchGoal goal;

	goal.order = 20;

	client.waitForServer();

	if(client.isServerConnected())
		ROS_INFO("connected");

	client.sendGoal(goal);

	client.waitForResult();*/

	ros::ServiceClient client = node.serviceClient<robot_control::search>("search", true);

	robot_control::search srv;

	ROS_INFO("calling service %s", client.getService().c_str());

	if(client.exists())
		ROS_INFO("service exists and is available");
	if(client.isPersistent())
		ROS_INFO("service is persistent");
	if(client.isValid())
		ROS_INFO("service is valid");

	client.call(srv);

	ROS_INFO("done");

	return 0;
}