#include "ros/ros.h"
#include "robot_control/searchAction.h"
#include "robot_control/createMapAction.h"
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char *argv[]){
	
	ros::init(argc, argv, "Control");

	ROS_INFO("Control started.");

	ros::NodeHandle node;

	actionlib::SimpleActionClient<robot_control::searchAction> search_client("search", true);
	actionlib::SimpleActionClient<robot_control::createMapAction> map_client("createMap", true);

	robot_control::searchGoal search_goal;
	robot_control::createMapGoal map_goal;

	search_client.waitForServer();
	map_client.waitForServer();

	if(search_client.isServerConnected())
		ROS_INFO("Search connected");
	if(map_client.isServerConnected())
		ROS_INFO("Map connected");

	map_client.sendGoal(map_goal);
	search_client.sendGoal(search_goal);
	
	search_client.waitForResult();
	map_client.waitForResult();
	
	ROS_INFO("done");
	/*

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

	*/

	return 0;
}