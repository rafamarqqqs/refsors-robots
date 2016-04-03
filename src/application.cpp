#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../include/robot_control/topics.h"
#include "robot_control/searchAction.h"
#include "robot_control/searchResult.h"
#include "robot_control/exitAction.h"
#include "robot_control/exitResult.h"
#include <string>

using namespace std;

int main(int argc, char** argv){

	ros::init(argc, argv, APPLICATION_NODE);

	string aux;

	aux = "/larger_robot/";
	aux += CONTROL_SEARCH_ACTION;
	ROS_INFO("%s", aux.c_str());
	
	actionlib::SimpleActionClient<robot_control::searchAction> 
				search_client_large(aux.c_str(), true);

	aux = "/larger_robot/";
	aux += EXIT_ACTION;
	ROS_INFO("%s", aux.c_str());

	actionlib::SimpleActionClient<robot_control::exitAction> 
						exit_client_large(aux.c_str(), true);

	aux = "/smaller_robot/";
	aux += CONTROL_SEARCH_ACTION;
	ROS_INFO("%s", aux.c_str());
	
	actionlib::SimpleActionClient<robot_control::searchAction> 
				search_client_small(aux.c_str(), true);

	aux = "/smaller_robot/";
	aux += EXIT_ACTION;
	ROS_INFO("%s", aux.c_str());

	actionlib::SimpleActionClient<robot_control::exitAction> 
						exit_client_small(aux.c_str(), true);


	robot_control::searchGoal large_search_goal;
	robot_control::searchGoal small_search_goal;

	robot_control::exitGoal large_exit_goal;
	robot_control::exitGoal small_exit_goal;

	ROS_INFO("Getting conections...");
	search_client_large.waitForServer();
	exit_client_large.waitForServer();
	
	search_client_small.waitForServer();
	exit_client_small.waitForServer();

	ROS_INFO("Control connected");
	
	search_client_large.sendGoal(large_search_goal);
	search_client_small.sendGoal(small_search_goal);

	ROS_INFO("Robots searching");

	while(ros::ok()){
		
		if(search_client_large.waitForResult(ros::Duration(1.0))){
			search_client_small.cancelGoal();
			break;
		}

		if(search_client_small.waitForResult(ros::Duration(1.0))){
			search_client_large.cancelGoal();
			break;
		}
	}

	while(!search_client_large.waitForResult(ros::Duration(1.0)));
	while(!search_client_small.waitForResult(ros::Duration(1.0)));

	ROS_INFO("Exiting robots");

	exit_client_small.sendGoal(small_exit_goal);
	while(!exit_client_small.waitForResult(ros::Duration(1.0)));

	exit_client_large.sendGoal(large_exit_goal);
	while(!exit_client_large.waitForResult(ros::Duration(1.0)));

	ROS_INFO("Mission ended");

	ros::spin();

	return 0;
}