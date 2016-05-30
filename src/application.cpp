#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "../include/robot_control/topics.h"

#include "robot_control/driveToAction.h"
#include "robot_control/driveToResult.h"

#include "robot_control/searchAction.h"
#include "robot_control/searchResult.h"

#include "robot_control/exitAction.h"
#include "robot_control/exitResult.h"

#include "robot_control/alignWithBombAction.h"

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

	aux = "/larger_robot/";
	aux += CONTROL_DRIVE_TO_ACTION;
	ROS_INFO("%s", aux.c_str());

	actionlib::SimpleActionClient<robot_control::driveToAction> 
						driveTo_client_large(aux.c_str(), true);

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
	
	robot_control::driveToGoal large_driveTo_goal;

	ROS_INFO("Getting conections...");
	search_client_large.waitForServer();
	exit_client_large.waitForServer();
	
	search_client_small.waitForServer();
	exit_client_small.waitForServer();

	ROS_INFO("Control connected");
	
	search_client_large.sendGoal(large_search_goal);
	search_client_small.sendGoal(small_search_goal);

	ROS_INFO("Robots searching for the bomb...");

	while(ros::ok()){
		
		if(search_client_large.waitForResult(ros::Duration(1.0))){
			ROS_INFO("Larger robot found the bomb. Exiting smaller robot...");

			search_client_small.cancelGoal();
			while(!search_client_small.waitForResult(ros::Duration(1.0)));
			exit_client_small.sendGoal(small_exit_goal);

			search_client_large.cancelGoal();
			while(!search_client_large.waitForResult(ros::Duration(1.0)));
			
			break;
		}

		if(search_client_small.waitForResult(ros::Duration(1.0))){
			robot_control::searchResultConstPtr ans = search_client_small.getResult();

			large_driveTo_goal.x_path.push_back(ans -> x);
			large_driveTo_goal.y_path.push_back(ans -> y);
			
			ROS_INFO("Larger robot driving to bomb location...");
			search_client_large.cancelGoal();
			while(!search_client_large.waitForResult(ros::Duration(1.0)));
			driveTo_client_large.sendGoal(large_driveTo_goal);
			
			ROS_INFO("Exiting smaller robot...");
			search_client_small.cancelGoal();
			while(!search_client_small.waitForResult(ros::Duration(1.0)));
			exit_client_small.sendGoal(small_exit_goal);

			ROS_INFO("Waiting for larger robot to arrive at bomb location...");
			while(!driveTo_client_large.waitForResult(ros::Duration(1.0)));
			ROS_INFO("Arrived at bomb location");


			aux = "/larger_robot/";
			aux += ALIGN_WITH_BOMB_ACTION;

			actionlib::SimpleActionClient<robot_control::alignWithBombAction> 
						align_client(aux.c_str(), true);

			ROS_INFO("Waiting for align server...");
			
			align_client.waitForServer();

			ROS_INFO("Aligning with bomb");

			robot_control::alignWithBombGoal align_goal;

			align_client.sendGoal(align_goal);
			while(!align_client.waitForResult(ros::Duration(1.0)));
			ROS_INFO("picking up bomb");
			
			break;
		}
	}


	ROS_INFO("Larger robot should approach the bomb now !!!");
	
	ROS_INFO("Exiting larger robot...");

	exit_client_large.sendGoal(large_exit_goal);

	while(!exit_client_small.waitForResult(ros::Duration(1.0)) ||
		  !exit_client_large.waitForResult(ros::Duration(1.0)));

	ROS_INFO("Mission ended successfully !");

	ros::spin();

	return 0;
}