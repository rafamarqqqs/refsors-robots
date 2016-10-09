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

class Application {
public:
	Application(ros::NodeHandle n, string larger_robot_search, string larger_robot_exit, 
		string larger_robot_driveTo, string smaller_robot_search, string smaller_robot_exit);
	~Application();

	void startMission();

private:

	actionlib::SimpleActionClient<robot_control::searchAction> search_client_large;
	actionlib::SimpleActionClient<robot_control::exitAction> exit_client_large;
	actionlib::SimpleActionClient<robot_control::driveToAction> driveTo_client_large;
	actionlib::SimpleActionClient<robot_control::searchAction> search_client_small;
	actionlib::SimpleActionClient<robot_control::exitAction> exit_client_small;
	
	ros::NodeHandle node;
};