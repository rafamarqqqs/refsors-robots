#include "ros/ros.h"
#include "resources.h"

#include "robot_control/searchAction.h"
#include "robot_control/searchResult.h"

#include "robot_control/exitAction.h"

#include "robot_control/driveToAction.h"

#include "robot_control/processImageAction.h"
#include "robot_control/processImageResult.h"

#include "robot_control/createMapAction.h"
#include "robot_control/defineGlobalPath.h"
#include "robot_control/getMap.h"

#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<robot_control::searchAction> SearchAction;
typedef actionlib::SimpleActionServer<robot_control::exitAction> ExitAction;

class Control {
public:
	Control(ros::NodeHandle n, char* type);
	~Control();

	bool search(const robot_control::searchGoalConstPtr& goal);
	bool exit(const robot_control::exitGoalConstPtr& goal);

private:
	void startMapping();
	void preemptSearch();

	actionlib::SimpleActionClient<robot_control::searchAction> search_client;
	actionlib::SimpleActionClient<robot_control::driveToAction> driveTo_client;
	actionlib::SimpleActionClient<robot_control::createMapAction> map_client;
	actionlib::SimpleActionClient<robot_control::processImageAction> image_client;

	SearchAction searchServer;
	ExitAction exitServer;
	ros::NodeHandle node;

	float cell_size;
	bool found;
	double x, y, start_x, start_y;
};