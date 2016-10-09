#include "ros/ros.h"
#include "resources.h"

#include "robot_control/searchAction.h"
#include "robot_control/searchResult.h"

#include "robot_control/exitAction.h"
#include "robot_control/exitResult.h"

#include "robot_control/driveToAction.h"
#include "robot_control/driveToResult.h"

#include "robot_control/processImageAction.h"
#include "robot_control/processImageResult.h"

#include "robot_control/createMapAction.h"
#include "robot_control/defineGlobalPath.h"
#include "robot_control/getMap.h"

#include "robot_control/alignWithBombAction.h"

#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<robot_control::searchAction> SearchAction;
typedef actionlib::SimpleActionServer<robot_control::driveToAction> DriveToAction;
typedef actionlib::SimpleActionServer<robot_control::exitAction> ExitAction;

class Control {
public:
	Control(ros::NodeHandle n, char* type);
	~Control();

	void search(const robot_control::searchGoalConstPtr& goal);
	void exit(const robot_control::exitGoalConstPtr& goal);
	void driveToBombPosition(const robot_control::driveToGoalConstPtr& goal);

private:
	void startMapping();
	void preemptSearch();

	actionlib::SimpleActionClient<robot_control::searchAction> search_client;
	actionlib::SimpleActionClient<robot_control::driveToAction> driveTo_client;
	actionlib::SimpleActionClient<robot_control::createMapAction> map_client;
	actionlib::SimpleActionClient<robot_control::processImageAction> image_client;

	SearchAction searchServer;
	ExitAction exitServer;
	DriveToAction driveToServer;

	ros::NodeHandle node;

	//ros::ServiceClient global_path_client;
	//ros::ServiceClient get_map_client;

	float cell_size;
	bool found;
	double x, y, start_x, start_y, yaw;
};