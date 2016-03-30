#include "ros/ros.h"
#include "../include/robot_control/resources.h"
#include "robot_control/searchAction.h"
#include "robot_control/searchResult.h"
#include "robot_control/createMapAction.h"
#include "robot_control/defineGlobalPath.h"
#include "robot_control/getMap.h"
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char *argv[]){
	
	ros::init(argc, argv, CONTROL_NODE);

	ROS_INFO("Control started.");

	ros::NodeHandle node;

	actionlib::SimpleActionClient<robot_control::searchAction> search_client(SEARCH_ACTION, true);
	actionlib::SimpleActionClient<robot_control::createMapAction> map_client(CREATE_MAP_ACTION, true);

	robot_control::searchGoal search_goal;
	robot_control::createMapGoal map_goal;

	search_client.waitForServer();
	map_client.waitForServer();

	/*if(search_client.isServerConnected())
		ROS_INFO("Search connected");
	if(map_client.isServerConnected())
		ROS_INFO("Map connected");*/

	map_client.sendGoal(map_goal);
	search_client.sendGoal(search_goal);

	ROS_INFO("called search");
	
	while(!search_client.waitForResult(ros::Duration(100.0)));
	//map_client.waitForResult();
	
	ROS_INFO("done searching");

	robot_control::searchResultConstPtr ans = search_client.getResult();

	if(!strcmp(argv[1], LARGER_ROBOT)){
		ROS_INFO("larger robot waiting");
		ros::spin();
	}

	double x = ans -> x;
	double y = ans -> y;

	ROS_INFO("robot pose %lf %lf", x, y);


	// GETTING MAP
	ros::ServiceClient get_map_client = node.serviceClient<robot_control::getMap>(GET_MAP_SERVICE);
    robot_control::getMap map_srv;

    map_srv.request.cell_size = 0.5;

    get_map_client.call(map_srv);

    // CALCULATING PATH
	ros::ServiceClient global_path_client = node.serviceClient<robot_control::defineGlobalPath>(DEFINE_GLOBAL_PATH_SERVICE);
    robot_control::defineGlobalPath srv;

    srv.request.x = x;
    srv.request.y = y;
    srv.request.destiny_x = -14;
    srv.request.destiny_y = 16;
    srv.request.cell_size = 0.5;

    /*for(int i = 0; i < TO_CELLS(length); i++){
        for(int j = 0; j < TO_CELLS(width); j++){
            srv.request.map.push_back((unsigned char) map[i][j]);
        }
    }*/

    srv.request.map = map_srv.response.map;

    ROS_INFO("calling service");
    global_path_client.call(srv);
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

	ros::spin();

	return 0;
}