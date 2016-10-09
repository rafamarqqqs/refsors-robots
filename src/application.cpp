#include "../include/robot_control/application.h"

Application::Application(ros::NodeHandle n, string larger_robot_search, string larger_robot_exit, 
		string larger_robot_driveTo, string smaller_robot_search, string smaller_robot_exit):

	search_client_large(larger_robot_search, true),
	exit_client_large(larger_robot_exit, true),
	driveTo_client_large(larger_robot_driveTo, true),
	search_client_small(smaller_robot_search, true),
	exit_client_small(smaller_robot_exit, true){

	this -> node = n;
}

Application::~Application(){}

void Application::startMission(){

	string aux;

	robot_control::searchGoal large_search_goal;
	robot_control::searchGoal small_search_goal;

	robot_control::exitGoal large_exit_goal;
	robot_control::exitGoal small_exit_goal;
	
	robot_control::driveToGoal large_driveTo_goal;

	search_client_large.waitForServer();
	exit_client_large.waitForServer();

	search_client_small.waitForServer();
	exit_client_small.waitForServer();

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
			
			ROS_INFO("Smaller robot found the robot. Larger robot driving to bomb location...");
			search_client_large.cancelGoal();
			while(!search_client_large.waitForResult(ros::Duration(1.0)));
			driveTo_client_large.sendGoal(large_driveTo_goal);
			
			ROS_INFO("Exiting smaller robot...");
			search_client_small.cancelGoal();
			while(!search_client_small.waitForResult(ros::Duration(1.0)));
			exit_client_small.sendGoal(small_exit_goal);

			ROS_INFO("Waiting for larger robot to arrive at bomb location...");
			while(!driveTo_client_large.waitForResult(ros::Duration(1.0)));
			ROS_INFO("Arrived at bomb location...");


			aux = "/larger_robot/";
			aux += ALIGN_WITH_BOMB_ACTION;

			actionlib::SimpleActionClient<robot_control::alignWithBombAction> 
						align_client(aux.c_str(), true);

			align_client.waitForServer();

			ROS_INFO("Aligning with bomb...");

			robot_control::alignWithBombGoal align_goal;

			align_client.sendGoal(align_goal);
			while(!align_client.waitForResult(ros::Duration(1.0)));
			
			break;
		}
	}

	ROS_INFO("Picking up bomb...");
	
	ROS_INFO("Exiting larger robot...");

	exit_client_large.sendGoal(large_exit_goal);

	while(!exit_client_small.waitForResult(ros::Duration(1.0)) ||
		  !exit_client_large.waitForResult(ros::Duration(1.0)));

	ROS_INFO("Mission ended successfully !");
	
}

int main(int argc, char** argv){

	ros::init(argc, argv, APPLICATION_NODE);

	ros::NodeHandle node;
	
	ROS_INFO("Application started.");

	string larger_robot_search = "/larger_robot/";
	larger_robot_search += CONTROL_SEARCH_ACTION;
	
	string larger_robot_exit = "/larger_robot/";
	larger_robot_exit += EXIT_ACTION;

	string larger_robot_driveTo = "/larger_robot/";
	larger_robot_driveTo += CONTROL_DRIVE_TO_ACTION;

	string smaller_robot_search = "/smaller_robot/";
	smaller_robot_search += CONTROL_SEARCH_ACTION;

	string smaller_robot_exit = "/smaller_robot/";
	smaller_robot_exit += EXIT_ACTION;

	Application* app = new Application(node, larger_robot_search, larger_robot_exit, larger_robot_driveTo, 
									smaller_robot_search, smaller_robot_exit);

	app -> startMission();
	
	ros::spin();

	delete app;

	return 0;
}