#include "../include/robot_control/control.h"

Control::Control(ros::NodeHandle n, char* type): search_client(SEARCH_ACTION, true), 
	driveTo_client(DRIVE_TO_ACTION, true),	map_client(CREATE_MAP_ACTION, true), 
	image_client(PROCESS_IMAGE_ACTION, true), node(n),
	searchServer(n, CONTROL_SEARCH_ACTION, boost::bind(&Control::search, this, _1), false),
	exitServer(n, EXIT_ACTION, boost::bind(&Control::exit, this, _1), false), found(false) {

	if(!strcmp(type, LARGER_ROBOT)){
		this -> cell_size = 1.0;
		this -> start_x = -18;
		this -> start_y = 18;
	}
	else{
		this -> cell_size = 0.5;
		this -> start_x = -14;
		this -> start_y = 16;
	}

	searchServer.registerPreemptCallback(boost::bind(&Control::preemptSearch, this));

	searchServer.start();
	exitServer.start();
}

Control::~Control(){}

bool Control::exit(const robot_control::exitGoalConstPtr &goal){
	robot_control::exitResult result;
	robot_control::driveToGoal driveTo_goal;
    
    robot_control::defineGlobalPath path_srv;
	robot_control::getMap map_srv;
    
	ros::ServiceClient global_path_client = node.serviceClient<robot_control::defineGlobalPath>(DEFINE_GLOBAL_PATH_SERVICE);

	ros::ServiceClient get_map_client = node.serviceClient<robot_control::getMap>(GET_MAP_SERVICE);

	// GETTING MAP
    map_srv.request.cell_size = cell_size;
    get_map_client.call(map_srv);

    // CALCULATING PATH
    path_srv.request.x = this -> x;
    path_srv.request.y = this -> y;
    path_srv.request.destiny_x = this -> start_x;
    path_srv.request.destiny_y = this -> start_y;
    path_srv.request.cell_size = cell_size;
    path_srv.request.map = map_srv.response.map;

    ROS_INFO("Calculating path...");
    global_path_client.call(path_srv);
    ROS_INFO("Done");

    driveTo_goal.x = path_srv.response.x_path;
    driveTo_goal.y = path_srv.response.y_path;

    driveTo_client.sendGoal(driveTo_goal);

    ROS_INFO("Exiting...");

    while(!driveTo_client.waitForResult(ros::Duration(1.0)));
    
    ROS_INFO("Exited");

    exitServer.setSucceeded(result);

    return true;
}

void Control::preemptSearch(){
	found = true;
}

bool Control::search(const robot_control::searchGoalConstPtr &goal){
	robot_control::searchGoal search_goal;
	robot_control::processImageGoal image_goal;
	robot_control::searchResult result;
	
	startMapping();

	image_client.waitForServer();
	search_client.waitForServer();
	
	image_client.sendGoal(image_goal);
	search_client.sendGoal(search_goal);

	ROS_INFO("Searching");
	
	while(found == false && !image_client.waitForResult(ros::Duration(1.0)));
	
	ROS_INFO("Found bomb !!");

	search_client.cancelGoal();

	while(!search_client.waitForResult(ros::Duration(1.0)));

	robot_control::searchResultConstPtr ans = search_client.getResult();

	ROS_INFO("%lf %lf", ans -> x, ans -> y);

	result.x = this -> x = ans -> x;
	result.y = this -> y = ans -> y;

    searchServer.setSucceeded(result);

	return true;
}

void Control::startMapping(){
	robot_control::createMapGoal map_goal;
	map_client.waitForServer();
	map_client.sendGoal(map_goal);
	ROS_INFO("Mapping");
}

int main(int argc, char *argv[]){
	
	ros::init(argc, argv, CONTROL_NODE);

	if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

	ROS_INFO("Control started.");

	ros::NodeHandle node;

	Control* control = new Control(node, argv[1]);

	ros::spin();

	return 0;
}