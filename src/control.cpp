#include "../include/robot_control/control.h"

Control::Control(ros::NodeHandle n, char* type): search_client(SEARCH_ACTION, true), 
	driveTo_client(DRIVE_TO_ACTION, true),	map_client(CREATE_MAP_ACTION, true), 
	image_client(PROCESS_IMAGE_ACTION, true), node(n),
	searchServer(n, CONTROL_SEARCH_ACTION, boost::bind(&Control::search, this, _1), false),
	exitServer(n, EXIT_ACTION, boost::bind(&Control::exit, this, _1), false),
	driveToServer(n, CONTROL_DRIVE_TO_ACTION, boost::bind(&Control::driveToBombPosition, this, _1), false),
	found(false) {

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
	driveToServer.start();
	exitServer.start();
}

Control::~Control(){}

void Control::exit(const robot_control::exitGoalConstPtr& goal){
	ros::ServiceClient get_map_client = node.serviceClient<robot_control::getMap>(GET_MAP_SERVICE);
	ros::ServiceClient global_path_client = node.serviceClient<robot_control::defineGlobalPath>(DEFINE_GLOBAL_PATH_SERVICE);

	robot_control::exitResult result;
	robot_control::driveToGoal driveTo_goal;
    
    robot_control::defineGlobalPath path_srv;
	robot_control::getMap map_srv;

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

    //ROS_INFO("Calculating path...");
    
    if(global_path_client.call(path_srv)){
	  //  ROS_INFO("Done");

	    driveTo_goal.x_path = path_srv.response.x_path;
	    driveTo_goal.y_path = path_srv.response.y_path;
    }
    else{
	    //ROS_INFO("Could not find path");
    	driveTo_goal.x_path.push_back(this -> start_x);
	    driveTo_goal.y_path.push_back(this -> start_y);	
    }

    driveTo_client.sendGoal(driveTo_goal);

    //ROS_INFO("Exiting...");

    while(!driveTo_client.waitForResult(ros::Duration(1.0)) && ros::ok());
    
    //ROS_INFO("Exited");

    exitServer.setSucceeded(result);
}

void Control::driveToBombPosition(const robot_control::driveToGoalConstPtr& goal){
	robot_control::driveToGoal driveTo_goal;
	robot_control::driveToResult result;
	robot_control::processImageGoal image_goal;
	
	image_client.waitForServer();
	image_client.sendGoal(image_goal);

	driveTo_goal.x_path = goal -> x_path;
	driveTo_goal.y_path = goal -> y_path;
	driveTo_goal.yaw = goal -> yaw;

	driveTo_client.waitForServer();
	driveTo_client.sendGoal(driveTo_goal);
    
	//ROS_INFO("Driving to bomb position...");

    while(found == false && !image_client.waitForResult(ros::Duration(1.0)) && ros::ok()
    	  && !driveTo_client.waitForResult(ros::Duration(1.0)));

    found = false;

	//ROS_INFO("Arrived");

	image_client.cancelGoal();
	driveTo_client.cancelGoal();

	robot_control::driveToResultConstPtr ans = driveTo_client.getResult();

	result.x = this -> x = ans -> x;
	result.y = this -> y = ans -> y;

	//ROS_INFO("Drive to bomb position ended: %lf %lf", result.x, result.y);
    
    driveToServer.setSucceeded(result);
}

void Control::preemptSearch(){
	found = true;
}

void Control::search(const robot_control::searchGoalConstPtr& goal){
	robot_control::searchGoal search_goal;
	robot_control::processImageGoal image_goal;
	robot_control::searchResult result;
	robot_control::alignWithBombResultConstPtr ans;

	startMapping();

	image_client.waitForServer();
	search_client.waitForServer();
	
	image_client.sendGoal(image_goal);
	search_client.sendGoal(search_goal);

	//ROS_INFO("Searching");
	
	//the search will be canceled by the application when one of the robots find the bomb
	//so we don't need to check found == false here

	while(found == false && !image_client.waitForResult(ros::Duration(1.0)) && ros::ok());

	search_client.cancelGoal();
	image_client.cancelGoal();

	found = false;

	robot_control::processImageResultConstPtr img_ans = image_client.getResult();

	if(img_ans ->  succeeded == true){

		while(!search_client.waitForResult(ros::Duration(1.0)) && ros::ok());
		actionlib::SimpleActionClient<robot_control::alignWithBombAction> 
					align_client(ALIGN_WITH_BOMB_ACTION, true);

		ROS_INFO("Waiting for align server...");
		
		align_client.waitForServer();

		ROS_INFO("Aligning with bomb");

		robot_control::alignWithBombGoal align_goal;

		align_client.sendGoal(align_goal);
		while(!align_client.waitForResult(ros::Duration(1.0)) && ros::ok());
		robot_control::alignWithBombResultConstPtr ans = align_client.getResult();

		result.x = this -> x = ans -> x;
		result.y = this -> y = ans -> y;
		result.yaw = this -> yaw = ans -> yaw;
	}
	else{
		robot_control::searchResultConstPtr search_ans = search_client.getResult();
		
		result.x = this -> x = search_ans -> x;
		result.y = this -> y = search_ans -> y;
		result.yaw = this -> yaw = search_ans -> yaw;	
	}

	//ROS_INFO("Bomb found");


	//ROS_INFO("search ended: %lf %lf", result.x, result.y);

    searchServer.setSucceeded(result);
}

void Control::startMapping(){
	robot_control::createMapGoal map_goal;
	map_client.waitForServer();
	map_client.sendGoal(map_goal);
	//ROS_INFO("Mapping");
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