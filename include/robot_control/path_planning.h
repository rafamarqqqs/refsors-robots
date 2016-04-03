#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <unordered_set>
#include <cmath>
#include <fstream>
#include <algorithm>
#include "resources.h"
#include "robot_control/defineGlobalPath.h"
#include <actionlib/server/simple_action_server.h>

using namespace std;

inline bool cmpNodes(Node* a, Node* b){
    return a -> F < b -> F;
}

class PathPlanning{

public:
	PathPlanning(ros::NodeHandle node, double length, double width);
	~PathPlanning();

	bool defineGlobalPath(robot_control::defineGlobalPath::Request& req,
                      	  robot_control::defineGlobalPath::Response& res);

private:
	void defineLocalPath(vector<int>* x_path, vector<int>* y_path);
	inline vector<Node*>::iterator find(vector<Node*>* vec, int x, int y);
	void writeMap(vector<unsigned char> map, float cell_size);
	inline Node* initNode(int x, int y, int g, Node* parent);

	double end_x, end_y;
	double length, width;

	ros::NodeHandle node;
};