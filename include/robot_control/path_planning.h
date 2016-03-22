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

#define ORTOGONAL_WEIGTH 10
#define DIAGONAL_WEIGTH 14
#define LENGTH 40
#define WIDTH 40

#define CTE 10

#define TO_CELLS(v, c) (floor(v/c))

#define BASE_X(c) (floor((this -> length/c) / 2))
#define BASE_Y(c) (floor((this -> width/c) / 2))

typedef struct node {
	int x, y;
	int F, G, H;
	node* parent;
} Node;

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
	void writeMap(vector<unsigned char> map);
	inline Node* initNode(int x, int y, int g, Node* parent);

	double end_x, end_y;
	double length, width;

	ros::NodeHandle node;
};