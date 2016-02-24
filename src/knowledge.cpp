#include "../include/robot_control/knowledge.h"


Knowledge::Knowledge(ros::NodeHandle n, float length, float width, float cell_size){

    this -> length = length;
    this -> width = width;
    this -> cell_size = cell_size;

    initMap();
}

Knowledge::~Knowledge(){
    delete map; 
}

void Knowledge::initMap(){
    int i, j;

    map = (char **) malloc(sizeof(char *)*TO_CELLS(length));

    for(i = 0; i < TO_CELLS(length); i++)
        map[i] = (char *) malloc(sizeof(char) * TO_CELLS(width));
    
    for(i = 0; i < TO_CELLS(length); i++){
        for(j = 0; j < TO_CELLS(width); j++)
            map[i][j] = UNKNOWN;

    }
}

bool Knowledge::addToMap(robot_control::addToMap::Request& req, robot_control::addToMap::Response& res){
    _2DPoint aux;

    aux.x = req.wall_x;
    aux.y = req.wall_y;

    if(!INSIDE(aux))
        return false;

    int x = (int) BASE_X + TO_CELLS(aux.x);
    int y = (int) BASE_Y + TO_CELLS(aux.y);
    float aux_x = req.start_x;
    float aux_y = req.start_y;
    int map_x, map_y;

    while(COMPARE(req.start_x, aux.x, aux_x) && COMPARE(req.start_y, aux.y, aux_y)){
        map_x = ((int) TO_CELLS(aux_x)) + BASE_X;
        map_y = ((int) TO_CELLS(aux_y)) + BASE_Y;

        if(map[map_x][map_y] != FULL)
            map[map_x][map_y] = EMPTY;

        aux_x += req.inc_x;
        aux_y += req.inc_y;
    }

    map[x][y] = FULL;

    writeMap();

    res.added = true;

    return true;
}

void Knowledge::writeMap(){
    int i, j;

    std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/containers.map");

    for(i = 0; i < TO_CELLS(length); i++){
        for(j = 0; j < TO_CELLS(width); j++){
            file.put(map[i][j]);
            //VISUALIZATION PURPOSES, REMOVE WHEN READY
            //file.put(map[i][j]);
        }

        file.put('\n');
    }

    file.close();
}

int main(int argc, char **argv) {

    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "Knowledge");

    ros::NodeHandle node;

    Knowledge *knowledge = new Knowledge(node, 40.0, 40.0, 0.5);

    ros::ServiceServer service = node.advertiseService("/Knowledge/addToMap", &Knowledge::addToMap, knowledge);

    ROS_INFO("Knowledge started.");

    ros::spin();

    return 0;
}