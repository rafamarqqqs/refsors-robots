#include "../include/robot_control/knowledge.h"


Knowledge::Knowledge(ros::NodeHandle n, double length, double width, double cell_size){

    this -> length = length;
    this -> width = width;
    this -> cell_size = cell_size;

    initMaps();
}

Knowledge::~Knowledge(){
    delete map;
    delete map_scans;
}

void Knowledge::initMaps(){
    int i, j;

    map = (float **) malloc(sizeof(float *) * TO_CELLS(length));
    map_scans = (int **) malloc(sizeof(int *) * TO_CELLS(length));

    for(i = 0; i < TO_CELLS(length); i++){
        map[i] = (float *) malloc(sizeof(float) * TO_CELLS(width));
        map_scans[i] = (int *) calloc(TO_CELLS(width), sizeof(int));
        //memset(&(map_scans[i]), 0.5, sizeof(float) * TO_CELLS(width));
    }
    
    for(i = 0; i < TO_CELLS(length); i++){
        for(j = 0; j < TO_CELLS(width); j++){
            map[i][j] = UNKNOWN;
        }
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
    double aux_x = req.start_x;
    double aux_y = req.start_y;
    double dist = 0;
    int map_x = 1000, map_y = 1000;
    double range = 0;
    double range_inc = pow(pow(req.inc_x, 2) + pow(req.inc_y, 2), 0.5);
    double aux_average;

    map_x = ((int) TO_CELLS(aux_x)) + BASE_X;
    map_y = ((int) TO_CELLS(aux_y)) + BASE_Y;

    //while(COMPARE(req.start_x, aux.x, aux_x) && COMPARE(req.start_y, aux.y, aux_y)){
    while(map_x != x && map_y != y && MAX_RANGE > range){

        map_scans[map_x][map_y]++;

        /*
        if(map[map_x][map_y] == UNKNOWN)
            map[map_x][map_y] = EMPTY;
        else if(map[map_x][map_y] == FULL){
            wall = true;
            break;
        }
        */

        map[map_x][map_y] = map[map_x][map_y]/(map_scans[map_x][map_y]*1.0);

        range += range_inc;

        aux_x += req.inc_x;
        aux_y += req.inc_y;
        
        map_x = ((int) TO_CELLS(aux_x)) + BASE_X;
        map_y = ((int) TO_CELLS(aux_y)) + BASE_Y;
    }

    if(req.range < MAX_RANGE /*&& map[map_x][map_y] == UNKNOWN && !wall*/){
        aux_average = map[x][y] * (map_scans[x][y]*1.0);

        map_scans[x][y]++;

        map[x][y] = (aux_average + FULL)/(map_scans[x][y]*1.0);
    }

    writeMap();

    res.added = true;

    return true;
}

void Knowledge::writeMap(){
    int i, j;
    char block;

    std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/containers.map");

    for(i = 0; i < TO_CELLS(length); i++){
        for(j = 0; j < TO_CELLS(width); j++){
            if(map[i][j] < 0.2)
                block = ' ';
            else if(map[i][j] > 0.8)
                block = '#';
            else
                block = ':';
            //VISUALIZATION PURPOSES, REMOVE WHEN READY
            file.put(block);
            file.put(block);
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

    delete knowledge;

    return 0;
}