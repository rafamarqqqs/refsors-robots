#include "../include/robot_control/knowledge.h"

using namespace std;

Knowledge::Knowledge(ros::NodeHandle n, double length, double width, double cell_size){

    this -> length = length;
    this -> width = width;
    this -> cell_size = cell_size;
    this -> ids = 0;
    this -> positions = new vector<_2DPoint>();
    mapped = false;

    initMaps();
}

Knowledge::~Knowledge(){
    int i, j;



    writeMap();
    
    for(i = 0; i < TO_CELLS(length); i++){
        free(map[i]);
        free(map_scans[i]);
    }

    delete positions;
}

void Knowledge::initMaps(){
    int i, j;

    map = (char **) malloc(sizeof(char *) * TO_CELLS(length));
    map_scans = (unsigned short **) malloc(sizeof(unsigned short *) * TO_CELLS(length));

    for(i = 0; i < TO_CELLS(length); i++){
        map[i] = (char *) malloc(sizeof(char) * TO_CELLS(width));
        map_scans[i] = (unsigned short *) calloc(TO_CELLS(width), sizeof(unsigned short));
    }
    
    for(i = 0; i < TO_CELLS(length); i++){
        for(j = 0; j < TO_CELLS(width); j++){
            map[i][j] = UNKNOWN;
        }
    }
}

bool Knowledge::getPositions(robot_control::getPositions::Request& req, robot_control::getPositions::Response& res){
    _2DPoint aux;
    vector<_2DPoint>::iterator it;
    int i;

    aux.x = req.my_x;
    aux.y = req.my_y;

    if(req.has_id == false){
        req.my_id = res.id = ids;
        ids++;
        positions -> push_back(aux);
    }
    else
        positions -> at(req.my_id) = aux;

    for(i = 0, it = positions -> begin(); it != positions -> end(); it++, i++){
        if(i != req.my_id){
            res.x.push_back((*it).x);
            res.y.push_back((*it).y);
        }
    }

    return true;
}

bool Knowledge::addToMap(robot_control::addToMap::Request& req, robot_control::addToMap::Response& res){

    if(!INSIDE(req.wall_x, req.wall_y))
        return false;

    int x = BASE_X + TO_CELLS(req.wall_x);
    int y = BASE_Y + TO_CELLS(req.wall_y);
    double aux_x = req.start_x;
    double aux_y = req.start_y;
    double range, aux_average;
    double range_inc = pow(pow(req.inc_x, 2) + pow(req.inc_y, 2), 0.5);
    int map_x = floor(TO_CELLS(aux_x)) + BASE_X;
    int map_y = floor(TO_CELLS(aux_y)) + BASE_Y;

    range = range_inc;

    while(map_x != x && map_y != y && MAX_RANGE > range){

        map[map_x][map_y] = (unsigned short) floor(map[map_x][map_y]/(++map_scans[map_x][map_y]));

        range += range_inc;

        aux_x += req.inc_x;
        aux_y += req.inc_y;
        
        map_x = floor(TO_CELLS(aux_x)) + BASE_X;
        map_y = floor(TO_CELLS(aux_y)) + BASE_Y;
    }

    if(req.range < MAX_RANGE){
        aux_average = map[x][y] * (map_scans[x][y]++);
        map[x][y] = (unsigned short) floor(aux_average + FULL)/(map_scans[x][y]);
    }

    //writeMap();

    res.added = true;

    if(req.start_x > 0 && req.start_y < 3 && !mapped){
        mapped = true;
        ros::ServiceClient client = node.serviceClient<robot_control::defineGlobalPath>("/PathPlanning/defineGlobalPath");

        robot_control::defineGlobalPath srv;

        srv.request.x = req.start_x;
        srv.request.y = req.start_y;
        srv.request.destiny_x = -14;
        srv.request.destiny_y = 16;
        srv.request.cell_size = 0.5;

        for(int i = 0; i < TO_CELLS(length); i++){
            for(int j = 0; j < TO_CELLS(width); j++){
                srv.request.map.push_back((unsigned char) map[i][j]);
            }
        }

        client.call(srv);
    }

    return true;
}

void Knowledge::writeMap(){
    int i, j;
    char block;

    std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/containers.map");

    for(i = 0; i < TO_CELLS(length); i++){
        for(j = 0; j < TO_CELLS(width); j++){
            if(map[i][j] < EMPTY_RANGE)
                block = ' ';
            else if(map[i][j] > FULL_RANGE)
                block = '#';
            else
                block = ':';

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

    ros::ServiceServer addToMap_service = node.advertiseService("/Knowledge/addToMap", 
                                                    &Knowledge::addToMap, knowledge);
    ros::ServiceServer getPositions_service = node.advertiseService("/Knowledge/getPositions", 
                                                    &Knowledge::getPositions, knowledge);

    ROS_INFO("Knowledge started.");

    ros::spin();

    delete knowledge;

    return 0;
}