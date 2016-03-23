#include "../include/robot_control/knowledge.h"

using namespace std;

Knowledge::Knowledge(ros::NodeHandle n, double len, double wid, double cell_s, double area_s):
    node(n), length(len), width(wid), cell_size(cell_s), area_size(area_s), ids(0), mapped(false){

    this -> positions = new vector<_2DPoint>();

    initMaps();
    initAreas();
}

Knowledge::~Knowledge(){
    int i, j;

    writeMap();
    
    for(i = 0; i < TO_CELLS(length); i++){
        free(map[i]);
        free(map_scans[i]);
    }

    delete positions;

    for(i = 0; i < floor(length/area_size); i++)
        free(areas[i]);

    free(areas);
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

void Knowledge::initAreas(){
    int i, j;

    areas = (uint8_t **) malloc(sizeof(uint8_t *) * floor(length/area_size));

    for(i = 0; i < floor(length/area_size); i++){
        areas[i] = (uint8_t *) malloc(sizeof(uint8_t) * floor(width/area_size));
        
        for(j = 0; j < floor(width/area_size); j++)
            areas[i][j] = EMPTY;
    }
}

bool Knowledge::getNewGoal(robot_control::getNewGoal::Request& req, robot_control::getNewGoal::Response& res){
    vector<uint8_t> occupied_areas = req.occupied_areas;
    vector<uint8_t>::iterator it;
    int i = 0, j = 0;

    for(it = occupied_areas.begin(); it != occupied_areas.end(); it++){
        areas[i][j] = (*it);

        j++;

        if(j == AREA_SIZE){
            j = 0;
            i++;
        }
    } 

    srand(time(NULL));

    res.new_x = rand() % ((int) floor(length/area_size));
    res.new_y = rand() % ((int) floor(width/area_size));

    if(req.x == INT_MAX && req.y == INT_MAX){
        while(areas[res.new_x][res.new_y] == OCCUPIED){
            res.new_x = rand() % ((int) floor(length/area_size));
            res.new_y = rand() % ((int) floor(width/area_size));
        }
    }

    writeAreas();

    return true;
}

void Knowledge::writeAreas(){
    int counter = 0;
    std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/areas.map");

    for(int i = 0; i < floor(length/area_size); i++){
        for(int j = 0; j < floor(width/area_size); j++){

        file << "|";
        
        file << std::setfill(' ') << (areas[i][j] == OCCUPIED ? '1' : '0');
        
        }

        file << "|";
        file << "\n";
    }

    file << "\n";
    file.close();
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

    Knowledge *knowledge = new Knowledge(node, LENGTH, WIDTH, CELL_SIZE, AREA_SIZE);

    ros::ServiceServer addToMap_service = node.advertiseService("/Knowledge/addToMap", 
                                                    &Knowledge::addToMap, knowledge);

    ros::ServiceServer getPositions_service = node.advertiseService("/Knowledge/getPositions", 
                                                    &Knowledge::getPositions, knowledge);

    ros::ServiceServer getNewGoal_service = node.advertiseService("/Knowledge/getNewGoal", 
                                                    &Knowledge::getNewGoal, knowledge);

    ROS_INFO("Knowledge started.");

    ros::spin();

    delete knowledge;

    return 0;
}