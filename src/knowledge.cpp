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

bool Knowledge::checkError(int x, int y){
    int map_x = (int) BASE_X + TO_CELLS(x);
    int map_x_1 = (int) BASE_X + TO_CELLS(x + DISCRETE_ERROR);
    int map_x_2 = (int) BASE_X + TO_CELLS(x - DISCRETE_ERROR);
    int map_y = (int) BASE_Y + TO_CELLS(y);
    int map_y_1 = (int) BASE_Y + TO_CELLS(y + DISCRETE_ERROR);
    int map_y_2 = (int) BASE_Y + TO_CELLS(y - DISCRETE_ERROR);

   if(map[map_x_1][map_y] == FULL ||                 
      map[map_y_2][map_y] == FULL ||                 
      map[map_x][map_y_1] == FULL ||                 
      map[map_x][map_y_2] == FULL /*||                 
      map[map_x_1][map_y_1] == FULL ||
      map[map_x_1][map_y_2] == FULL ||
      map[map_y_2][map_y_1] == FULL ||
      map[map_y_2][map_y_2] == FULL*/
    )
    return true;

    return false;

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
    float dist = 0;
    int map_x = 1000, map_y = 1000;
    float range = 0;
    float range_inc = pow(pow(req.inc_x, 2) + pow(req.inc_y, 2), 0.5);

    map_x = ((int) TO_CELLS(aux_x)) + BASE_X;
    map_y = ((int) TO_CELLS(aux_y)) + BASE_Y;

    //while(COMPARE(req.start_x, aux.x, aux_x) && COMPARE(req.start_y, aux.y, aux_y)){
    while(map_x != x && map_y != y && MAX_RANGE > range){

        if(map[map_x][map_y] == UNKNOWN)
            map[map_x][map_y] = EMPTY;

        range += range_inc;

        aux_x += req.inc_x;
        aux_y += req.inc_y;
        
        map_x = ((int) TO_CELLS(aux_x)) + BASE_X;
        map_y = ((int) TO_CELLS(aux_y)) + BASE_Y;
    }

    if(req.range < MAX_RANGE && fmod(req.range, 0.5) > 0.15 /*&& map[map_x][map_y] == UNKNOWN*/)
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

    Knowledge *knowledge = new Knowledge(node, 40.0, 40.0, 0.25);

    ros::ServiceServer service = node.advertiseService("/Knowledge/addToMap", &Knowledge::addToMap, knowledge);

    ROS_INFO("Knowledge started.");

    ros::spin();

    return 0;
}