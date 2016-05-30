#include "../include/robot_control/knowledge.h"

using namespace std;

Knowledge::Knowledge(ros::NodeHandle n, double len, double wid, double cell_s, double area_s):
    node(n), length(len), width(wid), cell_size(cell_s), area_size(area_s), ids(0), mapped(false), goals(NULL){

    this -> positions = new vector<_2DPoint>();
    this -> goals = new vector<Goal>();

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
    
    free(map);
    free(map_scans);

    delete positions;
    delete goals;

    for(i = 0; i < floor(length/area_size); i++)
        free(areas[i]);

    free(areas);
}

void Knowledge::initMaps(){
    int i, j;

    map = (char **) malloc(sizeof(char *) * TO_CELLS(length));
    map_scans = (int **) malloc(sizeof(int *) * TO_CELLS(length));

    for(i = 0; i < TO_CELLS(length); i++){
        map[i] = (char *) malloc(sizeof(char) * TO_CELLS(width));
        map_scans[i] = (int *) calloc(TO_CELLS(width), sizeof(int));
    }
    
    for(i = 0; i < TO_CELLS(length); i++){
        for(j = 0; j < TO_CELLS(width); j++){
            map[i][j] = MAP_UNKNOWN;
        }
    }
}

void Knowledge::initAreas(){
    int i, j;
    areas = (int **) malloc(sizeof(int *) * floor(length/area_size));

    for(i = 0; i < floor(length/area_size); i++){
        areas[i] = (int *) malloc(sizeof(int) * floor(width/area_size));
        
        for(j = 0; j < floor(width/area_size); j++)
            areas[i][j] = EMPTY;
    }
}

inline bool Knowledge::isCurrentGoal(int x, int y){
    vector<Goal>::iterator it;
    
    for(it = goals -> begin(); it != goals -> end(); it++){
        if((*it).x == x && (*it).y == y){
      //      ROS_INFO("same goal");
            return true;
        }
    }

    return false;
}

inline bool Knowledge::isInTheSameQuadrant(int x, int y){
    int middle_x = ((int) floor(length/area_size))/2;
    int middle_y = ((int) floor(width/area_size))/2;

    
    vector<Goal>::iterator it;
    
    for(it = goals -> begin(); it != goals -> end(); it++){
        ROS_INFO("quads (%d, %d) (%d, %d) middle: %d %d", x, y, (*it).x, (*it).y, middle_x, middle_y);
        
        if(x <= middle_x && y <= middle_y && (*it).x <= middle_x && (*it).y <= middle_y){
            return true;
        }
        if(x <= middle_x && y >= middle_y && (*it).x <= middle_x && (*it).y >= middle_y){
            return true;
        }
        if(x >= middle_x && y >= middle_y && (*it).x >= middle_x && (*it).y >= middle_y){
            return true;
        }
        if(x >= middle_x && y <= middle_y && (*it).x >= middle_x && (*it).y <= middle_y){
            return true;
        }
    }
    ROS_INFO("Goal not in the same quadrant as other current goals");
    return false;
}

inline void Knowledge::getLeastExploredQuadrant(int* x_displacement, int* y_displacement, int length, int width){
    int selected = 1;
    int quadrant = 1;
    int max = 0;
    int counter = 0;

    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 2; j++){
            for(int _i = 0; _i < TO_CELLS(length)/2; _i++){
                for(int _j = 0; _j < TO_CELLS(width)/2; _j++){
                    if(map[_i + (i*(TO_CELLS(length)/2))][_j + (j*(TO_CELLS(width)/2))] > EMPTY_RANGE)
                        counter++;
                }
            }

            if(counter > max){
                selected = quadrant;
                max = counter;
            }

            counter = 0;
            quadrant++;
        }
    }

    if(selected == 1){
        *x_displacement = 0;
        *y_displacement = 0;
    }
    else if(selected == 2){
        *x_displacement = length;
        *y_displacement = 0;
    }
    else if(selected == 3){
        *x_displacement = 0;
        *y_displacement = width;
    }
    else if(selected == 4){
        *x_displacement = length;
        *y_displacement = width;
    }
}

bool Knowledge::getNewGoal(robot_control::getNewGoal::Request& req, robot_control::getNewGoal::Response& res){
    goal_mtx.lock();

    vector<uint8_t> occupied_areas = req.occupied_areas;
    vector<uint8_t>::iterator it;
    vector<Goal>::iterator it_g;
    Goal goal;
    int last_goal_x = req.x, last_goal_y = req.y;
    int i = 0, j = 0;
    int n_l_areas = (int) floor(length/area_size);
    int n_w_areas = (int) floor(width/area_size);
    int tries = 0;
    int x_displacement, y_displacement;

    it = occupied_areas.begin();

    for(i = 0; i < n_l_areas; i++){
        for(j = 0; j < n_w_areas; j++){
            areas[i][j] = areas[i][j] == OCCUPIED ? OCCUPIED : (*it);
            it++;
        }
    }

    getLeastExploredQuadrant(&x_displacement, &y_displacement, n_l_areas/2, n_w_areas/2);

    srand(time(NULL));
    res.new_x = (rand() % (n_l_areas/2)) + x_displacement;
    res.new_y = (rand() % (n_w_areas/2)) + y_displacement;

    while((areas[res.new_x][res.new_y] == OCCUPIED || isCurrentGoal(res.new_x, res.new_y) 
        || isInTheSameQuadrant(res.new_x, res.new_y)) && tries < n_l_areas*n_w_areas && ros::ok()){

        res.new_x = (rand() % (n_l_areas/2)) + x_displacement;
        res.new_y = (rand() % (n_w_areas/2)) + y_displacement;
        
        tries++;
    }

    ROS_INFO("%d %d -> %d %d", req.x, req.y, res.new_x, res.new_y);

    if(tries == n_w_areas*n_l_areas){
        res.new_x = rand() % n_l_areas;
        res.new_y = rand() % n_w_areas;
        ROS_INFO("max tries");
    }

    ROS_INFO("Current goals: %d", (int) goals -> size());
    for(it_g = goals -> begin(); it_g != goals -> end(); it_g++){
        //ROS_INFO("* %d %d", (*it_g).x, (*it_g).y);
        if((*it_g).x == last_goal_x && (*it_g).y == last_goal_y){
            (*it_g).x = res.new_x;
            (*it_g).y = res.new_y;
            break;
        }
    }

    if(it_g == goals -> end()){
        goal.x = res.new_x;
        goal.y = res.new_y;
        goals -> push_back(goal);
    }

    writeAreas();

    goal_mtx.unlock();

    return true;
}

inline void Knowledge::writeAreas(){
    int counter = 0;
    std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/areas.map");

    for(int i = floor(length/area_size) - 1; i >= 0 ; i--){
        for(int j = 0; j < floor(width/area_size) ; j++){

            file << "|";
            file << std::setfill(' ') << (areas[i][j] == OCCUPIED ? '1' : '0');
        }

        file << "|";
        file << "\n";
    }

    file.close();
}

bool Knowledge::getPositions(robot_control::getPositions::Request& req, robot_control::getPositions::Response& res){
    pose_mtx.lock();
    
    _2DPoint aux;
    vector<_2DPoint>::iterator it;
    int i;

    aux.x = req.my_x;
    aux.y = req.my_y;

    map[TO_CELLS(aux.x) + BASE_X][TO_CELLS(aux.y) + BASE_Y] = WALKABLE;

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
    
    pose_mtx.unlock();

    return true;
}

bool Knowledge::getMap(robot_control::getMap::Request& req, robot_control::getMap::Response& res){
    int cell_diff = (int)(req.cell_size/cell_size);
    int occupied;
    double rate;
    double total = SQUARE(cell_diff);

    string aux = to_string(req.cell_size);

    std::ofstream file("/home/rafael/SORS_Application/src/robot_control/maps/new" + aux + ".map");

    for(int i = 0; i < TO_CELLS(length); i+=cell_diff){
        for(int j = 0; j < TO_CELLS(width); j+=cell_diff){
            occupied = 0;
    
            for(int _i = 0; _i < cell_diff; _i++){
                for(int _j = 0; _j < cell_diff; _j++){
                    if(map[i + _i][j + _j] > EMPTY_RANGE)
                        occupied++;
                }
            }

            rate = occupied/total;
            
            res.map.push_back(occupied < 0.4 ? ' ' : '#');
            file.put(occupied < 0.4 ? ' ' : '#');
        }
        file.put('\n');
    }

    file.close();

    return true;
}

bool Knowledge::addToMap(robot_control::addToMap::Request& req, robot_control::addToMap::Response& res){

    if(!IS_POINT_INSIDE(req.wall_x, req.wall_y))
        return false;

    map_mtx.lock();

    int x = BASE_X + TO_CELLS(req.wall_x);
    int y = BASE_Y + TO_CELLS(req.wall_y);
    double aux_x = req.start_x + req.inc_x;
    double aux_y = req.start_y + req.inc_y;
    double aux_average;
    int map_x = TO_CELLS(aux_x) + BASE_X;
    int map_y = TO_CELLS(aux_y) + BASE_Y;
    int last_map_x = INT_MAX, last_map_y = INT_MAX;

    while((map_x != x && map_y != y) && IS_CELL_INSIDE(map_x, map_y) && ros::ok()){

        if((last_map_x != map_x || last_map_y != map_y) && map_scans[map_x][map_y] < MAX_SCANS ? true : map[map_x][map_y] < FULL_RANGE
            && map[map_x][map_y] != WALKABLE){
            map_scans[map_x][map_y]++;
            map[map_x][map_y] = floor((MAP_EMPTY + map[map_x][map_y])/map_scans[map_x][map_y]);
            last_map_x = map_x;
            last_map_y = map_y;
        }

        aux_x += req.inc_x;
        aux_y += req.inc_y;
        
        map_x = TO_CELLS(aux_x) + BASE_X;
        map_y = TO_CELLS(aux_y) + BASE_Y;
    }

    if(IS_CELL_INSIDE(x, y) && map_scans[x][y] < MAX_SCANS ? true : map[x][y] > EMPTY_RANGE
       && map[map_x][map_y] != WALKABLE){

        aux_average = map[x][y] * (map_scans[x][y]);
        map_scans[x][y]++;
        map[x][y] = floor(aux_average + MAP_FULL)/(map_scans[x][y]);
    }

    res.added = true;

    map_mtx.unlock();

    return true;
}

void Knowledge::writeMap(){
    int i, j;
    char block;

    std::ofstream file(CONTAINERS_MAP);

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
    ros::init(argc, argv, KNOWLEDGE_NODE);

    ros::NodeHandle node;

    Knowledge* knowledge = new Knowledge(node, MAP_LENGTH, MAP_WIDTH, CELL_SIZE, AREA_SIZE);

    ros::ServiceServer getMap_service = node.advertiseService(GET_MAP_SERVICE, &Knowledge::getMap, knowledge);

    ros::ServiceServer addToMap_service = node.advertiseService(ADD_TO_MAP_SERVICE, &Knowledge::addToMap, knowledge);

    ros::ServiceServer getPositions_service = node.advertiseService(GET_POSITIONS_SERVICE, &Knowledge::getPositions, knowledge);

    ros::ServiceServer getNewGoal_service = node.advertiseService(GET_NEW_GOAL_SERVICE, &Knowledge::getNewGoal, knowledge);

    ROS_INFO("Knowledge started.");

    ros::spin();

    delete knowledge;

    return 0;
}