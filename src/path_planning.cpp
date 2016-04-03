#include "../include/robot_control/path_planning.h"

PathPlanning::PathPlanning(ros::NodeHandle n, double len, double wid):
     node(n), length(len), width(wid) {}

PathPlanning::~PathPlanning(){

}

inline Node* PathPlanning::initNode(int x, int y, int g, Node* parent){
    Node* node = (Node*) malloc(sizeof(Node));
    node -> x = x;
    node -> y = y;
    node -> H = (abs(x - end_x) + (abs(y - end_y))) * CTE;
    node -> G = g;
    node -> F = node -> G + node -> H;
    node -> parent = parent;

    return node;
}

inline vector<Node*>::iterator PathPlanning::find(vector<Node*>* vec, int x, int y){
    vector<Node*>::iterator it;

    for(it = vec -> begin(); it != vec -> end(); it++){
        if((*it) -> x == x && (*it) -> y == y)
            break;
    }

    return it;
}

void PathPlanning::defineLocalPath(vector<int>* x_path, vector<int>* y_path){
    vector<int>::iterator x = x_path -> begin();
    vector<int>::iterator y = y_path -> begin();

    // the destiny node is never erased
    x++; y++;

    int x_prop, y_prop;
    int x_last = (*x), y_last = (*y);

    // first propotion (second to the third node)
    x++; y++;
    x_prop = x_last - (*x);
    y_prop = y_last - (*y);

    // cicle trough the path erasing nodes "in the same line"
    // x_path.size() == y_path.size() always
    while(x != x_path -> end() && ros::ok()){

        // if the proportion changes, don't erase the last node before the one that made the proportion change
        // and change the proportion to the node that changed it
        if(x_prop != x_last - (*x) || y_prop != y_last - (*y)){
            x_prop = x_last - (*x);
            y_prop = y_last - (*y);

            x_last = (*x);
            y_last = (*y);
                    
            x++; y++;
                
        }

        // we don't need the starting node because it's our current position, so we erase it and end
        if(x >= x_path -> end()){
            if(x == x_path -> end()){
                x--; y--;
            }
            x_path -> erase(x);
            y_path -> erase(y);    

            break;
        }
        x_last = (*x);
        y_last = (*y);
        
        // "erasing the node before"
        x_path -> erase(x - 1);
        y_path -> erase(y - 1);
    }
}

bool PathPlanning::defineGlobalPath(robot_control::defineGlobalPath::Request& req,
                      robot_control::defineGlobalPath::Response& res){

    ROS_INFO("calculating path from %d %d to %d %d", (int)TO_UNKNOWN_CELLS(req.x, req.cell_size)  + UNKNOWN_CELL_BASE_X(req.cell_size), 
                                                    (int)TO_UNKNOWN_CELLS(req.y, req.cell_size)  + UNKNOWN_CELL_BASE_Y(req.cell_size),
                                                    (int)TO_UNKNOWN_CELLS(req.destiny_x, req.cell_size)  + UNKNOWN_CELL_BASE_X(req.cell_size), 
                                                    (int)TO_UNKNOWN_CELLS(req.destiny_y, req.cell_size) + UNKNOWN_CELL_BASE_Y(req.cell_size));

    vector<unsigned char> map = req.map;
    
    vector<Node*>* open_nodes = new vector<Node*>();
    vector<Node*>* closed_nodes = new vector<Node*>();

    vector<int> x_path, y_path;
    
    vector<Node*>::iterator it;

    Node* p_node = NULL;
    Node* current_node = NULL;
    Node* node = NULL;
    Node* end_node = NULL;
    int i, j;
    int F, G, H;

    // end position
    end_x = TO_UNKNOWN_CELLS(req.destiny_x, req.cell_size) + UNKNOWN_CELL_BASE_X(req.cell_size);
    end_y = TO_UNKNOWN_CELLS(req.destiny_y, req.cell_size) + UNKNOWN_CELL_BASE_Y(req.cell_size);
    
    // creating start and end nodes
    node = initNode(TO_UNKNOWN_CELLS(req.x, req.cell_size) + UNKNOWN_CELL_BASE_X(req.cell_size), 
                    TO_UNKNOWN_CELLS(req.y, req.cell_size) + UNKNOWN_CELL_BASE_Y(req.cell_size), 0, NULL);

    end_node = initNode(end_x, end_y, 0, NULL);

    // add start node to the open list
    open_nodes -> push_back(node);

    // while the end node isn't on the closed list
    while(find(closed_nodes, end_x, end_y) == closed_nodes -> end() && ros::ok()){
        
        // sort the array
        sort(open_nodes -> begin(), open_nodes -> end(), cmpNodes);
        
        // put the node with the minimum F value on the closed list and remove it from the open list
        closed_nodes -> push_back(open_nodes -> front());
        current_node = open_nodes -> front();
        open_nodes -> erase(open_nodes -> begin());
        
        // for all neighbours of the current node
        for(i = -1; i <= 1; i++){
            for(j = -1; j <= 1; j++){

                // if it isn't himself, the path is clear and it's not on the closed list
                if(IS_UNKNOWN_CELL_INSIDE(current_node -> x + i, current_node -> y + j, req.cell_size) && !(i == 0 && j == 0)
                   && map.at(((current_node -> x + i) * TO_UNKNOWN_CELLS(length, req.cell_size)) + current_node -> y + j) < EMPTY_RANGE
                   && find(closed_nodes, current_node -> x + i, current_node -> y + j) == closed_nodes -> end()){
                    
                    it = find(open_nodes, current_node -> x + i, current_node -> y + j);

                    // if the node isn't on the open list
                    if(it == open_nodes -> end()){

                        // calculate G, H, F
                        node = initNode(current_node -> x + i, current_node -> y + j,
                              ((i == 0 || j == 0) ? ORTOGONAL_WEIGTH : DIAGONAL_WEIGTH) + current_node -> G,
                              current_node);
                        
                        // add it to the open list
                        open_nodes -> push_back(node);
                    }
                    // else if the node is already on the open list
                    else{
                        node = (*it);

                        // path from the current node to the node
                        int aux_G = ((i == 0 || j == 0) ? ORTOGONAL_WEIGTH : DIAGONAL_WEIGTH) + current_node -> G;

                        // if this path to the node is better than the old one, change it's settings
                        if(node -> G > aux_G){
                            node -> parent = current_node;
                            node -> G = aux_G;
                            node -> H = (abs(current_node -> x + i - end_x) + (abs(current_node -> y + j - end_y))) * CTE;
                            node -> F = node -> G + node -> H;
                        }
                    }
                }
            }
        }
    }

    // get pointer to the end node (starting position)
    node = *(find(closed_nodes, end_x, end_y));

    // go backwards until the starting node (destination position)
    while(node != NULL && ros::ok()){
        // create and x y path
        x_path.push_back(node -> x);
        y_path.push_back(node -> y);
        // adding to the map for visualizing purposes
        map.at((node -> x * (TO_UNKNOWN_CELLS(length, req.cell_size))) + node -> y) = 'x';
        node = node -> parent;
    }

    // erase unecessary nodes on the path
    defineLocalPath(&x_path, &y_path);

    vector<int>::iterator x, y;

    vector<int>::reverse_iterator _y;
    vector<int>::reverse_iterator _x;

    // adding to the map for visualizing purposes
    for(_x = x_path.rbegin(), _y = y_path.rbegin();
        _x != x_path.rend(); _x++, _y++){

        res.x_path.push_back(((*_x)*req.cell_size) - (MAP_LENGTH/2));
        res.y_path.push_back(((*_y)*req.cell_size) - (MAP_WIDTH/2));
    }

    for(x = x_path.begin(), y = y_path.begin(); x != x_path.end(); x++, y++)
        map.at(((*x) * (TO_UNKNOWN_CELLS(length, req.cell_size))) + (*y)) = 'X';

    for(it = open_nodes -> begin(); it != open_nodes -> end(); it++)
        free((*it));
    for(it = closed_nodes -> begin(); it != closed_nodes -> end(); it++)
        free((*it));

    writeMap(map, req.cell_size);

    delete open_nodes;
    delete closed_nodes;

    return true;
}

void PathPlanning::writeMap(vector<unsigned char> map, float req_cell_size){
    int i = 1;
    char block;
    vector<unsigned char>::iterator it;

    std::ofstream file(PATH_PLANNING_MAP);

    for(it = map.begin(); it != map.end(); it++){
            if((*it) == 'X')
                block = 'X';
            else if((*it) == 'x')
                block = 'x';
            else if((*it) < 30)
                block = ' ';
            else if((*it) > 70)
                block = '#';
            else
                block = ':';

            file.put(block);

        if(i % ((int)TO_UNKNOWN_CELLS(length, req_cell_size)) == 0){
            file.put('\n');
            i = 0;
        }

        i++;
    }

    file.close();
}

int main(int argc, char **argv) {

    //Initializes ROS, and sets up a node
    ros::init(argc, argv, PATH_PLANNING_NODE);

    ros::NodeHandle node;

    PathPlanning* path_planner = new PathPlanning(node, MAP_LENGTH, MAP_WIDTH);

    ros::ServiceServer service = node.advertiseService(DEFINE_GLOBAL_PATH_SERVICE, 
                                                    &PathPlanning::defineGlobalPath, path_planner);

    ROS_INFO("Path planning started.");

    ros::spin();

    delete path_planner;

    return 0;
}