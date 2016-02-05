#include "resources.h"
#include "laser.h"
#include <vector>
#include <ros/ros.h>
#include <fstream>

#ifndef _MAPPING_H_
#define _MAPPING_H_

#define BASE_X (floor((this -> length/this -> cell_size) / 2))
#define BASE_Y (floor((this -> width/this -> cell_size) / 2))

class Mapper {
public:
	Mapper(ros::NodeHandle n, float length, float width, float cell_size);
	~Mapper();

	void handleLaser(const sensor_msgs::LaserScan::ConstPtr &data);

	std::vector<std::vector<CellValue>* >* getMap();

	void addToMap(_2DPoint point, CellValue value);





//		SOMAR DISPLACEMENT !!!!!!!!!!!!!!!!!!!!!!!!!





	void writeMap();

	void createMap();
	//void increaseCell(_2DPoint point);

private:
	float length;
	float width;
	float cell_size;
	Laser *laser;
	std::vector<std::vector<CellValue>* >* map;
	ros::NodeHandle node;
	ros::Subscriber laser_sub;
	//std::vector<std::vector<int>* >* occupancy_grid;
};


#endif