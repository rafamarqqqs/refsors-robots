#include "../include/robot_control/localization.h"

Localizator::Localizator(ros::NodeHandle n, char *type): node(n), robot_name(MOBILE(type)), position_ready(false) {
    gazebo_pose_sub = n.subscribe("/gazebo/model_states", 1, &Localizator::handleGazeboModelState, this);
}

Localizator::~Localizator(){}

void Localizator::handleGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr& data){

    if(!position_ready){
        int i = 0;
        
        while(data -> name[i].compare(robot_name) != 0){        
            i++;
        }

        id = i;
        
        position_ready = true;
    }

    this -> pose.pose = data -> pose[id];

    tf::Quaternion q(data -> pose[id].orientation.x,
                    data -> pose[id].orientation.y,
                    data -> pose[id].orientation.z,
                    data -> pose[id].orientation.w);

    q = q.normalized();

    this -> pose.header.stamp = ros::Time::now();

    tf::quaternionTFToMsg(q, this -> pose.pose.orientation);

}

geometry_msgs::PoseStamped Localizator::getPose(){
    /*odom_listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1));
    odom_listener.lookupTransform("/odom", "/base_link", ros::Time(0), odom_transform);

    this -> pose.position.x = odom_transform.getOrigin().x();
    this -> pose.position.y = odom_transform.getOrigin().y();
    
    tf::Quaternion q(odom_transform.getRotation().x(),
                    odom_transform.getRotation().y(),
                    odom_transform.getRotation().z(),
                    odom_transform.getRotation().w());

    q = q.normalized();

    ROS_INFO("%3.2f %3.2f %3.2f", pose.position.x, pose.position.y, tf::getYaw(q));

    tf::quaternionTFToMsg(q, this -> pose.orientation);
    */


    return this -> pose;
}

void Localizator::publishPose(char* type){
    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>(POSE(type), 1000);
    ros::Rate r(20.0);

    // waits until the messages with the positions start arriving
    while(!position_ready && ros::ok()){
        ros::spinOnce();
    }

    while(node.ok()){
        ros::spinOnce();
        pose_pub.publish(getPose());
        r.sleep();
    }
}

int main(int argc, char **argv){
    if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    ros::init(argc, argv, "Localization");

    ros::NodeHandle n;

    Localizator *localizator = new Localizator(n, argv[1]);

    ROS_INFO("Localization started.");

    localizator -> publishPose(argv[1]);

    delete localizator;

    return 0;
}