#include "../include/robot_control/localization.h"

Localizator::Localizator(ros::NodeHandle n, char *type): node(n), robot_name(MODEL_NAME(type)), position_ready(false),
    laser_displacement(LASER_DISPLACEMENT(type)) {
    gazebo_pose_sub = n.subscribe(MODEL_STATES_TOPIC, 1, &Localizator::handleGazeboModelState, this);
}

Localizator::~Localizator(){}

inline void Localizator::handleGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr& data){

    if(data == NULL)
        return;

    if(!position_ready){
        int i = 0;
        
        while(data -> name[i].compare(robot_name) != 0 && ros::ok()){        
            i++;
            
            if(i == data -> name.size())
                return;
        }

        id = i;
    }

    this -> pose.pose = data -> pose[id];

    tf::Quaternion q(data -> pose[id].orientation.x,
                    data -> pose[id].orientation.y,
                    data -> pose[id].orientation.z,
                    data -> pose[id].orientation.w);

    q = q.normalized();

    double yaw = tf::getYaw(q);

    this -> pose.pose.position.x += laser_displacement * cos(yaw);
    this -> pose.pose.position.y += laser_displacement * sin(yaw);

    this -> pose.header.stamp = ros::Time::now();

    tf::quaternionTFToMsg(q, this -> pose.pose.orientation);

}

inline geometry_msgs::PoseStamped Localizator::getPose(){
    return this -> pose;
}

inline void Localizator::publishPose(char* type){
    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC(type), 1);
    ros::Rate r(20.0);

    while(node.ok() && ros::ok()){
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

    ros::init(argc, argv, LOCALIZATION_NODE);

    ros::NodeHandle n;

    Localizator *localizator = new Localizator(n, argv[1]);

    ROS_INFO("Localization started.");

    localizator -> publishPose(argv[1]);

    delete localizator;

    return 0;
}