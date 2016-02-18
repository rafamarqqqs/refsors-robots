#include "../include/robot_control/localization.h"

Localizator::Localizator(ros::NodeHandle n, char *type){
    node = n;
    gazebo_pose_sub = n.subscribe("/gazebo/model_states", 1, &Localizator::handleGazeboModelState, this);
    robot_name = (char *) malloc(sizeof(char) * (strlen(MOBILE(type)) + 1));
    strcpy(robot_name, MOBILE(type));
}

Localizator::~Localizator(){
    free(robot_name);
}

void Localizator::handleGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr& data){
    int i = 0;
    std::string model_name(robot_name);

    while(data -> name[i].compare(model_name) != 0){        
        i++;
    }

    this -> pose = data -> pose[i];

    tf::Quaternion q(data -> pose[i].orientation.x,
                    data -> pose[i].orientation.y,
                    data -> pose[i].orientation.z,
                    data -> pose[i].orientation.w);

    q = q.normalized();

    tf::quaternionTFToMsg(q, this -> pose.orientation);
}

geometry_msgs::Pose Localizator::getPose(){
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

void Localizator::publishPose(){
    ros::Publisher pose_pub = node.advertise<geometry_msgs::Pose>(POSE(argv[1]), 1000);
    geometry_msgs::Pose pose;
    ros::Rate r(10.0);

    while(node.ok()){

        ros::spinOnce();               // check for incoming messages

        //next, we'll publish the odometry message over ROS
        //geometry_msgs::Pose aux;

        //set the position
        pose = localizator -> getPose();

        //publish the message
        pose_pub.publish(pose);

        r.sleep();
    }
}

int main(int argc, char **argv){
    if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    ros::init(argc, argv, "pose_publisher");

    ros::NodeHandle n;
    //tf::TransformListener listener;

    //while(!listener.frameExists("base_link") ||
    //    !listener.frameExists("odom"));


    Localizator *localizator = new Localizator(n, argv[1]);

    localizator -> publishPose();

    return 0;
}