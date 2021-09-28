#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <vector>



ros::Publisher pub;
nav_msgs::Path my_path;

void callback(const geometry_msgs::PoseStamped::ConstPtr& data){
    my_path.header = data->header;
    my_path.poses.push_back(*data);
    pub.publish(my_path);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "pose_to_path_node");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::Path>("/ground_truth_path",1);

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/ground_truth_to_tf/pose",1,callback);

    ros::spin();
}