#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <vector>



ros::Publisher pub;
nav_msgs::Path my_path;

void callback(const nav_msgs::Odometry::ConstPtr& data){
    my_path.header = data->header;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = my_path.header;
    pose_stamped.pose = data->pose.pose;
    my_path.poses.push_back(pose_stamped);
    pub.publish(my_path);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odomety_to_path_node");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::Path>("/ruido_imu_gps",1);

    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered",1,callback);

    ros::spin();
}