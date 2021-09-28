#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr my_cloud(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& data){
    pcl::fromROSMsg(*data, *velodyne_cloud);
    /*my_cloud->width = velodyne_cloud->width;
    my_cloud->height = velodyne_cloud->height;
    my_cloud->points.resize(my_cloud->width * my_cloud->height);
    */
    *my_cloud = *velodyne_cloud;

    for(int i = 0; i < velodyne_cloud->points.size(); i++){
        my_cloud->points[i].x = velodyne_cloud->points[i].y;
        my_cloud->points[i].y = velodyne_cloud->points[i].z;
        my_cloud->points[i].z = velodyne_cloud->points[i].x;
    }
    pub.publish(my_cloud);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "my_velodyne");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2>("/my_velodyne_points",1);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",1,callback);

    ros::spin();
}