/**
 * @file pcd_publisher.cpp
 * @brief Node for visualizing a pcd file using PointCloud.
 */

# include <iostream>
# include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

/**
 * @brief Main function of the node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 */
int main(int argc, char** argv)
{
  double x_cloud; double y_cloud; double z_cloud;

  // Initialize ROS publisher node 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ros::init (argc, argv, "pcd_publish_as_ros_msg");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("yourfile.pcd", *cloud);

  // Print PointCloud info upon start-up 
  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "frame1";
  msg->height = cloud->height;
  msg->width = cloud->width;

  // Get x, y, z data for each PointCloud element 
  for (size_t i = 0; i < cloud->size (); i++) {
  x_cloud = cloud->points[i].x;
  y_cloud = cloud->points[i].y;
  z_cloud = cloud->points[i].z;
  msg->points.push_back (pcl::PointXYZ (x_cloud, y_cloud, z_cloud));
  }

  ros::Rate loop_rate(4);

  // Keep spinning 
  while (nh.ok())
  {
    //msg->header.stamp = ros::Time::now().toNSec();
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
