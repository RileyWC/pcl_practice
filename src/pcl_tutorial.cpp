#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

//global variables
ros::Publisher pub;
std::string filter_field_name;
float filter_min_limit, filter_max_limit, distance_threshold, point_color_threshold, region_color_threshold;
int min_cluster_size;

void subReact(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // make two pcl variables
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // populate raw_msg with the pcl data in cloud_msg that we subscribed to
  pcl::fromROSMsg(*cloud_msg, *raw_cloud);

  /*
  // debugging input cloud (PointXYZ)
  int r_width = raw_cloud->width;
  ROS_INFO("width: %d", r_width);
  */

  // filter
  /*
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(raw_cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*transformed_cloud);
  */

  //segment
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (raw_cloud);
  pass.setFilterFieldName (filter_field_name);
  pass.setFilterLimits (filter_min_limit, filter_max_limit);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (raw_cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (distance_threshold);
  reg.setPointColorThreshold (point_color_threshold);
  reg.setRegionColorThreshold (region_color_threshold);
  reg.setMinClusterSize (min_cluster_size);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  /*
  // debugging transformed cloud (PointXYZ)
  int t_width = transformed_cloud->width;
  ROS_INFO("width: %d", t_width);
  */

  // make an output cloud and populate it with the transformed cloud
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*colored_cloud, output_msg);
  output_msg.header.frame_id = "camera_depth_optical_frame";

  // publish output
  pub.publish(output_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_pub_node");
  ros::NodeHandle nh;
  // ros::NodeHandle nh("~");

  ros::Rate rate(1.0);
  rate.sleep();

  ROS_INFO("sanity check");

  std::string pcl_input_topic, output_topic;
  nh.param<std::string>("pcl_input_topic", pcl_input_topic,
                        "camera/depth/points");
  nh.param<std::string>("output_topic", output_topic, "/pcl_output");

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "camera/depth/points", 1, subReact);
  pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
  

  ros::Rate loop_rate(10.0);
  while(ros::ok()){
    //put params in here
    nh.param<std::string>("filter_field_name", filter_field_name,"z");
    nh.param<float>("filter_min_limit", filter_min_limit, 0.0);  
    nh.param<float>("filter_max_limit", filter_max_limit, 1.0);
    nh.param<float>("distance_threshold", distance_threshold, 10);
    nh.param<float>("point_color_threshold", point_color_threshold, 6);
    nh.param<float>("region_color_threshold", region_color_threshold, 5);
    nh.param<int>("min_cluster_size", min_cluster_size, 600);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}