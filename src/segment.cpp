#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sstream>

ros::Publisher pub_nearestCloud;
ros::Publisher pub_nearestCloudCenter;
double _max_distance = 0.01;

double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients)
{
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

pcl::PointXYZ calc_cluster_center(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  pcl::PointXYZ centerPoint;
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  float sumPoints = 0;

  for (auto &point : cluster->points)
  {
    sumX = sumX + point.x;
    sumY = sumY + point.y;
    sumZ = sumZ + point.z;
    sumPoints++;
  }
  centerPoint.x = sumX/sumPoints;
  centerPoint.y = sumY/sumPoints;
  centerPoint.z = sumZ/sumPoints;

  return centerPoint;
}

float calc_dist_from_camera(pcl::PointXYZ point)
{
  return sqrt(pow(point.x,2) + pow(point.y,2) + pow(point.z,2));
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

  // Convert to pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg,*cloud_msg);

  // Filter cloud
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_msg);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(0.001,2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_zf (new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter (*cloud_zf);

  pass.setInputCloud(cloud_zf);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0,10);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter (*cloud);


  // Get segmentation ready
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold(_max_distance);

  // Create pointcloud to publish inliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
  int original_size(cloud->height*cloud->width);
  int n_planes(0);
  
  // Fit a plane
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Check result
  if (inliers->indices.size() == 0)
  {
    ROS_INFO("No Plane Found!");
  }

  // Extract All points that is not the plane
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudF(new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter(*cloudF);

  // Creating the KdTree object for the search method of the euclidean extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_filtered = *cloudF;
  tree->setInputCloud (cloud_filtered);

  // Set up Eucludean Cluster Extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.002); // 2cm
  ec.setMinClusterSize (200);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // Push the clusters into a vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices)
      cloud_cluster->points.push_back(cloud_filtered->points[idx]);
    cloud_cluster->width = cloud_cluster->points.size (); // For unorganized pointclouds, height is 1. Else,
    cloud_cluster->height = 1;                            // it is like a stack of images with height and width
    cloud_cluster->is_dense = true; //Specify if all the points are finite
    // std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
    clusters.push_back(cloud_cluster);
  }
  // std::cout << "End\n";

  // Find center point of the clusters and return the nearest cluster
  // Declare initial nearest point
  pcl::PointXYZ nearestCenter;
  int nearestCluster_idx = 0;
  nearestCenter.y = FLT_MAX;

  for (int i = 0; i < clusters.size(); ++i)
  {
    pcl::PointXYZ clusterCenter;
    clusterCenter = calc_cluster_center(clusters[i]);

    if (calc_dist_from_camera(clusterCenter) < calc_dist_from_camera(nearestCenter))
    {
      nearestCenter = clusterCenter;
      nearestCluster_idx = i;
    }
  }


  // Publish points
  sensor_msgs::PointCloud2 cloud_publish;

  if (clusters.size() != 0)
  {
    std::cout << "Cluster Array Size: " << clusters.size() << "\n";
    pcl::toROSMsg(*clusters[nearestCluster_idx], cloud_publish);
  }
  else
  {
    std::cout << "No Cluster Found!\n";
  }

 
  // pcl::toROSMsg(*cloudF,cloud_publish);
  cloud_publish.header = msg->header;

  pub_nearestCloud.publish(cloud_publish);
}

int main(int argc, char **argv)
{
  // Initialise ROS and specify name of node 
  ros::init(argc, argv, "segment");
  
  // Initialise the node handle
  ros::NodeHandle n;

  // Initialise the pub object
  // This pub object will advertise a PointCloud2 sensor_msgs with the topic /segmented_cloud and buffer of 1
  pub_nearestCloud = n.advertise<sensor_msgs::PointCloud2>("/armCamera/segmented_cloud", 1);
  pub_nearestCloudCenter = n.advertise<std_msgs::Float64MultiArray>("/armCamera/segmented_cloudCenter", 1);


  // Subscribe message
  ros::Subscriber sub = n.subscribe("/armCamera/depth_registered/points", 1, cloud_callback);

  ros::spin();
  }
