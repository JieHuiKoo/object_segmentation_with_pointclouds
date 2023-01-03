#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sstream>

ros::Publisher pub_nearestCloud;
ros::Publisher pub_nearestCloud_filled;
ros::Publisher pub_nearestCloudCenter;
double _max_distance = 0.01;

double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients)
{
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

struct rgb_values{
  int r, g, b;
};

rgb_values segmented_obj_colour;
rgb_values background_colour;

struct filter_limits{
  int max, min;
};

filter_limits z_axis_limits;
filter_limits y_axis_limits;

pcl::PointXYZ calc_cluster_center(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  pcl::PointXYZ centerPoint;
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  int sumPoints = 0;

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

void zero_out_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
  for (auto &point : pointcloud->points)
  {
    point.z = 0.0;
  }
}

float calc_dist_from_camera(pcl::PointXYZ point)
{
  return sqrt(pow(point.x,2) + pow(point.y,2) + pow(point.z,2));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert_pointcloud_pointXYZ_to_pointXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud_xyz, *cloud_xyzrgb);

  return cloud_xyzrgb;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr fill_pointcloud_with_colour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, rgb_values &fill_colour)
{
  for (auto &point : cloud_xyzrgb->points)
  {
    point.r = fill_colour.r;
    point.b = fill_colour.b;
    point.g = fill_colour.g;
  }

  return cloud_xyzrgb;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr fill_pointcloud_cluster_with_colour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_xyz, rgb_values &fill_colour)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
  
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_xyz);
  std::cout << "blah1" << "\n";
  for (auto &searchPoint : cluster_xyz->points)
  {
    int K = 3;
    std::vector<int> pointIdxKSearch(K); //to store index of surrounding points 
    std::vector<float> pointKSquaredDistance(K); // to store distance to surrounding points

    if ( kdtree.nearestKSearch(searchPoint, K, pointIdxKSearch, pointKSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < K; ++i)
        {
          cloud_xyzrgb->points[ pointIdxKSearch[i] ].r = fill_colour.r;
          cloud_xyzrgb->points[ pointIdxKSearch[i] ].g = fill_colour.g;
          cloud_xyzrgb->points[ pointIdxKSearch[i] ].b = fill_colour.b;
        }
    }
  }

  return cloud_xyzrgb;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, std::string axis, filter_limits axis_limits)
{
  // Create output cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);


  // Create passthrough object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(axis_limits.min, axis_limits.max);
  pass.setKeepOrganized(false);
  pass.filter (*output_cloud);

  return output_cloud;
}


void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Convert to pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg,*cloud_msg);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg,*cloud_rgb_msg);

  // Filter the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud = filter_cloud(cloud_msg, "z", z_axis_limits);
  filtered_cloud = filter_cloud(filtered_cloud, "y", y_axis_limits);
  
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
  int original_size(filtered_cloud->height*filtered_cloud->width);
  int n_planes(0);
  
  // Fit a plane
  seg.setInputCloud(filtered_cloud);
  seg.segment(*inliers, *coefficients);

  // Check result
  if (inliers->indices.size() == 0)
  {
    ROS_INFO("No Plane Found!");
  }

  // Extract All points that is not the plane
  extract.setInputCloud(filtered_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudF(new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter(*cloudF);

  // Creating the KdTree object for the search method of the euclidean extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloudF);


  // Set up Eucludean Cluster Extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_filtered = *cloudF;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.002); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract(cluster_indices);

  // Push the clusters into a vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices)
      cloud_cluster->points.push_back(cloud_filtered->points[idx]);

    cloud_cluster->width = cloud_cluster->points.size();   // For unorganized pointclouds, height is 1. Else, it is like a stack of images with height and width.
    cloud_cluster->height = 1; // In this case, we want the a organized pointcloud with cluster but rest of the points zero depth

    cloud_cluster->is_dense = false; //True if there are no invalid points
    clusters.push_back(cloud_cluster);
  }

  // Find center point of the clusters and return the idx of nearest cluster

  // Declare initial nearest point
  pcl::PointXYZ nearestCenter;
  int nearestCluster_idx = 0;
  nearestCenter.y = FLT_MAX;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_rgb;
  for (int i = 0; i < clusters.size(); ++i)
  {
    // Find center point of the clusters
    pcl::PointXYZ clusterCenter;
    clusterCenter = calc_cluster_center(clusters[i]);

    if (calc_dist_from_camera(clusterCenter) < calc_dist_from_camera(nearestCenter))
    {
      nearestCenter = clusterCenter;
      nearestCluster_idx = i;
    }
  }

  // Compare the points of cluster and original, and colour them in white
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filled_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_filled_msg = fill_pointcloud_cluster_with_colour(cloud_rgb_msg, clusters[nearestCluster_idx], segmented_obj_colour);

  // Publish points
  sensor_msgs::PointCloud2 cloud_publish;
  sensor_msgs::PointCloud2 cloud_filled_publish;
  geometry_msgs::Point nearestCenter_publish;

  // If we have valid clusters
  if (clusters.size() != 0)
  {
    std::cout << "Num of Clusters: " << clusters.size() << "\n";
    std::cout << "Cluster Num: " << nearestCluster_idx << "\n";

    pcl::toROSMsg(*clusters[nearestCluster_idx], cloud_publish);
    pcl::toROSMsg(*cloud_filled_msg, cloud_filled_publish);

    nearestCenter_publish.x = nearestCenter.x;
    nearestCenter_publish.y = nearestCenter.y;
    nearestCenter_publish.z = nearestCenter.z;
  }
  else
  {
    std::cout << "No Cluster Found!\n";
    nearestCenter_publish.x = 0;
    nearestCenter_publish.y = 0;
    nearestCenter_publish.z = 0;
  }

  cloud_publish.header = msg->header;
  cloud_filled_publish.header = msg->header;

  pub_nearestCloud.publish(cloud_publish);
  pub_nearestCloud_filled.publish(cloud_filled_publish);
  pub_nearestCloudCenter.publish(nearestCenter_publish);
}

int main(int argc, char **argv)
{
  segmented_obj_colour.r = 255;
  segmented_obj_colour.g = 255;
  segmented_obj_colour.b = 255;

  background_colour.r = 0;
  background_colour.g = 0;
  background_colour.b = 0;

  z_axis_limits.min = -10000;
  z_axis_limits.max = 10000;

  y_axis_limits.min = -10000;
  y_axis_limits.max = 10000;
  
  // Initialise ROS and specify name of node 
  ros::init(argc, argv, "segment");
  
  // Initialise the node handle
  ros::NodeHandle n;

  // Initialise the pub object
  // This pub object will advertise a PointCloud2 sensor_msgs with the topic and buffer of 1
  pub_nearestCloud = n.advertise<sensor_msgs::PointCloud2>("/armCamera/nearest_cloudCluster", 1);
  pub_nearestCloud_filled = n.advertise<sensor_msgs::PointCloud2>("/armCamera/nearest_cloudCluster_filled", 1);
  pub_nearestCloudCenter = n.advertise<geometry_msgs::Point>("/armCamera/nearest_cloudClusterCenter", 1);


  // Subscribe message
  ros::Subscriber sub = n.subscribe("/armCamera/depth_registered/points", 1, cloud_callback);

  ros::spin();
  }
