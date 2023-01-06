#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include <cmath>

#include <pcl/pcl_config.h>
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
ros::Publisher pub_nearestCloud2;

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

struct cluster_info{
  int idx = 0;
  pcl::PointXYZ centroid;
};

cluster_info nearestCluster;

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

float calc_dist_from_camera(pcl::PointXYZ point)
{
  return sqrt(pow(point.x,2) + pow(point.y,2) + pow(point.z,2));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr fill_pointcloud_cluster_with_colour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_xyz, rgb_values &fill_colour)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

  // 
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
          cloud_xyzrgb->points[pointIdxKSearch[i]].r = fill_colour.r;
          cloud_xyzrgb->points[pointIdxKSearch[i]].g = fill_colour.g;
          cloud_xyzrgb->points[pointIdxKSearch[i]].b = fill_colour.b;
        }
    }
  }

  return cloud_xyzrgb;
}

pcl::PCLPointCloud2::Ptr segmented_pointcloud2_clusters(pcl::PCLPointCloud2::Ptr cloud_2, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_xyz)
{
  pcl::PCLPointCloud2::Ptr clustered_cloud_2 (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(*cloud_2, *cloud_xyz);
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_xyz);
  std::cout << "blah2" << "\n";
  std::vector<int> clusterIndices;
  for (auto &searchPoint : cluster_xyz->points)
  {
    int K = 3;
    std::vector<int> pointIdxKSearch(K); //to store index of surrounding points 
    std::vector<float> pointKSquaredDistance(K); // to store distance to surrounding points

    if ( kdtree.nearestKSearch(searchPoint, K, pointIdxKSearch, pointKSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < K; ++i)
        {
          clusterIndices.push_back(pointIdxKSearch[i]);
          //clustered_cloud_2 -> data[pointIdxKSearch[i]] = cloud_2 -> data[pointIdxKSearch[i]]; 
          // pcl::copyPoint(cloud_2 -> data[pointIdxKSearch[i]], clustered_cloud_2 -> data[pointIdxKSearch[i]]);
        }
    }
  }
  pcl::PointIndices::Ptr ror_indices (new pcl::PointIndices);
  ror_indices -> indices = clusterIndices;
  pcl::copyPointCloud(*cloud_2, ror_indices -> indices, *clustered_cloud_2);
  return clustered_cloud_2;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_limit_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, std::string axis, filter_limits axis_limits)
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

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_plane_from_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  // Get segmentation ready
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold(0.005);
  
  // Fit a plane
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  // Check result
  if (inliers->indices.size() == 0)
  {
    ROS_INFO("No Plane Found!");
  }

  // Extract All points that is not the plane
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*input_cloud);

  return input_cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> find_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  // Creating the KdTree object for the search method of the euclidean extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  // Set up Eucludean Cluster Extraction
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.002); // in m
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract(cluster_indices);

  // Push each cluster into a vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices)
      cloud_cluster->points.push_back(input_cloud->points[idx]);

    cloud_cluster->width = cloud_cluster->points.size();   // For unorganized pointclouds, height is 1. Else, it is like a stack of images with height and width.
    cloud_cluster->height = 1;

    cloud_cluster->is_dense = false; //True if there are no invalid points
    clusters.push_back(cloud_cluster);
  }

  return clusters;
}

cluster_info find_nearest_cluster(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters)
{
  // Declare initial nearest point
  cluster_info nearestCluster;

  nearestCluster.centroid.y = FLT_MAX;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_rgb;
  for (int i = 0; i < clusters.size(); ++i)
  {
    // Find center point of the clusters
    pcl::PointXYZ clusterCenter;
    clusterCenter = calc_cluster_center(clusters[i]);

    if (calc_dist_from_camera(clusterCenter) < calc_dist_from_camera(nearestCluster.centroid))
    {
      nearestCluster.centroid = clusterCenter;
      nearestCluster.idx = i;
    }
  }

  return nearestCluster;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::console::print_info ("Available dimensions from msg: ");
  pcl::console::print_value ("%s\n", pcl::getFieldsList (*msg).c_str ());

  // std::string result;
  // for (size_t i = 0; i < *msg.fields.size () - 1; ++i) {
  //   result += *msg.fields[i].name + " ";
  // }
  // result += *msg.fields[msg.fields.size () - 1].name;
  
  // pcl::console::print_value ("%s\n", result.c_str ());
  
  // Convert to pcl point cloud, XYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg,*cloud_msg);

  // Convert to pcl point cloud, XYZRGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg,*cloud_rgb_msg);

  // Convert to pcl point cloud 2
  pcl::PCLPointCloud2::Ptr cloud_2_msg (new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*msg, *cloud_2_msg);
  pcl::console::print_info ("Available dimensions: "); 
  pcl::console::print_value ("%s\n", pcl::getFieldsList (*cloud_2_msg).c_str ());


  // Filter the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud = filter_limit_cloud(cloud_msg, "z", z_axis_limits);
  filtered_cloud = filter_limit_cloud(filtered_cloud, "y", y_axis_limits);
  
  // Remove the biggest plane
  filtered_cloud = filter_plane_from_cloud(filtered_cloud);

  // Find the clusters and put them in a vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  clusters = find_clusters(filtered_cloud);

  // Find center point of the clusters and return the idx of nearest cluster
  nearestCluster = find_nearest_cluster(clusters);

  // Compare the points of cluster and original, and colour them in white
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filled_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_filled_msg = fill_pointcloud_cluster_with_colour(cloud_rgb_msg, clusters[nearestCluster.idx], segmented_obj_colour);

  // Convert clusters to PointCloud2
  pcl::PCLPointCloud2::Ptr cloud_2_segmented_msg (new pcl::PCLPointCloud2);
  cloud_2_segmented_msg = segmented_pointcloud2_clusters(cloud_2_msg, clusters[nearestCluster.idx]);

  // Copy relevant parts of pointcloud
  cloud_2_segmented_msg -> header = cloud_2_msg -> header;
  cloud_2_segmented_msg -> height = cloud_2_msg -> height;
  cloud_2_segmented_msg -> width = cloud_2_msg -> width;
  cloud_2_segmented_msg -> fields = cloud_2_msg -> fields;
  cloud_2_segmented_msg -> is_bigendian = cloud_2_msg -> is_bigendian;
  cloud_2_segmented_msg -> point_step = cloud_2_msg -> point_step;
  cloud_2_segmented_msg -> row_step = cloud_2_msg -> row_step;
  cloud_2_segmented_msg -> is_dense = cloud_2_msg -> is_dense; 

  // Publish points
  sensor_msgs::PointCloud2 cloud_publish;
  sensor_msgs::PointCloud2 cloud_filled_publish;
  sensor_msgs::PointCloud2 cloud_2_segmented_publish;

  geometry_msgs::Point nearestCenter_publish;

  // If we have valid clusters
  if (clusters.size() != 0)
  {
    std::cout << "Num of Clusters: " << clusters.size() << "\n";
    std::cout << "Cluster Num: " << nearestCluster.idx << "\n";

    pcl::toROSMsg(*clusters[nearestCluster.idx], cloud_publish);
    pcl::toROSMsg(*cloud_filled_msg, cloud_filled_publish);
    pcl_conversions::moveFromPCL(*cloud_2_segmented_msg, cloud_2_segmented_publish);

    nearestCenter_publish.x = nearestCluster.centroid.x;
    nearestCenter_publish.y = nearestCluster.centroid.y;
    nearestCenter_publish.z = nearestCluster.centroid.z;
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
  cloud_2_segmented_publish.header = msg->header;



  pub_nearestCloud.publish(cloud_publish);
  pub_nearestCloud_filled.publish(cloud_filled_publish);
  pub_nearestCloud2.publish(cloud_2_segmented_publish);
  pub_nearestCloudCenter.publish(nearestCenter_publish);
}

int main(int argc, char **argv)
{
  std::cout << PCL_VERSION << std::endl;
  
  segmented_obj_colour.r = 255;
  segmented_obj_colour.g = 255;
  segmented_obj_colour.b = 255;

  background_colour.r = 0;
  background_colour.g = 0;
  background_colour.b = 0;

  z_axis_limits.min = -1000;
  z_axis_limits.max = 1000;

  y_axis_limits.min = -1000;
  y_axis_limits.max = 1000;
  
  // Initialise ROS and specify name of node 
  ros::init(argc, argv, "segment");
  
  // Initialise the node handle
  ros::NodeHandle n;

  // Initialise the pub object
  // This pub object will advertise a PointCloud2 sensor_msgs with the topic and buffer of 1
  pub_nearestCloud = n.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster", 1);
  pub_nearestCloud_filled = n.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudClusterFilled", 1);
  pub_nearestCloud2 = n.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloud2Cluster", 1);
  pub_nearestCloudCenter = n.advertise<geometry_msgs::Point>("/armCamera/nearestCloudClusterCentroid", 1);


  // Subscribe message
  ros::Subscriber sub = n.subscribe("/armCamera/depth_registered/points", 1, cloud_callback);

  ros::spin();
  }
