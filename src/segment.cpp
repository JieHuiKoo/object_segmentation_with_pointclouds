#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
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

ros::Subscriber sub;
ros::Publisher pub_nearestCloud;
ros::Publisher pub_nearestCloud_filled;
ros::Publisher pub_nearestCloudCenter;
ros::Publisher pub_nearestCloud2; 

class Robot
{
public:

  ros::Subscriber sub;
  ros::Publisher pub_nearestCloud;
  ros::Publisher pub_nearestCloud_filled;
  ros::Publisher pub_nearestCloudCenter;
  ros::Publisher pub_nearestCloud2; 
  ros::Rate rate{1};

  Robot(ros::NodeHandle *nh, ros::Rate defined_rate){
      rate = defined_rate;
      sub = nh->subscribe("/armCamera/depth_registered/points",1000,&Robot::cloud_callback,this);
      pub_nearestCloud = nh->advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster", 1);
      pub_nearestCloud_filled = nh->advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster_FilledPointCloud", 1);
      pub_nearestCloudCenter = nh->advertise<geometry_msgs::Point>("/armCamera/nearestCloudCluster_Centroid", 1);
      pub_nearestCloud2 = nh->advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloud2Cluster", 1);
  }
  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
}; 

double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients)
{
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

struct plane_info{
  pcl::ModelCoefficients::Ptr plane_coefficients;
  pcl::PointIndices::Ptr inlier_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud;
};

struct rgb_values{
  int r, g, b;
};

rgb_values segmented_obj_colour;
rgb_values background_colour;

struct filter_limits{
  float max, min;
};

filter_limits z_axis_limits;
filter_limits y_axis_limits;
filter_limits x_axis_limits;

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

  pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_xyz);
  
  for (auto &searchPoint : cluster_xyz->points)
  {
    int K = 10;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_plane_from_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointIndices::Ptr plane_indices)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Extract All points that is the plane
  extract.setInputCloud(input_cloud);
  extract.setIndices(plane_indices);
  extract.setNegative(true);
  extract.filter(*input_cloud);
  
  return input_cloud;
}

void get_biggest_plane(plane_info* biggest_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  // Get segmentation ready
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold(0.01);

  // Fit a plane
  seg.setInputCloud(input_cloud);
  seg.segment(*(biggest_plane->inlier_indices), *(biggest_plane->plane_coefficients));

  // Check result
  if ((biggest_plane->inlier_indices)->indices.size() == 0)
  {
    ROS_INFO("No Plane Found!");
  }
  std::cout << "2" << std::endl;

}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> find_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  // Creating the KdTree object for the search method of the euclidean extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  // Set up Eucludean Cluster Extraction
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); // in m
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (10000);
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

bool is_cluster_on_plane(pcl::PointXYZ cluster_centroid, pcl::ModelCoefficients::Ptr plane_coefficients)
{
  // We find if the cluster_centroid is roughly perpendicular (+- 10 degrees) to the plane
  // TODO
  return true;
}

cluster_info find_nearest_cluster(pcl::ModelCoefficients::Ptr plane_coefficients, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters)
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

    if (is_cluster_on_plane(clusterCenter, plane_coefficients))
    {
      float object_dist_from_camera = calc_dist_from_camera(clusterCenter);

      if (object_dist_from_camera > 0.2)
      {
        if (calc_dist_from_camera(clusterCenter) < calc_dist_from_camera(nearestCluster.centroid))
        {
          nearestCluster.centroid = clusterCenter;
          nearestCluster.idx = i;
        }
      }
    }
  }
  return nearestCluster;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // Publish points
  sensor_msgs::PointCloud2 cloud_publish;
  sensor_msgs::PointCloud2 cloud_filled_publish;
  sensor_msgs::PointCloud2 cloud_2_segmented_publish;
  geometry_msgs::PoseStamped nearestCenter_publish;

  // Downsample Cloud
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPTR(cloud);
  pcl::PCLPointCloud2 cloud_downsampled;
  
  pcl_conversions::toPCL(*msg, *cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPTR);
  sor.setLeafSize (0.005, 0.005, 0.005);
  sor.filter(cloud_downsampled);
  
  // Convert to pcl point cloud, XYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_downsampled, *cloud_msg);

  // Convert to pcl point cloud, XYZRGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg,*cloud_rgb_msg);

  // Convert to pcl point cloud 2
  pcl::PCLPointCloud2::Ptr cloud_2_msg (new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*msg, *cloud_2_msg);

  // Filter the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_limited_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  filter_limited_cloud = filter_limit_cloud(cloud_msg, "z", z_axis_limits);
  filter_limited_cloud = filter_limit_cloud(filter_limited_cloud, "x", x_axis_limits);
  filter_limited_cloud = filter_limit_cloud(filter_limited_cloud, "y", y_axis_limits);

  // Get biggest plane
  plane_info biggest_plane;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);


  biggest_plane.plane_coefficients = coefficients;
  biggest_plane.inlier_indices = inliers;
  biggest_plane.plane_cloud = plane_cloud;
  get_biggest_plane(&biggest_plane, filter_limited_cloud);
  pcl::copyPointCloud(*filter_limited_cloud, *biggest_plane.inlier_indices, *biggest_plane.plane_cloud);

  // Remove the biggest plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud = filter_plane_from_cloud(filter_limited_cloud, biggest_plane.inlier_indices);
  // pcl::toROSMsg(*filtered_cloud, cloud_filled_publish); // Downsampled cloud

  // Find the clusters and put them in a vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  clusters = find_clusters(filtered_cloud);

  // If we have valid clusters
  if (clusters.size() != 0)
  {
    // Find center point of the clusters and return the idx of nearest cluster
    nearestCluster = find_nearest_cluster(biggest_plane.plane_coefficients, clusters);

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
  

    std::cout << "Num of Clusters: " << clusters.size() << "\n";
    std::cout << "Cluster Num: " << nearestCluster.idx << "\n";

    pcl::toROSMsg(*clusters[nearestCluster.idx], cloud_publish);
    pcl::toROSMsg(*cloud_filled_msg, cloud_filled_publish); // Filled Point Cloud
    // pcl::toROSMsg(*biggest_plane.plane_cloud, cloud_filled_publish); // Biggest plane
    // pcl::toROSMsg(*cloud_msg, cloud_filled_publish); // Downsampled cloud
    pcl_conversions::moveFromPCL(*cloud_2_segmented_msg, cloud_2_segmented_publish);

    nearestCenter_publish.header = msg->header;
    nearestCenter_publish.pose.position.x = nearestCluster.centroid.x;
    nearestCenter_publish.pose.position.y = nearestCluster.centroid.y;
    nearestCenter_publish.pose.position.z = nearestCluster.centroid.z;
  }
  else
  {
    std::cout << "No Cluster Found!\n";
    nearestCenter_publish.pose.position.x = 0;
    nearestCenter_publish.pose.position.y = 0;
    nearestCenter_publish.pose.position.z = 0;
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
  std::cout << "PCL Version: " << PCL_VERSION << std::endl;
  
  segmented_obj_colour.r = 255;
  segmented_obj_colour.g = 255;
  segmented_obj_colour.b = 255;

  background_colour.r = 0;
  background_colour.g = 0;
  background_colour.b = 0;

  z_axis_limits.min = 0;
  z_axis_limits.max = 1;

  // Front and Back
  y_axis_limits.min = 0; // 0 is start of camera
  y_axis_limits.max = 0.8;    

  x_axis_limits.min = -0.25; // 0 is mid, Left is neg. right is pos
  x_axis_limits.max = 0.25;  
  // Initialise ROS and specify name of node 
  ros::init(argc, argv, "segment");
  
  // Initialise the node handle
  ros::NodeHandle nh;

  ros::Rate rate{1};

  // Robot nc = Robot(&nh, rate);

  pub_nearestCloud = nh.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster", 1);
  pub_nearestCloud_filled = nh.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster_FilledPointCloud", 1);
  pub_nearestCloud2 = nh.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloud2Cluster", 1);
  pub_nearestCloudCenter = nh.advertise<geometry_msgs::PoseStamped>("/armCamera/nearestCloudCluster_Centroid", 1);
  ros::Subscriber sub = nh.subscribe("/armCamera/depth_registered/points", 1, cloud_callback);

  ros::spin();
  }
