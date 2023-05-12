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
#include <pcl/surface/mls.h>

#include <sstream>

ros::Subscriber sub;
ros::Publisher pub_nearestCloud;
ros::Publisher pub_nearestCloud_filled;
ros::Publisher pub_nearestCloudCenter;
ros::Publisher pub_nearestCloud2; 

// For debug
ros::Publisher pub_debug1;
ros::Publisher pub_debug2;
ros::Publisher pub_debug3;
ros::Publisher pub_debug4;
ros::Publisher pub_debug5;

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
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr plane_cloud;
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

cluster_info nearestClusterInfo;

pcl::PointXYZ calc_cluster_center(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster)
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

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr fill_pointcloud_cluster_with_colour(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgb, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster_xyz, rgb_values &fill_colour)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
  
  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
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

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filter_limit_cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud, std::string axis, filter_limits axis_limits)
{
  // Create output cloud
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // Create passthrough object
  pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(axis_limits.min, axis_limits.max);
  pass.setKeepOrganized(false);
  pass.filter (*output_cloud);

  return output_cloud;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filter_plane_from_cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud, pcl::PointIndices::Ptr plane_indices)
{
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;

  // Extract All points that is the plane
  extract.setInputCloud(input_cloud);
  extract.setIndices(plane_indices);
  extract.setNegative(true);
  extract.filter(*input_cloud);
  
  return input_cloud;
}

void get_biggest_plane(plane_info* biggest_plane, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud)
{
  // Get segmentation ready
  pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
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

std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> find_clusters(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud)
{
  // Creating the KdTree object for the search method of the euclidean extraction
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree->setInputCloud(input_cloud);

  // Set up Eucludean Cluster Extraction
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
  ec.setClusterTolerance (0.01); // in m
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract(cluster_indices);

  // Push each cluster into a vector
  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clusters;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
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

cluster_info find_nearest_cluster(pcl::ModelCoefficients::Ptr plane_coefficients, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clusters)
{
  // Declare initial nearest point
  cluster_info nearestCluster;

  nearestCluster.centroid.y = FLT_MAX;

  for (int i = 0; i < clusters.size(); ++i)
  {
    // Find center point of the clusters
    pcl::PointXYZ clusterCenter;
    clusterCenter = calc_cluster_center(clusters[i]);

    if (is_cluster_on_plane(clusterCenter, plane_coefficients))
    {
      float object_dist_from_camera = calc_dist_from_camera(clusterCenter);

      if (object_dist_from_camera > 0.2) // original = 0.1
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

cluster_info find_largest_cluster(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clusters)
{
  cluster_info largestCluster;

  int largestCount = -1;
  int index = -1;
  int currCount;
  for (int i = 0; i < clusters.size(); i++)
  {
    currCount = clusters[i] -> points.size();
    if (currCount > largestCount) 
    {
      largestCount = currCount;
      index = i;
    }
  }
  std::cout << "Size of largest cluster: " << largestCount << "\n";
  std::cout << "Index of largest cluster: " << index << "\n";
  largestCluster.idx = index;
  return largestCluster;
}

void apply_smoothing(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr segmentedCluster)
{
  // smoothing out nearest cluster found
  std::cout << "// Surface smoothening //" << "\n";

  // moving least average filter
  // // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree->setInputCloud(segmentedCluster);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (segmentedCluster);
  mls.setPolynomialOrder (1);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.001); //0.004
  // mls.setSqrGaussParam(0.01 * 0.01); // square of search radius

  // Reconstruct
  mls.process(*segmentedCluster);
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // For debug
  sensor_msgs::PointCloud2 cloud_publish_debug1;
  sensor_msgs::PointCloud2 cloud_publish_debug2;
  sensor_msgs::PointCloud2 cloud_publish_debug3;
  sensor_msgs::PointCloud2 cloud_publish_debug4;
  sensor_msgs::PointCloud2 cloud_publish_debug5;
  

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
  sor.setLeafSize (0.003, 0.003, 0.003);
  // sor.setLeafSize (0.005, 0.005, 0.005);
  sor.filter(cloud_downsampled);

  // Convert to pcl point cloud, XYZ
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::fromPCLPointCloud2(cloud_downsampled, *cloud_msg); 
  // pcl::fromPCLPointCloud2(*cloud, *cloud_msg); // skip downsampling
  pcl::toROSMsg(*cloud_msg, cloud_publish_debug1);


  // Convert to pcl point cloud, XYZRGB
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rgb_msg (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::fromROSMsg(*msg,*cloud_rgb_msg);

  // Filter the cloud
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filter_limited_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // filter_limited_cloud = filter_limit_cloud(cloud_msg, "z", z_axis_limits);
  filter_limited_cloud = filter_limit_cloud(cloud_msg, "x", x_axis_limits);
  // filter_limited_cloud = filter_limit_cloud(filter_limited_cloud, "x", x_axis_limits);
  filter_limited_cloud = filter_limit_cloud(filter_limited_cloud, "y", y_axis_limits);
  pcl::toROSMsg(*filter_limited_cloud, cloud_publish_debug2);

  // Get biggest plane
  plane_info biggest_plane;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  biggest_plane.plane_coefficients = coefficients;
  biggest_plane.inlier_indices = inliers;
  biggest_plane.plane_cloud = plane_cloud;
  get_biggest_plane(&biggest_plane, filter_limited_cloud);
  pcl::copyPointCloud(*filter_limited_cloud, *biggest_plane.inlier_indices, *biggest_plane.plane_cloud);

  // Remove the biggest plane
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  filtered_cloud = filter_plane_from_cloud(filter_limited_cloud, biggest_plane.inlier_indices);
  pcl::toROSMsg(*filter_limited_cloud, cloud_publish_debug3);

  // Find the clusters and put them in a vector
  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clusters;
  clusters = find_clusters(filtered_cloud);

  // If we have valid clusters
  if (clusters.size() != 0)
  {
    // Find center point of the clusters and return the idx of nearest cluster
    nearestClusterInfo = find_nearest_cluster(biggest_plane.plane_coefficients, clusters);
    // nearestClusterInfo = find_largest_cluster(clusters);

    std::cout << "Num of Clusters: " << clusters.size() << "\n";
    std::cout << "Cluster Num: " << nearestClusterInfo.idx << "\n";

    // Smooth the nearestCluster
    // apply_smoothing(clusters[nearestClusterInfo.idx]);
    pcl::toROSMsg(*clusters[nearestClusterInfo.idx], cloud_publish_debug5);


    // Compare the points of cluster and original, and colour them in white
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filled_msg (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    cloud_filled_msg = fill_pointcloud_cluster_with_colour(cloud_rgb_msg, clusters[nearestClusterInfo.idx], segmented_obj_colour);

    pcl::toROSMsg(*clusters[nearestClusterInfo.idx], cloud_publish);
    pcl::toROSMsg(*cloud_filled_msg, cloud_filled_publish); // Filled Point Cloud

    nearestCenter_publish.header = msg->header;
    nearestCenter_publish.pose.position.x = nearestClusterInfo.centroid.x;
    nearestCenter_publish.pose.position.y = nearestClusterInfo.centroid.y;
    nearestCenter_publish.pose.position.z = nearestClusterInfo.centroid.z;
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
  pub_nearestCloudCenter.publish(nearestCenter_publish);


  // For Debug

  cloud_publish_debug1.header = msg->header;
  pub_debug1.publish(cloud_publish_debug1);

  cloud_publish_debug2.header = msg->header;
  pub_debug2.publish(cloud_publish_debug2);

  cloud_publish_debug3.header = msg->header;
  pub_debug3.publish(cloud_publish_debug3);

  cloud_publish_debug5.header = msg->header;
  pub_debug5.publish(cloud_publish_debug5);
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

  z_axis_limits.min = -1;
  z_axis_limits.max = 1;

  // Front and Back
  y_axis_limits.min = -1; // 0 is start of camera
  y_axis_limits.max = 2; // 40 cm   

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
  pub_nearestCloudCenter = nh.advertise<geometry_msgs::PoseStamped>("/armCamera/nearestCloudCluster_Centroid", 1);
  ros::Subscriber sub = nh.subscribe("/armCamera/depth_registered/points", 1, cloud_callback);

  // For debug
  pub_debug1 = nh.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster_debug1", 1);
  pub_debug2 = nh.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster_debug2", 1);
  pub_debug3 = nh.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster_debug3", 1);
  pub_debug4 = nh.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster_debug4", 1);
  pub_debug5 = nh.advertise<sensor_msgs::PointCloud2>("/armCamera/nearestCloudCluster_debug5", 1);

  ros::spin();
  }
