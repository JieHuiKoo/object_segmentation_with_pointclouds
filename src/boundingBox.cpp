#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>

#include <opencv2/features2d.hpp>

image_transport::Publisher annotatedImagePub;
ros::Publisher boundingBoxPointsPub;

cv::Mat get_cluster_mask(cv::Mat inputMat)
{
  // Convert image to grayscale
  cv::Mat outputMat;
  cv::cvtColor(inputMat, outputMat, CV_BGR2GRAY);

  // Threshold to zero everything except cluster. Cluster is denoted with 255 pixel values
  int thresholdValue = 254;
  int thresholdType = 3;
  int const maxValue = 255;
  cv::threshold(outputMat, outputMat, thresholdValue, maxValue, thresholdType);

  int elementSize = 9;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                      cv::Size(2*elementSize + 1, 2*elementSize+1),
                                      cv::Point(elementSize, elementSize));

  cv::dilate(outputMat, outputMat, element);

  return outputMat;
}

cv_bridge::CvImagePtr convert_to_cv(sensor_msgs::ImageConstPtr msg)
{
    cv_bridge::CvImagePtr cvPtr;
    
    try
    {
      cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cvPtr;
}

std::vector<cv::Point> get_biggest_contour(cv::Mat image)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

  float biggestContourArea = 0;
  int biggestContourIdx = 0;
  for (size_t idx = 0; idx < contours.size(); idx++) 
  {
    float contourArea = cv::contourArea(contours[idx]);
      if (contourArea > biggestContourArea)
      {
        biggestContourArea = contourArea;
        biggestContourIdx = idx;
      }
  }

  return contours[biggestContourIdx];
}

std::vector<cv::Point> calc_bounding_box_points(std::vector<cv::Point> biggestContourPoints, cv::Point maxPoint)
{
  std::vector<cv::Point> boundingBoxPoints;

  cv::Point topLeft;
  topLeft.x = INT_MAX;
  topLeft.y = INT_MAX;
  cv::Point bottomRight;
  bottomRight.x = INT_MIN;
  bottomRight.y = INT_MIN;

  for (size_t idx = 0; idx < biggestContourPoints.size(); idx++)
  {
    cv::Point contourPoint = biggestContourPoints[idx];
    if (topLeft.x > contourPoint.x)
      topLeft.x = contourPoint.x - 20;
    if (topLeft.y > contourPoint.y)
      topLeft.y = contourPoint.y - 20;
    if (bottomRight.y < contourPoint.y)
      bottomRight.y = contourPoint.y + 20;
    if (bottomRight.x < contourPoint.x)
      bottomRight.x = contourPoint.x + 20;
  }

  if (topLeft.x < 0)
    topLeft.x = 0;
  if (topLeft.y < 0)
    topLeft.y = 0;
  if (bottomRight.x > maxPoint.x)
    bottomRight.x = maxPoint.x;
  if (bottomRight.y > maxPoint.y)
    bottomRight.y = maxPoint.y;

  boundingBoxPoints.push_back(topLeft);
  boundingBoxPoints.push_back(bottomRight);

  return boundingBoxPoints;
}

visualization_msgs::Marker convert_CVpoints_to_MarkerPoints(std::vector<cv::Point> CVpoints)
{
  if (CVpoints.size() % 2 != 0)
  {
    ROS_ERROR("Bounding Box Exception: Bounding box needs to be even number of points");
  }
  
  visualization_msgs::Marker boundingBoxMarker;
  boundingBoxMarker.type = visualization_msgs::Marker::POINTS;

  for (size_t idx = 0; idx < CVpoints.size(); idx++)
  {
    geometry_msgs::Point p;
    p.x = CVpoints[idx].x;
    p.y = CVpoints[idx].y;
    boundingBoxMarker.points.push_back(p);
  }

  return boundingBoxMarker;  
}

void imageCallback(const sensor_msgs::ImageConstPtr&);

int main(int argc, char **argv)
{
  std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  
  ros::init(argc, argv, "boundingBox");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/armCamera/nearestCloudCluster_ImageFromFilledPointCloud", 1, imageCallback);
  annotatedImagePub = it.advertise("/armCamera/nearestCloudCluster_AnnotatedImageFromFilledPointCloud", 1);
  boundingBoxPointsPub = nh.advertise<visualization_msgs::Marker>("/armCamera/nearestCloudCluster_BoundingBoxPoints", 1);

  ros::spin();
  cv::destroyWindow("view");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert ROS image to opencv image
  cv_bridge::CvImagePtr cvPtr;
  cvPtr = convert_to_cv(msg);

  // Create thresholded mask of the object
  cv::Mat clusterMask;
  clusterMask = get_cluster_mask(cvPtr->image);

  // Get Contour Points of the biggest contour
  std::vector<cv::Point> biggestContourPoints;
  biggestContourPoints = get_biggest_contour(clusterMask);
  
  // Calculate Bounding Box Coordinates
  std::vector<cv::Point> boundingBoxPoints;
  boundingBoxPoints = calc_bounding_box_points(biggestContourPoints, cv::Point(cvPtr->image.cols, cvPtr->image.rows));

  // Convert the vector of CVpoints to markerPoints
  visualization_msgs::Marker boundingBoxMarker;
  boundingBoxMarker = convert_CVpoints_to_MarkerPoints(boundingBoxPoints);
  
  // Draw the bounding box 
  cv::rectangle(cvPtr->image, boundingBoxPoints[0], boundingBoxPoints[1], cv::Scalar(0, 255, 0), 3);

  // Publish the image and coordinates of bounding box
  annotatedImagePub.publish(cvPtr->toImageMsg());
  boundingBoxPointsPub.publish(boundingBoxMarker);
}