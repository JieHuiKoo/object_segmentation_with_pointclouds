#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat obtain_cluster_mask(cv::Mat input_mat)
{
    // Convert image to grayscale
    cv::Mat output_mat;
    cv::cvtColor(input_mat, output_mat, CV_BGR2GRAY);

    // Threshold to zero everything except cluster. Cluster is denoted with 255 pixel values
    int threshold_value = 254;
    int threshold_type = 3;
    int const max_value = 255;
    int const max_binary_value = 255;
    cv::threshold(output_mat, output_mat, threshold_value, max_binary_value, threshold_type );

    return output_mat;
}

cv_bridge::CvImagePtr convert_to_cv(sensor_msgs::ImageConstPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cv_ptr;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = convert_to_cv(msg);

    cv::Mat cluster_mask;
    cluster_mask = obtain_cluster_mask(cv_ptr->image);

    cv::imshow("view", cluster_mask);
    cv::waitKey(30);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "boundingBox");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/armcamera/nearestCloudClusterImage", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}