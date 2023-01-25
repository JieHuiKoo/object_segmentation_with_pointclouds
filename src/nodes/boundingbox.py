#! /home/jiehui/anaconda3/envs/tensorflow/bin/python

import os
cwd = os.getcwd()

import rospy
import sys
import math
import numpy as np
import os
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

print("Python Version: " + str(sys.version_info[0]) + '.' + str(sys.version_info[1]))
print("OpenCV Version: " + str(cv2.__version__))

def imgmsg_to_cv2(img_msg):
    rgb8_flag = 0
    if img_msg.encoding != "bgr8":
        rgb8_flag = 1
    
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    if rgb8_flag:
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)

    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

def get_cluster_mask(input_image):
    gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

    # Threshold to zero everything except cluster. Cluster is denoted with 255 pixel values
    threshold_value = 254
    max_value = 255
    _, thresholded = cv2.threshold(gray, threshold_value, max_value, cv2.THRESH_TOZERO)

    dilated = cv2.dilate(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9)), iterations=3)

    return dilated

def get_biggest_contour(image):
    contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    biggest_contour_area = 0
    biggest_contour_idx = 0

    for idx in range(0, len(contours)):
        contour = contours[idx]
        contour_area = cv2.contourArea(contour)
        if contour_area > biggest_contour_area:
            biggest_contour_area = contour_area
            biggest_contour_idx = idx

    return contours, biggest_contour_idx

def get_bounding_box_points(biggest_contour_points, max_point):
    
    bounding_box_marker = Marker()
    bounding_box_marker.type = 8
    
    top_left = Point(np.inf, np.inf, None)
    bottom_right = Point(0, 0, None)

    for contour in biggest_contour_points:
        contour_point = contour[0]  
    
        if top_left.x > contour_point[0]:
            top_left.x = contour_point[0] - 20
        if top_left.y > contour_point[1]:
            top_left.y = contour_point[1] - 20
        if bottom_right.x < contour_point[0]:
            bottom_right.x = contour_point[0] + 20
        if bottom_right.y < contour_point[1]:
            bottom_right.y = contour_point[1] + 20

    if top_left.x < 0:
        top_left.x = 0
    if top_left.y < 0:
        top_left.y = 0
    if bottom_right.x > max_point.x:
        bottom_right.x = max_point.x
    if bottom_right.y > max_point.y:
        bottom_right.y = max_point.y
    
    bounding_box_marker.points = [top_left, bottom_right]

    return bounding_box_marker

def draw_bounding_box(image, marker):
    top_left = marker.points[0]
    bottom_right = marker.points[1]
    image = cv2.rectangle(image, (top_left.x, top_left.y), (bottom_right.x, bottom_right.y), (255, 0, 0), 3)
    return image

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def process_image(image_msg):

    # Declare the cvBridge object
    proc_image = imgmsg_to_cv2(image_msg)

    # Create thresholded mask of the object
    cluster_mask = get_cluster_mask(proc_image)

    # Get contour points of the biggest contour
    contours, biggest_contour_idx = get_biggest_contour(cluster_mask)
    # showImage(cv2.drawContours(proc_image, contours, biggest_contour_idx, (0,255,0), 3))

    # Calculate Bounding Box Coordinates
    bounding_box_marker = get_bounding_box_points(contours[biggest_contour_idx], Point(proc_image.shape[1], proc_image.shape[0], None))

    # Draw Bounding Box Coordinates
    proc_image = draw_bounding_box(proc_image, bounding_box_marker)

    image_pub = cv2_to_imgmsg(proc_image)

    bounding_box_points_pub = rospy.Publisher("/armCamera/nearestCloudCluster_BoundingBoxPoints", Marker, queue_size = 1)
    bounding_box_points_pub.publish(bounding_box_marker)

    bounding_box_image_pub = rospy.Publisher('armCamera/nearestCloudCluster_AnnotatedImage', Image, queue_size=1)
    bounding_box_image_pub.publish(image_pub)

def start_node():
    rospy.init_node('image_masking')
    rospy.loginfo('image_identify node started')

    rospy.Subscriber("/armCamera/nearestCloudCluster_ImageFromFilledPointCloud", Image, process_image)
    
    rospy.spin()

if __name__ == '__main__':
    
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass

