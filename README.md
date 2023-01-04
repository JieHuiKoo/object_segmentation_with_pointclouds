# object-segmentation-with-pointclouds

To run, execute the following command
```
rosrun object-segmentation-with-pointclouds object-segmentation-with-pointclouds_segment
```

The node will publish 3 topics:
1) /armCamera/nearest_cloudCluster - An unorganised pointcloud containing only the cluster
2) /armCamera/nearest_cloudCluster_filled - An organised pointcloud with the input pointcloud and detected nearest object coloured in white
3) /armCamera/nearest_cloudClusterCentroid - A point message detailing the x,y,z coordinates of the centroid of the nearest object

To convert the organised pointcloud to image, run the following command
```
rosrun pcl_ros convert_pointcloud_to_image input:=/armCamera/nearest_cloudCluster_filled output:=/armcamera/depth/converted_2d
```
