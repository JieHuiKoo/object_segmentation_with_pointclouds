# object-segmentation-with-pointclouds

To run, execute the following command
```
rosrun object-segmentation-with-pointclouds object-segmentation-with-pointclouds_segment
```

The node will publish 3 topics:
1) /armCamera/nearestCloudCluster - An unorganised pointcloud containing only the cluster
<img src="https://user-images.githubusercontent.com/31171083/210493870-480ec94c-8f88-4930-81c2-5c7e41be2044.png" width="500" height="300">
2) /armCamera/nearestCloudClusterFilled - An organised pointcloud with the input pointcloud and detected nearest object coloured in white
<img src="https://user-images.githubusercontent.com/31171083/210493942-444399c9-4d7a-4bc0-b6a2-85e44000ca29.png" width="500" height="300">
3) /armCamera/nearestCloudClusterCentroid - A point message detailing the x,y,z coordinates of the centroid of the nearest object
   <br />
   <br />


To convert the organised pointcloud to image, run the following command
```
rosrun pcl_ros convert_pointcloud_to_image input:=/armCamera/nearest_cloudClusterFilled output:=/armcamera/nearest_cloudClusterImage
```
<img src="https://user-images.githubusercontent.com/31171083/210494018-130a06d2-2968-4163-b348-64517a69a83d.png" width="500" height="300">
