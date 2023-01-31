# detect_area_distance

## Installation
### Cloning this repository
```
cd ~/catkin_ws/src
git clone https://github.com/arayabrain/detect_face.git
```
### Installation of dependencies
```
pip install open3d

```

## Running
```
rosrun detect_face main.py
```

### Input/Output topic
#### Input
- input_topic [PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html): /hokuyo3d/hokuyo3d/hokuyo_cloud2_field by default.

#### Output
- is_face_topic [Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html): /is_face by default. 

- face_pcd_topic [PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html): /face_pcd by default. 

- face_position_topic [PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html): /face_position by default. 

### Parameters
- voxel_size(float): . Sampling interval of point cloud. Default value is 0.5.
- inlier_threshold(float) : Threshold for the distance between the plane and the point cloud when estimating the plane. Default value is 0.5
- face_width(float) : Width of face plane. Default value is 5.
- face_height(float) : Height of face plane. Default value is 8.5.
- error(float) : Threshold of the difference between the width and height of the face in determining whether the point cloud of the estimated plane can be considered a face. Default value is 5.
- point_num_threshold(float) : Threshold for the number of point clouds in determining whether a point cloud of an estimated plane can be considered a face. Default value is 5.5.