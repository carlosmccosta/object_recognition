# object_recognition

ROS package for estimating the 6 DoF pose of objects on top of a plane (in the future it will support object recognition using global descriptors from PCL).

This package relies on [dynamic_robot_localization](https://github.com/carlosmccosta/dynamic_robot_localization) for point cloud filtering, segmentation and registration.

Configuration of this package for correcting point cloud offsets available in the [pointcloud_registration](https://github.com/carlosmccosta/pointcloud_registration) repository.

Presentation of 6 DoF pose estimation benchmarks available [here](https://www.dropbox.com/s/frnpqfj71omnet9/Scalable_RM_T4.5.pptx?dl=0).


[![6 DoF pose estimation of objects on top of a conveyor](http://img.youtube.com/vi/-RKdsaOq-58/maxresdefault.jpg)](http://www.youtube.com/watch?v=-RKdsaOq-58&t=42)

Video 1: 6 DoF pose estimation of objects on top of a conveyor using PCA and ICP matching


[![6 DoF pose estimation of objects on top of a conveyor](http://img.youtube.com/vi/pSa6ALIIDis/maxresdefault.jpg)](http://www.youtube.com/watch?v=pSa6ALIIDis&t=79)

Video 2: 6 DoF pose estimation of objects on top of a conveyor using PCA and ICP matching


[![6 DoF pose estimation of objects on top of a conveyor](http://img.youtube.com/vi/GRR2e6lvnEs/maxresdefault.jpg)](http://www.youtube.com/watch?v=GRR2e6lvnEs)

Video 3: 6 DoF pose estimation of objects on top of a conveyor using feature matching and ICP from CAD data


[![Bin picking using region growing and principal component analysis](http://img.youtube.com/vi/6BgnQP4TeJM/maxresdefault.jpg)](http://www.youtube.com/watch?v=6BgnQP4TeJM)

Video 4: 6 DoF pose estimation of the vacuum grasp pose for smooth objects inside bins using region growing and principal component analysis


![Overview of the main processing modules](https://raw.githubusercontent.com/carlosmccosta/dynamic_robot_localization/kinetic-devel/docs/perception-overview.png)

Figure 1: Overview of main processing modules


## Usage

### Startup

For starting the perception pipeline, run the [object_recognition.launch](object_recognition_skill_server/launch/object_recognition.launch) after configuring it to your 3D sensing hardware (namely, specifying the ambient_pointcloud_topic and frame_ids).
```
roslaunch object_recognition_skill_server object_recognition.launch
```

### Configuration

By default, the perception system is configured to perform 6 DoF pose estimation of objects on top of a plane using 3D feature matching followed by Iterative Closest Point (ICP) registration.

Currently, the objects are segmented using [euclidean clustering](object_recognition_skill_server/yaml/filters_euclidean_clustering.yaml) and sorted by their point count, and as such, require the objects to be separated from each other by at least 15 mm.

You can change which modules are loaded by the perception pipeline by activating / deactivating flags available in the [object_recognition.launch](object_recognition_skill_server/launch/object_recognition.launch) or configuring new yamls for your particular use case (check the documentation on [drl_configs.yaml](https://github.com/carlosmccosta/dynamic_robot_localization/blob/noetic-devel/yaml/schema/drl_configs.yaml)).

The perception pipeline was pre-configured for thin objects in which the reference point cloud is generated from CAD data (check documentation in [mesh_tessellation.md](https://github.com/carlosmccosta/dynamic_robot_localization/blob/melodic-devel/docs/mesh_tessellation.md)).

Thin objects are a challenge because a correspondence estimation algorithm (that associates sensor points to reference points) may pick the wrong side of the surface if it selects the closest point without taking into consideration the surface normals. For solving this issue, each point in the sensor data cluster must be compared to a region of points in the reference point cloud, taking into consideration the distance between the sensor point and the reference points while also including the surface normals data for selecting the best correspondences.

If your reference point cloud does not contain surfaces too close to each other (associated with thin objects), you can reduce the perception time by setting `use_iterative_closest_point_with_normals` to false in the [object_recognition.launch](object_recognition_skill_server/launch/object_recognition.launch).

Moreover, if the objects arrive in front of the sensor in a controlled environment (such as a conveyor), you can significantly reduce the perception time by using Principal Component Analysis (PCA) instead of feature matching. For enabling PCA, set `use_pca_for_initial_alignment` to true and `use_feature_matching_for_initial_alignment` to false in the [object_recognition.launch](object_recognition_skill_server/launch/object_recognition.launch) and then adjust the PCA flip axes in either the [PCA yaml](object_recognition_skill_server/yaml/initial_pose_estimators_pca_matchers.yaml) or the [action goal](object_recognition_skill_msgs/action/ObjectRecognitionSkill.action).

Initial pose estimation for thin objects using normals may take around 3 seconds, while disabling normals and relying on a correspondence estimation algorithm that uses only the closest point may reduce the perception time to 2 seconds. Finally, using PCA may reduce the perception time further, to 0.5 seconds.

As such, for efficient usage of the perception pipeline, you should customize it to your particular use case requirements and    hardware (check the documentation on [drl_configs.yaml](https://github.com/carlosmccosta/dynamic_robot_localization/blob/noetic-devel/yaml/schema/drl_configs.yaml)).

For inspecting the perception pipeline configuration, check the parameter server (after running the [object_recognition.launch](object_recognition_skill_server/launch/object_recognition.launch)):
```
rosparam get /object_recognition/object_recognition_skill_server/ -p
```

### ROS action

For triggering the perception pipeline, capture a 3D point cloud and then send a ROS action goal to `/object_recognition/ObjectRecognitionSkill/goal`.

Check the documentation in [ObjectRecognitionSkill.action](object_recognition_skill_msgs/action/ObjectRecognitionSkill.action) for more details about the goal fields.

The reference point cloud must have uniform point density (for CAD data, check [mesh_tessellation.md](https://github.com/carlosmccosta/dynamic_robot_localization/blob/melodic-devel/docs/mesh_tessellation.md)).


### Perception results

The perception pipeline publishes a TF named `object` along with other diagnostic topics, that can be inspected in the command line and rviz.

```
rostopic list /object_recognition/
roslaunch object_recognition_skill_server rviz.launch
```
