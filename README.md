# object_recognition

ROS package for estimating the 6 DoF pose of objects on top of a plane (in the future it will support object recognition using global descriptors from PCL).

This package relies on [dynamic_robot_localization](https://github.com/carlosmccosta/dynamic_robot_localization) for point cloud filtering, segmentation and registration.

Presentation of 6 DoF pose estimation benchmarks available [here](https://www.dropbox.com/s/frnpqfj71omnet9/Scalable_RM_T4.5.pptx?dl=0).


![Overview of main processing modules](https://www.dropbox.com/s/xs8g477731vigdh/localization-system-modules.png?raw=1)

Figure 1: Overview of main processing modules


[![6 DoF pose estimation of objects on top of a conveyor](http://img.youtube.com/vi/GRR2e6lvnEs/maxresdefault.jpg)](http://www.youtube.com/watch?v=GRR2e6lvnEs)

Video 1: 6 DoF pose estimation of objects on top of a conveyor using CAD matching


[![Bin picking using region growing and principal component analysis](http://img.youtube.com/vi/6BgnQP4TeJM/maxresdefault.jpg)](http://www.youtube.com/watch?v=6BgnQP4TeJM)

Video 2: 6 DoF pose estimation of the vacuum grasp pose for smooth objects inside bins using region growing and principal component analysis
