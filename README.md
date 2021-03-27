# Multiple objects detection, tracking and classification from LIDAR scans/point-clouds (Changed to Class form)

[![DOI](https://zenodo.org/badge/47581608.svg)](https://zenodo.org/badge/latestdoi/47581608)

![Sample demo of multiple object tracking using LIDAR scans](https://media.giphy.com/media/3YKG95w9gu263yQwDa/giphy.gif)

PCL based ROS package to Detect/Cluster --> Track --> Classify static and dynamic objects in real-time from LIDAR scans implemented in C++.

### Features:

- Stable tracking (object ID, Position, Velocity) with an ensemble of Infinite Horizon Gaussian Process
- K-D tree based point cloud processing for object feature detection from point clouds
- Unsupervised euclidean cluster extraction (3D)
- Robust compared to k-means clustering with mean-flow tracking

### Usage:

#### Installation

Follow the steps below to use this (`multiple_object_tracking_lidar`) package:

``` bash
cd ~/catkin_ws/src
git clone https://github.com/MLCS-Yonsei/multiple-object-tracking-lidar.git -b gp
cd ..
catkin_make --only-pkg-with-deps multiple_object_tracking_lidar
```

#### Run

##### 1. Run Demo rosbag file

terminal 1,2,3 mean "open new terminal and run this command"

``` bash
(terminal 1)$ roscore
(terminal 2)$ rosbag play ${HOME}/catkin_ws/src/multiple-object-tracking-lidar/bag/gazebo_sim_01.bag -r 1.0
```

##### 2. Run tracker node

``` bash
(terminal 3)$ roslaunch multiple-object-tracking-lidar simTracker.launch
```

you can change some tracking configuration on launch file.


### Supported point-cloud streams/sources:
The input point-clouds can be from:
1. A real LiDAR or 
2. A simulated LiDAR or 
3. A point cloud dataset or 
4. Any other data source that produces point clouds

### TODO

- [x] fix private and public
- [x] lower removeStatic() computing resource
- [ ] map orientation bug fix

- [x] PCL Clustering memorized last step
- [x] multi obstacle ID

- [x] clean up codes (callback parts)
- [x] add comments
- [x] update READ.md

- [ ] change IHGP filter to Bilateral Filter for position
- [ ] change multiple lidar merging method
- [ ] solve Occlusion Problem

## Citing

If you use the code or snippets from this repository in your work, please cite:

```bibtex
@software{solin2018infinite,
  title={Infinite-horizon Gaussian processes},
  author={Solin, Arno and Hensman, James and Turner, Richard E},
  journal={Advances in Neural Information Processing Systems},
  volume={31},
  pages={3486--3495},
  year={2018}
}
```

```bibtex
@software{praveen_palanisamy_2019_3559187,
  author       = {Praveen Palanisamy},
  title        = {{praveen-palanisamy/multiple-object-tracking-lidar: 
                   Multiple-Object-Tracking-from-Point-Clouds_v1.0.2}},
  month        = dec,
  year         = 2019,
  publisher    = {Zenodo},
  version      = {1.0.2},
  doi          = {10.5281/zenodo.3559187},
  url          = {https://doi.org/10.5281/zenodo.3559186}
}
```
