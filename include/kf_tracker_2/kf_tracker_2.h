#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <time.h> // We don't need it for run. It's for Runtime check
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>

// ros msgs
#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

// obstacle ros msgs
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovariance.h>

//tf msgs
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// pcl 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/common/geometry.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

// IHGP
#include "ihgp/InfiniteHorizonGP.hpp"
#include "ihgp/Matern32model.hpp"
 
// visualization
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>

//*****************************************************************
using namespace std;
using namespace Eigen;

class ObstacleTrack
{
public:

    ObstacleTrack();

    ~ObstacleTrack();

    bool initialize();

    void updateParam();

    ros::Publisher obstacle_pub; // obstacle pos&vel
    ros::Publisher marker_pub; // obstacle pose visualization 

    ros::Subscriber pointnet_sub;
    
    // RUNTIME DEBUG
    clock_t s_1, s_2, s_3, s_4, s_5, s_6, s_7;
    clock_t e_1, e_2, e_3, e_4, e_5, e_6, e_7;
    
    // Stack of objects position
    std::vector<std::vector<pcl::PointXYZI>> objects_centroids; 
    std::vector<int> objIDs = {}; // object ID

    // IHGP
    float dt_gp;  
    std::vector<InfiniteHorizonGP*> GPs_x;
    std::vector<InfiniteHorizonGP*> GPs_y;
    
    // configuration
    int obstacle_num_;
    float ClusterTolerance_; // (m) default 0.3
    int MinClusterSize_; // default 10
    int MaxClusterSize_; // default 600
    float VoxelLeafSize_;

    double logSigma2_x_;
    double logMagnSigma2_x_;
    double logLengthScale_x_;

    double logSigma2_y_;
    double logMagnSigma2_y_;
    double logLengthScale_y_;

    int data_length=10; // IHGP data length
    bool param_fix;

    // check
    bool firstFrame = true;
    bool t_init = false;
    double time_init; // inital timestamp for real world test

    //tf msgs
    // tf::TransformBroadcaster tf_broadcast;
    // tf::Transform transform;
    // tf::TransformListener tf_listener;
    // tf::StampedTransform stamped_transform;

    //radius
    float obstacle_radius;
    
private:

    ros::NodeHandle nh_;

    void spinNode();

    void pointnetCallback(const geometry_msgs::PoseArray input_msg);

    void publishObstacles(pcl::PointXYZI predicted_centroid, \
        pcl::PointXYZI predicted_velocity);

    void publishMarkers(pcl::PointXYZI predicted_centroid);

    void publishObjID();

    pcl::PointXYZI IHGP_fixed(std::vector<pcl::PointXYZI> centroids, int n, string variable);
};