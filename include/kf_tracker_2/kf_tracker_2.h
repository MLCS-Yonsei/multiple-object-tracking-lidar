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

    ros::Publisher pc1;
    ros::Publisher pc2;
    ros::Publisher pc3;

    ros::Subscriber map_sub; // Occupied grid map
    ros::Subscriber input_sub; // input Pointclouds
    
    // RUNTIME DEBUG
    clock_t s_1, s_2, s_3, s_4, s_5, s_6, s_7;
    clock_t e_1, e_2, e_3, e_4, e_5, e_6, e_7;
    
    int data_length=10;
    std::vector<pcl::PointXYZI> centroids; // t~(t-10) cluster Centers stack
    std::vector<int> objID;
    nav_msgs::OccupancyGrid map_copy;

    // IHGP state space model
    Matern32model model_x;
    Matern32model model_y;

    // IHGP
    InfiniteHorizonGP gp_x;
    InfiniteHorizonGP gp_y;

    bool param_fix;
    float dt_gp;    
    double logSigma2_x_;
    double logMagnSigma2_x_;
    double logLengthScale_x_;
    double logSigma2_y_;
    double logMagnSigma2_y_;
    double logLengthScale_y_;

    double vel_x=999;
    double vel_y=999;
    double mean_x=0;
    double mean_y=0;
    std::vector<double> Eft_x;
    std::vector<double> Eft_y;
    std::vector<double> logLengthScales_x;
    std::vector<double> logLengthScales_y;
    std::vector<double> logMagnSigma2s_x;
    std::vector<double> logMagnSigma2s_y;
    
    // configuration
    int obstacle_num_;
    float ClusterTolerance_; // (m) default 0.3
    int MinClusterSize_; // default 10
    int MaxClusterSize_; // default 600
    float VoxelLeafSize_;
    bool firstFrame = true;
    bool t_init = false;
    bool map = false;
    double time_init;

    //tf msgs
    tf::TransformBroadcaster tf_broadcast;
    tf::Transform transform;
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_transform;

    //radius
    float obstacle_radius;
    
private:

    ros::NodeHandle nh_;

    void spinNode();

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

    void mapCallback(const nav_msgs::OccupancyGrid& map_msg);

    void publishObstacles(pcl::PointXYZI predicted_centroid, \
        pcl::PointXYZI predicted_velocity,\
        const sensor_msgs::PointCloud2ConstPtr& input);

    void publishMarkers(pcl::PointXYZI predicted_centroid);

    void publishObjID();

    pcl::PointCloud<pcl::PointXYZ> removeStatic( \
        pcl::PointCloud<pcl::PointXYZ> input_cloud, \
        pcl::PointCloud<pcl::PointXYZ> cloud_pre_process);

    pcl::PointXYZI getCentroid( \
        std::vector<pcl::PointIndices> cluster_indices, \
        const pcl::PointCloud<pcl::PointXYZ> cloud_filtered, \
        const sensor_msgs::PointCloud2 input);

    pcl::PointXYZI IHGP_fixed(std::vector<pcl::PointXYZI> centroids, string variable);

    pcl::PointXYZI IHGP_nonfixed(std::vector<pcl::PointXYZI> centroids);

    float euc_dist(Vector3d P1, Vector3d P2);

    pcl::PointXYZI getRelativeCentroid(pcl::PointXYZI centroid);
};
