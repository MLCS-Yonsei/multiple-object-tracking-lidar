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
#include <sstream>
#include <iomanip>
#include <cstdlib> 

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
#include <std_msgs/ColorRGBA.h>

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

    void spinNode();

    ros::Publisher obstacle_pub; // obstacle pos&vel
    ros::Publisher marker_pub; // obstacle pose visualization 

    // pointcloud publisher for debugging
    ros::Publisher pc1; 
    ros::Publisher pc2;
    ros::Publisher pc3;

    ros::Subscriber map_sub; // Occupied grid map
    ros::Subscriber input_sub; // input Pointclouds

private:   
    // clustering
    double time_init;
    bool firstFrame = true;
    std::vector<int> objIDs; // obj lists
    std::vector<std::vector<pcl::PointXYZI>> stack_obj; // t~(t-10) cluster Centers stack
    std::vector<std_msgs::ColorRGBA> colorset; // rviz msg colorset

    // IHGP state space model
    int next_obj_num = 0;
    int spin_counter = 0;
    float dt_gp; 
    std::vector<InfiniteHorizonGP*> GPs_x;
    std::vector<InfiniteHorizonGP*> GPs_y;

    // map data
    nav_msgs::MapMetaData map_info;
    Eigen::MatrixXd map_copy;
    bool map_init = false;
    
    // Configureation (ROS parameter)
    float frequency; // node frequency

    float ClusterTolerance; // (m) default 0.3
    int MinClusterSize; 
    int MaxClusterSize; 
    float VoxelLeafSize;
    
    int tolarance; // tolarance for removing pointcloud on static obstacles
    float id_thershold;
 
    double logSigma2_x;
    double logMagnSigma2_x;
    double logLengthScale_x;

    double logSigma2_y;
    double logMagnSigma2_y;
    double logLengthScale_y;

    int data_length=10;
    bool param_fix;

    // RUNTIME DEBUG
    clock_t s_1, s_2, s_3, s_4, s_5, s_6, s_7;
    clock_t e_1, e_2, e_3, e_4, e_5, e_6, e_7;


    ros::NodeHandle nh_;

    void updateParam();

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

    void mapCallback(const nav_msgs::OccupancyGrid& map_msg);

    void publishObstacles(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, \
                        std::string frame_id, \
                        std::vector<int> this_objIDs);

    void publishMarkers(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, \
                        std::string frame_id, \
                        std::vector<int> this_objIDs);

    std::vector<pcl::PointXYZI> clusterPointCloud(const sensor_msgs::PointCloud2ConstPtr& input);

    void registerNewObstacle(pcl::PointXYZI centroid);

    void unregisterOldObstacle(double now);

    void updateObstacleQueue(const int i, pcl::PointXYZI centroid);

    void fill_with_linear_interpolation(const int i, pcl::PointXYZI centroid);

    float quaternion2eularYaw(geometry_msgs::Quaternion map_angle);

    float euc_dist(Vector3d P1, Vector3d P2);

    std::vector<std::vector<pcl::PointXYZI>> callIHGP(std::vector<int> this_objIDs);

    pcl::PointCloud<pcl::PointXYZ> removeStatic(pcl::PointCloud<pcl::PointXYZ> input_cloud);

    std::vector<pcl::PointXYZI> getCentroid( \
        std::vector<pcl::PointIndices> cluster_indices, \
        const pcl::PointCloud<pcl::PointXYZ> cloud_filtered, \
        const sensor_msgs::PointCloud2 input);

    pcl::PointXYZI LPF_pos(std::vector<pcl::PointXYZI> centroids, int n);

    pcl::PointXYZI IHGP_fixed_pos(std::vector<pcl::PointXYZI> centroids, int n);

    pcl::PointXYZI IHGP_fixed_vel(std::vector<pcl::PointXYZI> centroids, int n);

    pcl::PointXYZI IHGP_nonfixed(std::vector<pcl::PointXYZI> centroids, int n);
};
