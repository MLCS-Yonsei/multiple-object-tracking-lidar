#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <time.h> // We don't need it for run. It's for Runtime check
#include <cmath>

// Filter
#include "kf_tracker_2/featureDetection.h"
#include "kf_tracker_2/CKalmanFilter.h"

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/video/tracking.hpp"

// ros msgs
#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

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
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
 
// visualization
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>

//*****************************************************************
using namespace std;
using namespace cv;

class ObstacleTrack
{
public:

    ObstacleTrack();

    ~ObstacleTrack();

    bool initialize();

    void updateParam();

    // ros::Publisher objState0_pub, objState1_pub, objState2_pub, objState3_pub, objState4_pub, objState5_pub; 
    
    // ros::Publisher cluster0_pub, cluster1_pub, cluster2_pub, cluster3_pub, cluster4_pub, cluster5_pub;
    // ros::Publisher cc_pos; // cluster centroid pose (non KF)
    // ros::Publisher cckf_pos; // cluster centroid pose (KF)
    ros::Publisher objID_pub; // objID (KF)
    ros::Publisher markerPub; // obstacle pose visualization (KF)
    ros::Subscriber input_sub;
    

private:

    ros::NodeHandle nh_;

    void spinNode();
    
    void clearDataMember();

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

    void publishMarkers(std::vector<geometry_msgs::Point> KFpredictions, std::vector<geometry_msgs::Point> clusterCenters);

    void publishObjID();

    void initKalmanFilters(const sensor_msgs::PointCloud2ConstPtr& input);

    void KFT(const std::vector<geometry_msgs::Point> cc);

    std::pair<int,int> findIndexOfMin(std::vector<std::vector<float>> distMat);

    // calculate euclidean distance of two points
    double euclidean_distance(geometry_msgs::Point& p1, geometry_msgs::Point& p2);


};
