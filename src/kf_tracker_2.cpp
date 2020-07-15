#include "kf_tracker_2/kf_tracker_2.h"

/* TODO
1. GP parameter setup
2. Static obstacle pointcloud removal 이전에 예외처리 추가
(slam이 충분하지 않아서 dynamic까지 전부 지워버리면 에러남)
3. GP 추가
*/


ObstacleTrack::ObstacleTrack()
{
    ;
}

ObstacleTrack::~ObstacleTrack()
{
    nh_.deleteParam("obstacle_num");
    nh_.deleteParam("cluster_tolerance");
    nh_.deleteParam("min_cluster_size");
    nh_.deleteParam("max_cluster_size");
    nh_.deleteParam("voxel_leaf_size");
}

bool ObstacleTrack::initialize()
{ 
    if (ros::ok())
    {
        // update pcl, KF parameters
        updateParam();

        // Create a ROS Publishers 
        obstacle_pub = nh_.advertise<costmap_converter::ObstacleArrayMsg> ("obstacle",1); // the state of objects (pos and vel)
        debug_pub = nh_.advertise<sensor_msgs::PointCloud2>("debug_msg", 1); // debugging
        // objID_pub = nh_.advertise<std_msgs::Int32MultiArray>("obj_id", 1); // the objID of objects
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("viz", 1); // rviz visualization

        // Initialize Subscriber for input Pointcloud2  
        input_sub = nh_.subscribe("input_pointcloud", 1, &ObstacleTrack::cloudCallback, this);
        map_sub = nh_.subscribe("/map", 1, &ObstacleTrack::mapCallback, this);
        
        ROS_INFO_STREAM("ObstacleTrack Initialized");
        return true;
    }
    else
    {
        return false;
    }
}

void ObstacleTrack::updateParam()
{
    nh_.param<int>("/kf_tracker_2/obstacle_num", obstacle_num_, 1); // # of maximum observable obstacle 
    nh_.param<float>("/kf_tracker_2/cluster_tolerance", ClusterTolerance_, 0.15); // pcl extraction tolerance
    nh_.param<int>("/kf_tracker_2/min_cluster_size", MinClusterSize_, 10);
    nh_.param<int>("/kf_tracker_2/max_cluster_size", MaxClusterSize_, 200);
    nh_.param<float>("/kf_tracker_2/voxel_leaf_size", VoxelLeafSize_, 0.05); // default is same with map resolution
}

void ObstacleTrack::spinNode()
{
  ros::spin();
}

void ObstacleTrack::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // If this is the first frame, initialize kalman filters for the clustered objects
    if (firstFrame)
    {   
        ;
    }
 
    else
    { 
        // Process the point cloud 
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg (*input, input_cloud);

        // Voxel Down sampling 
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        vg.setInputCloud (input_cloud.makeShared());
        vg.setLeafSize (VoxelLeafSize_, VoxelLeafSize_, VoxelLeafSize_); // Leaf size 10cm
        vg.filter (cloud_1);

        // Removing static obstacles
        pcl::PointCloud<pcl::PointXYZ> cloud_2;
        cloud_2 = removeStatic(cloud_1, cloud_2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_filtered = cloud_2;

        // Creating the KdTree from voxel point cloud 
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);

        // Here we are creating a vector of PointIndices, which contains the actual index
        // information in a vector<int>. The indices of each detected cluster are saved here.
        // Cluster_indices is a vector containing one instance of PointIndices for each detected 
        // cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (ClusterTolerance_);
        ec.setMinClusterSize (MinClusterSize_);
        ec.setMaxClusterSize (MaxClusterSize_);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);

        // Extract the clusters out of pc and save indices in cluster_indices.
        ec.extract (cluster_indices); // most of Runtime are used from this step.

        // To separate each cluster out of the vector<PointIndices> we have to 
        // iterate through cluster_indices, create a new PointCloud for each 
        // entry and write all points of the current cluster in the PointCloud. 

        // pcl::PointXYZ origin (0,0,0);
        // float mindist_this_cluster = 1000;
        // float dist_this_point = 1000;
        std::vector<pcl::PointIndices>::const_iterator it;
        std::vector<int>::const_iterator pit;

        // Cluster Centroid 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_vec; // Vector of cluster pointclouds 
        pcl::PointXYZI centroid; // (t) timestep cluster centroid

        it = cluster_indices.begin();
        {
            float x=0.0; float y=0.0;
            int numPts=0;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
            {
                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

                x+=cloud_filtered->points[*pit].x;
                y+=cloud_filtered->points[*pit].y;
                numPts++;

                //dist_this_point = pcl::geometry::distance(cloud_filtered->points[*pit], origin);
                //mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
            }
            centroid.x=x/numPts;
            centroid.y=y/numPts;
            centroid.z=0.0;
            centroid.intensity=input->header.stamp.toSec(); // used intensity slot(float) for time with GP
            cluster_vec = cloud_cluster;
        }

        // Ensure at least obstacle_num_ clusters exist to publish (later clusters may be empty) 
        // while (cluster_vec.size() < obstacle_num_)
        // {
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        //     empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
        //     cluster_vec = empty_cluster;
        // }
        // while (centroid.size() < obstacle_num_)
        // {
        //     pcl::PointXYZI centroid;
        //     centroid.x=0.0;
        //     centroid.y=0.0;
        //     centroid.z=0.0;
        //     centroid.intensity=input->header.stamp.sec + (1e-8)*input->header.stamp.nsec;
        // }

        /* Predict with GP 
        predicted_centroid: i, predicted centroid from GP
        predicted_centroids: (i-10)~(i), predicted centroid stack from GP
        centroids: i, observed centroid */
        pcl::PointXYZI predicted_centroid;
        predicted_centroid = GP(predicted_centroids, centroid);
               
        if (predicted_centroids.size() > 10)
        {
            predicted_centroids.erase(predicted_centroids.begin());
        }
        predicted_centroids.push_back(predicted_centroid);

        /* Publish state & rviz marker */
        // publishObstacles();
        // publishMarkers();
    }
} 

void ObstacleTrack::mapCallback(const nav_msgs::OccupancyGrid& map_msg)
{
    map_copy = map_msg;
}

void ObstacleTrack::publishObstacles(std::vector<pcl::PointXYZI> predicted_centroids)
{
    // costmap_converter::ObstacleArrayMsg obstacle_array;
    // costmap_converter::ObstacleMsg obstacle;

    // obstacle_array.header.stamp.secs = predicted_centroids[-1].header.stamp/1;
    // obstacle_array.header.stamp.nsecs = predicted_centroids[-1].header.stamp%1;
    // obstacle_array.header.frame_id = "odom";

    // // Add point obstacle
    // obstacle_array.obstacles.append(obstacle)
    // obstacle_array.obstacles[0].id = 1;
    // obstacle_array.obstacles[0].radius = pointcloud range;

    // geometry_msgs::Point32[] points;
    // obstacle_array.obstacles[0].polygon.points = points;
    // obstacle_array.obstacles[0].polygon.points[0].x = 0;
    // obstacle_array.obstacles[0].polygon.points[0].y = 0;
    // obstacle_array.obstacles[0].polygon.points[0].z = 0;
   
    // float dx; float dy;
    // float dt = predicted_centroids[-1].intensity - predicted_centroids[-2].intensity;
    // dx = (predicted_centroids[-1].x - predicted_centroids[-2].x)/dt;
    // dy = (predicted_centroids[-1].y - predicted_centroids[-2].y)/dt;

    // float yaw;
    // yaw = cmath::atan2(dy, dx);
    // q = tf.transformations.quaternion_from_euler(0,0,yaw);

    // obstacle orientation & velocity
    // obstacle_array.obstacles[0].orientation = Quaternion(*q);
    // obstacle_array.obstacles[0].velocities.twist.linear.x = dx;
    // obstacle_array.obstacles[0].velocities.twist.linear.y = dy;
    // obstacle_array.obstacles[0].velocities.twist.linear.z = 0;
    // obstacle_array.obstacles[0].velocities.twist.angular.x = 0;
    // obstacle_array.obstacles[0].velocities.twist.angular.y = 0;
    // obstacle_array.obstacles[0].velocities.twist.angular.z = 0;

    // if (dy >= 0)
    // {
    //     obstacle_array.obstacles[0].polygon.points[0].y = y_0 + (dy*t)%range_y;
    // }
    // else
    // {
    //     obstacle_array.obstacles[0].polygon.points[0].y = y_0 + (dy*t)%range_y - range_y;
    // }
    
    // obstacle_pub.publish(obstacle_array);
    ;
}

void ObstacleTrack::publishMarkers(std::vector<geometry_msgs::Point> KFpredictions, std::vector<geometry_msgs::Point> clusterCenters)
{
    visualization_msgs::MarkerArray clusterMarkers;
    for (int i=0;i<KFpredictions.size();i++)
    {
        visualization_msgs::Marker m;

        m.id=i;
        m.type=visualization_msgs::Marker::CUBE;
        m.header.frame_id="/map";
        m.scale.x=0.3;         m.scale.y=0.3;         m.scale.z=0.3;
        m.action=visualization_msgs::Marker::ADD;
        m.color.a=1.0;
        m.color.r=i%2?1:0;
        m.color.g=i%3?1:0;
        m.color.b=i%4?1:0;

        //geometry_msgs::Point clusterC(clusterCenters.at(objID[i]));
        geometry_msgs::Point clusterC(KFpredictions[i]);        
        m.pose.position.x=clusterC.x;
        m.pose.position.y=clusterC.y;
        m.pose.position.z=clusterC.z;

        clusterMarkers.markers.push_back(m);
    }
    // prevClusterCenters = clusterCenters;
    marker_pub.publish(clusterMarkers);
}

void ObstacleTrack::publishObjID()
{
    std_msgs::Int32MultiArray obj_id;
    obj_id.data.reserve(obstacle_num_);
    for(auto it=objID.begin(); it!=objID.end(); it++) 
    {
        obj_id.data.push_back(*it);
    }
    objID_pub.publish(obj_id);
}

pcl::PointCloud<pcl::PointXYZ> ObstacleTrack::removeStatic(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ> cloud_pre_process)
{
    int width_i = map_copy.info.width; // width (integer) for grid calculation
    float width = map_copy.info.width; // width (float) for division calculation
    float height = map_copy.info.height;
    float resolution = map_copy.info.resolution;
    float pos_x = map_copy.info.origin.position.x;
    float pos_y = map_copy.info.origin.position.y;

    /* Make lists of occupied grids coordinate */
    std::vector<float> x_min; // lower boundary x of occupied grid
    std::vector<float> x_max; // higher boundary x of occupied grid
    std::vector<float> y_min; // lower boundary y of occupied grid
    std::vector<float> y_max; // higher boundary y of occupied grid
    int count_occupied = 0;
    float clearance = resolution * 0.5;

    for (int i=0; i<map_copy.data.size(); i++) 
    {
        int cell = map_copy.data[i]; 
        if (cell > 60) // all cell with occupancy larger than 60.0
        { 
            x_min.push_back((i%width_i)*resolution + pos_x - clearance);
            x_max.push_back((i%width_i)*resolution  + pos_x + resolution + clearance);
            y_min.push_back((i/width_i)*resolution + pos_y - clearance);
            y_max.push_back((i/width_i)*resolution  + pos_y + resolution + clearance);
            count_occupied++;
        }
    }

    /* Removal pointclouds of occupied grids */
    for (int j=0; j<input_cloud.size(); j++) 
    {
        
        for (int k=0; k<count_occupied; k++) 
        {
            if (x_min[k] < input_cloud.points[j].x && input_cloud.points[j].x < x_max[k] && \
            y_min[k] < input_cloud.points[j].y && input_cloud.points[j].y < y_max[k])
            {
                break;
            }
            else 
            {
                if (k==count_occupied-1)
                {
                    cloud_pre_process.push_back(input_cloud.points[j]);   
                }
            }
        }
    }

    return cloud_pre_process;
}

pcl::PointXYZI ObstacleTrack::GP(std::vector<pcl::PointXYZI> predicted_centroids, pcl::PointXYZI centroid)
{
    pcl::PointXYZI temp;
    return centroid;
}