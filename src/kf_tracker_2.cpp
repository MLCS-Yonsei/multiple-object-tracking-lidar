#include "kf_tracker_2/kf_tracker_2.h"

/* TODO
2. Static obstacle pointcloud removal 이전에 예외처리 추가
(slam이 충분하지 않아서 dynamic까지 전부 지워버리면 에러나는듯?)
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

        // debug for pcl
        pc1 = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud1", 1);
        pc2 = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud2", 1);
        pc3 = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud3", 1);

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
        s_1 = clock();

        // Process the point cloud 
        // change PointCloud data type (ros sensor_msgs to pcl_Pointcloud)
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg (*input, input_cloud);

        // Voxel Down sampling 
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        vg.setInputCloud (input_cloud.makeShared());
        vg.setLeafSize (1*VoxelLeafSize_, 1*VoxelLeafSize_, 20*VoxelLeafSize_); // Leaf size 0.1m
        vg.filter (cloud_1);

        // 2D Projection
        for(int l=0; l!=cloud_1.points.size(); l++)
        {
            cloud_1.points[l].z=0.0;
        }

        // Voxel Down sampling
        pcl::VoxelGrid<pcl::PointXYZ> vg2;
        pcl::PointCloud<pcl::PointXYZ> cloud_2;
        vg2.setInputCloud (cloud_1.makeShared());
        vg2.setLeafSize (0.5*VoxelLeafSize_, 0.5*VoxelLeafSize_, 0.1*VoxelLeafSize_); // Leaf size 0.1m
        vg2.filter (cloud_2);

        // Remove static obstacles from occupied grid map msg
        pcl::PointCloud<pcl::PointXYZ> cloud_3;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_3 = removeStatic(cloud_2, cloud_3);
        *cloud_filtered = cloud_3;

        sensor_msgs::PointCloud2 cloud_filtered_ros;
        pcl::toROSMsg(*cloud_filtered, cloud_filtered_ros);
        cloud_filtered_ros.header.frame_id = "map";
        pc1.publish(cloud_filtered_ros);

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

        // Predict obstacle center with circumcenter method
        pcl::PointXYZI centroid;
        centroid = getCentroid(cluster_indices, *cloud_filtered, *input);     

        /* Predict with GP 
        predicted_centroid: i, predicted centroid from GP
        centroids: (i-10)~(i), predicted centroid stack from GP
        centroid: i, observed centroid */
        pcl::PointXYZI predicted_centroid; 
        predicted_centroid = GP(centroids, centroid);

        if (centroids.size() >= 10) // centroids array update
        {
            centroids.erase(centroids.begin());
        }
        centroids.push_back(centroid);

        if (predicted_centroids.size() >= 2) // centroids array(from GP) update
        {
            predicted_centroids.erase(centroids.begin());
        }
        predicted_centroids.push_back(predicted_centroid); 

        e_1 = clock();
        cout<<input->header.stamp.toSec()<<","<<centroid.x<<","<<centroid.y<<","<<((e_1-s_1)*(1e-3))<<endl;

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
    costmap_converter::ObstacleArrayMsg obstacle_array;
    costmap_converter::ObstacleMsg obstacle;

    // ObstacleArray header
    obstacle_array.header.stamp = ros::Time::now();
    obstacle_array.header.frame_id = "odom";

    obstacle_array.obstacles[0].id = 1;
    // obstacle_array.obstacles[0].radius = ; //pointcloud range
    obstacle_array.obstacles[0].header.stamp = ros::Time::now();

    // velocity
    float dx; float dy;
    float dt = predicted_centroids[1].intensity - predicted_centroids[1].intensity;
    dx = (predicted_centroids[1].x - predicted_centroids[0].x)/dt;
    dy = (predicted_centroids[1].y - predicted_centroids[0].y)/dt;

    obstacle_array.obstacles[0].velocities.twist.linear.x = dx;
    obstacle_array.obstacles[0].velocities.twist.linear.y = dy;
    obstacle_array.obstacles[0].velocities.twist.linear.z = 0;
    obstacle_array.obstacles[0].velocities.twist.angular.x = 0;
    obstacle_array.obstacles[0].velocities.twist.angular.y = 0;
    obstacle_array.obstacles[0].velocities.twist.angular.z = 0;

    obstacle_array.obstacles[0].velocities.covariance[0] = .1;
    obstacle_array.obstacles[0].velocities.covariance[7] = .1;
    obstacle_array.obstacles[0].velocities.covariance[14] = 1e9;
    obstacle_array.obstacles[0].velocities.covariance[21] = 1e9;
    obstacle_array.obstacles[0].velocities.covariance[28] = 1e9;
    obstacle_array.obstacles[0].velocities.covariance[35] = .1;

    // orientation
    // float yaw;
    // yaw = cmath::atan2(dy, dx);
    // q = tf.transformations.quaternion_from_euler(0,0,yaw);
    // obstacle_array.obstacles[0].orientation = Quaternion(*q);

    // Polgon of obstacle
    // geometry_msgs::Point32[] points;
    // obstacle_array.obstacles[0].polygon.points = points;
    // obstacle_array.obstacles[0].polygon.points[0].x = 0;
    // obstacle_array.obstacles[0].polygon.points[0].y = 0;
    // obstacle_array.obstacles[0].polygon.points[0].z = 0;

    // if (dy >= 0)
    // {
    //     obstacle_array.obstacles[0].polygon.points[0].y = y_0 + (dy*t)%range_y;
    // }
    // else
    // {
    //     obstacle_array.obstacles[0].polygon.points[0].y = y_0 + (dy*t)%range_y - range_y;
    // }

    // obstacle_pub.publish(obstacle_array);
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
    float pos_x = map_copy.info.origin.position.x; // origin of map
    float pos_y = map_copy.info.origin.position.y;

    /* Make lists of occupied grids coordinate */
    std::vector<float> x_min; // lower boundary x of occupied grid
    std::vector<float> x_max; // higher boundary x of occupied grid
    std::vector<float> y_min; // lower boundary y of occupied grid
    std::vector<float> y_max; // higher boundary y of occupied grid
    int count_occupied = 0; // number of occupied
    float clearance = resolution * 0.5; // clearance of occupied grid for pointcloud pose error

    // get coordinate range of occupied cell from map msg 
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

    // select pointclouds not above occupied cell(static cell)
    for (const auto& point: input_cloud.points)
    {
        for (int k=0; k<count_occupied; k++) 
        {
            if (x_min[k] < point.x && point.x < x_max[k] && \
            y_min[k] < point.y && point.y < y_max[k])
            {
                break;
            }
            else 
            {
                if (k==count_occupied-1) // leave only points that are not in the occupied range
                {
                    cloud_pre_process.push_back(point);   
                }
            }
        }
    }

    return cloud_pre_process;
}

pcl::PointXYZI ObstacleTrack::getCentroid(std::vector<pcl::PointIndices> cluster_indices, const pcl::PointCloud<pcl::PointXYZ> cloud_filtered, const sensor_msgs::PointCloud2 input)
{
        // To separate each cluster out of the vector<PointIndices> we have to 
        // iterate through cluster_indices, create a new PointCloud for each 
        // entry and write all points of the current cluster in the PointCloud. 
        std::vector<pcl::PointIndices>::const_iterator it;
        std::vector<int>::const_iterator pit;

        pcl::PointXYZI centroid; // (t) timestep cluster centroid (얘도 원래 vector나 스마트포인터로 해야함)
        it = cluster_indices.begin(); // 원래 object가 여러개 잡혀서 iterator로 돌려야하나, 현재는 하나만 찾도록 해둠. 원래 for문이 들어갈 자리라 중괄호 남겨둠
        {
            Vector3d Pi; 
            Vector3d Pj; 
            Vector3d Pk; 

            Vector3d Vij; // vector between Pi, Pj
            Vector3d Po(0,0,0); // origin vector

            // 1. get Pi, Pj (First, Second Point)
            // get Vij norm and select vector that maximizes norm
            float dist_max=-1;
            for(int i=0; i!=it->indices.size(); i++)
            {
                for(int j=i+1; j!=it->indices.size(); j++)
                {
                    float dist;     

                    Vector3d P1; 
                    Vector3d P2;     
                    pit = it->indices.begin()+i;              
                    P1(0) = cloud_filtered.points[*pit].x;
                    P1(1) = cloud_filtered.points[*pit].y;
                    P1(2) = cloud_filtered.points[*pit].z;
                    pit = it->indices.begin()+j;
                    P2(0) = cloud_filtered.points[*pit].x;
                    P2(1) = cloud_filtered.points[*pit].y;
                    P2(2) = cloud_filtered.points[*pit].z;

                    dist = euc_dist(P1, P2);
                    if (dist > dist_max) 
                    {
                        Pi = P1; 
                        Pj = P2;
                        Vij(0) = (P2(1)-P1(1))/(P2(0)-P1(0));
                        Vij(1) = -1;
                        Vij(2) = Vij(0)*(-P1(0))+P1(1);
                        dist_max = dist;
                    }
                }
            }

            // 2. get Pk (third Point)
            dist_max = -1; // initialize dist_max 
            for(int k=0; k!=it->indices.size(); k++)
            {
                float dist;

                Vector3d P3;
                pit = it->indices.begin()+k;
                P3(0) = cloud_filtered.points[*pit].x;
                P3(1) = cloud_filtered.points[*pit].y;
                P3(2) = cloud_filtered.points[*pit].z;

                // Euclidean distance between point and line
                dist = std::abs(Vij(0)*P3(0) + Vij(1)*P3(1) + Vij(2))/std::sqrt(Vij(0)*Vij(0) + Vij(1)*Vij(1));
                if (dist > dist_max)
                {
                    if(Pj==P3 || Pi==P3)
                    {
                        continue;
                    }
                    Pk = P3;
                    dist_max = dist;
                }
            }

            // 3. circumcenter coordinates from cross and dot products
            float alpha;
            float beta;
            float gamma;

            Vector3d Pi_j = Pi - Pj;
            Vector3d Pj_k = Pj - Pk;
            Vector3d Pk_i = Pk - Pi;

            float radius;
            radius = 0.5*(euc_dist(Po, Pi_j)*euc_dist(Po, Pj_k)*euc_dist(Po, Pk_i)) \
                    /euc_dist(Po, Pi_j.cross(Pj_k));

            alpha = 0.5 * std::pow(euc_dist(Po, Pj_k), 2.0) * Pi_j.dot(-Pk_i) \
                    /std::pow(euc_dist(Po, Pi_j.cross(Pj_k)), 2.0);
            beta = 0.5 * std::pow(euc_dist(Po, -Pk_i), 2.0) * -Pi_j.dot(Pj_k) \
                    /std::pow(euc_dist(Po, Pi_j.cross(Pj_k)), 2.0); 
            gamma = 0.5 * std::pow(euc_dist(Po, Pi_j), 2.0) * Pk_i.dot(-Pj_k) \
                    /std::pow(euc_dist(Po, Pi_j.cross(Pj_k)), 2.0); 

            Vector3d Pcentroid;
            Pcentroid = alpha*Pi + beta*Pj + gamma*Pk;
            centroid.x = Pcentroid(0);
            centroid.y = Pcentroid(1);
            centroid.z = 0.0;           
            centroid.intensity=input.header.stamp.toSec(); // used intensity slot(float) for time with GP
        } 

        return centroid;
} 

pcl::PointXYZI ObstacleTrack::GP(std::vector<pcl::PointXYZI> predicted_centroids, pcl::PointXYZI centroid)
{
    pcl::PointXYZI temp;
    return centroid;
}

float ObstacleTrack::euc_dist(Vector3d P1, Vector3d P2)
{
    return std::sqrt((P1(0)-P2(0))*(P1(0)-P2(0)) + (P1(1)-P2(1))*(P1(1)-P2(1)) + (P1(2)-P2(2))*(P1(2)-P2(2)));
}
