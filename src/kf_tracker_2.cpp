#include "kf_tracker_2/kf_tracker_2.h"


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
        obstacle_pub = nh_.advertise<costmap_converter::ObstacleArrayMsg> ("move_base/TebLocalPlannerROS/obstacles",1); // the state of objects (pos and vel)
        // objID_pub = nh_.advertise<std_msgs::Int32MultiArray>("obj_id", 1); // the objID of objects
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("tracker_viz", 1); // rviz visualization

        pc1 = nh_.advertise<sensor_msgs::PointCloud2>("pc1",1);
        pc2 = nh_.advertise<sensor_msgs::PointCloud2>("pc2",1);
        pc3 = nh_.advertise<sensor_msgs::PointCloud2>("pc3",1);

        // Initialize Subscriber for input Pointcloud2 
        map_sub = nh_.subscribe("/map", 1, &ObstacleTrack::mapCallback, this);
        input_sub = nh_.subscribe("input_pointcloud", 10, &ObstacleTrack::cloudCallback, this);

        time_init = ros::Time::now().toSec(); // for real world test
        
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

    // log scale hyperparameter
    // nh_.param<double>("/kf_tracker_2/smooth_Sigma2", smooth_Sigma2, 0.5);
    // nh_.param<double>("/kf_tracker_2/smooth_MagnSigma2", smooth_MagnSigma2, 1.0);
    // nh_.param<double>("/kf_tracker_2/smooth_LengthScale", smooth_LengthScale, 1.1);

    nh_.param<double>("/kf_tracker_2/logSigma2_x", logSigma2_x_, -5.5); // measurement noise
    nh_.param<double>("/kf_tracker_2/logMagnSigma2_x", logMagnSigma2_x_, -3.5);
    nh_.param<double>("/kf_tracker_2/logLengthScale_x", logLengthScale_x_, 0.75);

    nh_.param<double>("/kf_tracker_2/logSigma2_y", logSigma2_y_, -5.5);
    nh_.param<double>("/kf_tracker_2/logMagnSigma2_y", logMagnSigma2_y_, -3.5);
    nh_.param<double>("/kf_tracker_2/logLengthScale_y", logLengthScale_y_, 0.75);

    nh_.param<int>("/kf_tracker_2/data_length", data_length, 10);
    nh_.param<bool>("/kf_tracker_2/param_fix", param_fix, true);
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
        if (input->header.stamp.toSec() < 1.0e9)
        {
            time_init = 0;
        }
        if (input->header.stamp.toSec() - time_init < 0 && t_init==false)
        {
            time_init = input->header.stamp.toSec();
            t_init = true;
        }
        
        // Process the point cloud 
        // change PointCloud data type (ros sensor_msgs to pcl_Pointcloud)
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg (*input, input_cloud);

        // filter pointcloud only z>0 and update to z=0
        pcl::PointCloud<pcl::PointXYZ> cloud_0;
        for (const auto& point: input_cloud)
        {
            if(point.z > 0.0)
            {
                cloud_0.push_back(point);
                cloud_0.back().z = 0;
            }
        }

        // Voxel Down sampling 
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        vg.setInputCloud (cloud_0.makeShared());
        vg.setLeafSize (1*VoxelLeafSize_, 1*VoxelLeafSize_, 20*VoxelLeafSize_); // Leaf size 0.1m
        vg.filter (cloud_1);

        // Remove static obstacles from occupied grid map msg
        pcl::PointCloud<pcl::PointXYZ> cloud_3;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_3 = removeStatic(cloud_1, cloud_3);
        *cloud_filtered = cloud_3;

        // exit callback if no obstacles
        if (cloud_filtered->empty())
        {
            ROS_INFO("No obstacles around");
            return;
        }

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

        if (cluster_indices.empty())
            return;

        // Predict obstacle center with circumcenter method
        pcl::PointXYZI centroid;
        centroid = getCentroid(cluster_indices, *cloud_filtered, *input);     

        if (centroids.size() >= data_length) // centroids array update
        {
            firstFrame = false;
            centroids.erase(centroids.begin());

            float dt_sum;
            for(int i=0; i!=centroids.size(); i++)
            {
                dt_sum += centroids[i+1].intensity - centroids[i].intensity;
            }
            dt_gp = dt_sum/(centroids.size());

            if (param_fix == true) 
            {
                // Set hyperparameters
                model_x.setSigma2(exp(logSigma2_x_));
                model_x.setMagnSigma2(exp(logMagnSigma2_x_)); 
                model_x.setLengthScale(exp(logLengthScale_x_));

                model_y.setSigma2(exp(logSigma2_y_));
                model_y.setMagnSigma2(exp(logMagnSigma2_y_)); 
                model_y.setLengthScale(exp(logLengthScale_y_)); 

                gp_x.init_InfiniteHorizonGP(dt_gp,model_x.getF(),model_x.getH(),model_x.getPinf(),model_x.getR(),model_x.getdF(),model_x.getdPinf(),model_x.getdR());
                gp_y.init_InfiniteHorizonGP(dt_gp,model_y.getF(),model_y.getH(),model_y.getPinf(),model_y.getR(),model_y.getdF(),model_y.getdPinf(),model_y.getdR());
            }

            if (param_fix == false)
            {
                // Set hyperparameters
                model_x.setSigma2(0.1);
                model_x.setMagnSigma2(0.1); 
                model_x.setLengthScale(0.1);

                model_y.setSigma2(0.1);
                model_y.setMagnSigma2(0.1); 
                model_y.setLengthScale(0.1); 
            }
        }
        centroids.push_back(centroid);
    }
 
    else
    { 
        // s_1 = clock();
        // Process the point cloud 
        // change PointCloud data type (ros sensor_msgs to pcl_Pointcloud)
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg (*input, input_cloud);

        // filter pointcloud only z>0 and update to z==0
        pcl::PointCloud<pcl::PointXYZ> cloud_0;
        for (const auto& point: input_cloud)
        {
            if(point.z > 0.0)
            {
                cloud_0.push_back(point);
                cloud_0.back().z = 0;
            }
        }

        // Voxel Down sampling 
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        vg.setInputCloud (cloud_0.makeShared());
        vg.setLeafSize (1*VoxelLeafSize_, 1*VoxelLeafSize_, 20*VoxelLeafSize_); // Leaf size 0.1m
        vg.filter (cloud_1);

        // publish pointcloud for debuging 
        sensor_msgs::PointCloud2 cloud_debug;
        pcl::toROSMsg(cloud_1, cloud_debug);
        cloud_debug.header.frame_id = "velodyne";
        pc1.publish(cloud_debug);

        // Remove static obstacles from occupied grid map msg
        pcl::PointCloud<pcl::PointXYZ> cloud_3;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_3 = removeStatic(cloud_1, cloud_3);
        *cloud_filtered = cloud_3;

        // publish pointcloud for debuging 
        sensor_msgs::PointCloud2 cloud_debug2;
        pcl::toROSMsg(cloud_3, cloud_debug2);
        cloud_debug2.header.frame_id = "velodyne";
        pc2.publish(cloud_debug2);


        // exit callback if no obstacles
        if (cloud_filtered->empty())
        {
            ROS_INFO("No obstacles around");
            return;
        }

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
        if (cluster_indices.empty())
            return;
        pcl::PointXYZI centroid;
        centroid = getCentroid(cluster_indices, *cloud_filtered, *input);     

        // Change map coordinate centroid to base_link coordinate centroid 
        // centroid = getRelativeCentroid(centroid);
        
        // centroids array update
        centroids.erase(centroids.begin());
        centroids.push_back(centroid);

        // Predict with GP 
        pcl::PointXYZI predicted_centroid;
        pcl::PointXYZI predicted_velocity;
        if (param_fix == true) 
        { 
            predicted_centroid = IHGP_fixed(centroids, "pos"); 
            predicted_velocity = IHGP_fixed(centroids, "vel");
        }
        //else if(param_fix == false) { predicted_centroid = IHGP_nonfixed(centroids); }

        // e_1 = clock(); 
        // cout<<(centroids[data_length-1].intensity-time_init)<<"," \
        //     <<centroids[data_length-1].x<<","<<centroids[data_length-1].y<<"," \
        //     <<predicted_centroid.x<<","<<predicted_centroid.y<<"," \
        //     <<predicted_velocity.x<<","<<predicted_velocity.y<<"," \
        //     <<((e_1-s_1)*(1e-3))<<endl;  

        // Publish state & rviz marker 
        publishObstacles(predicted_centroid, predicted_velocity, input);
        publishMarkers(predicted_centroid);
    } 
} 

void ObstacleTrack::mapCallback(const nav_msgs::OccupancyGrid& map_msg)
{
    if (map==false) 
    {
        map_copy = map_msg;
        map=true;
    }
    else 
    {
        ;
    }    
}

void ObstacleTrack::publishObstacles(pcl::PointXYZI position, pcl::PointXYZI velocity, const sensor_msgs::PointCloud2ConstPtr& input)
{
    costmap_converter::ObstacleArrayMsg obstacle_array;
    costmap_converter::ObstacleMsg obstacle;

    // ObstacleArray header
    obstacle_array.header.stamp = input->header.stamp;
    obstacle_array.header.frame_id = "map";
    
    obstacle_array.obstacles.push_back(costmap_converter::ObstacleMsg());
    obstacle_array.obstacles[0].id = 99;
    obstacle_array.obstacles[0].radius = obstacle_radius; //pointcloud range
    obstacle_array.obstacles[0].header.stamp = input->header.stamp;
    obstacle_array.obstacles[0].header.frame_id = "map";

    // velocity
    obstacle_array.obstacles[0].velocities.twist.linear.x = velocity.x;
    obstacle_array.obstacles[0].velocities.twist.linear.y = velocity.y;
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
    float yaw;
    yaw = atan2(velocity.y, velocity.x);
    tf2::Quaternion Quaternion;
    Quaternion.setRPY(0,0,yaw);
    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(Quaternion);
    obstacle_array.obstacles[0].orientation = quat_msg;

    // Polygon of obstacle
    std::vector<geometry_msgs::Point32> _points(1);
    obstacle_array.obstacles[0].polygon.points = _points;
    obstacle_array.obstacles[0].polygon.points[0].x = position.x;
    obstacle_array.obstacles[0].polygon.points[0].y = position.y;
    obstacle_array.obstacles[0].polygon.points[0].z = 0;


    obstacle_pub.publish(obstacle_array);

    //Completed: move_base의 teb planner에게 obstacle 전달
    //TODO: navigation 돌리면서 local costmap과 path 확인해보기
}

void ObstacleTrack::publishMarkers(pcl::PointXYZI predicted_centroid)
{
    visualization_msgs::MarkerArray obstacleMarkers;
    // for (int i=0;i<predicted_centroid.size();i++)
    int i=0;
    {
        visualization_msgs::Marker m;

        m.id=i;
        m.header.frame_id="/map";
        m.type=visualization_msgs::Marker::CYLINDER;
        m.action=visualization_msgs::Marker::ADD;
        m.scale.x=2*obstacle_radius;         m.scale.y=2*obstacle_radius;         m.scale.z=0.05;
        
        m.color.r=0.75;
        m.color.g=0.0;
        m.color.b=0.0;
        m.color.a=1.0;

        m.pose.position.x=predicted_centroid.x;
        m.pose.position.y=predicted_centroid.y;
        m.pose.position.z=0.0;  

        obstacleMarkers.markers.push_back(m);
    }
    marker_pub.publish(obstacleMarkers);
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
    float clearance = resolution * 1.0; // clearance of occupied grid for pointcloud pose error

    // get coordinate range of occupied cell from map msg 
    for (int i=0; i<map_copy.data.size(); i++) 
    {
        int cell = map_copy.data[i]; 
        if (cell > 50 || cell == -1) // all cell with occupancy larger than 50.0
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
            float A = Pj(0) - Pi(0);
            float B = Pj(1) - Pi(1);
            float C = Pk(0) - Pi(0);
            float D = Pk(1) - Pi(1);
            float E = A * (Pi(0) + Pj(0)) + B * (Pi(1) + Pj(1));
            float F = C * (Pi(0) + Pk(0)) + D * (Pi(1) + Pk(1));
            float G = 2.0 * (A * (Pk(1) - Pj(1)) - B * (Pk(0) - Pj(0)));
            if(G==0)
            {
                centroid.x = Pi(0);
                centroid.y = Pi(1);
                centroid.z = 0.0;
                centroid.intensity=input.header.stamp.toSec() - time_init; // used intensity slot(float) for time with GP
            }
            else
            {
                centroid.x = (D * E - B * F) / G;
                centroid.y = (A * F - C * E) / G;
                centroid.z = 0.0;
                centroid.intensity=input.header.stamp.toSec() - time_init; // used intensity slot(float) for time with GP
            }

            // set radius for publishObstacles
            Vector3d V_centroid(centroid.x, centroid.y, centroid.z);
            obstacle_radius = euc_dist(V_centroid, Pj);
            if (obstacle_radius > 0.5) // obstacle radius constraint
            {
                obstacle_radius = 0.49;
            }
            
            /*
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
            */
        } 



        return centroid;
} 

pcl::PointXYZI ObstacleTrack::IHGP_fixed(std::vector<pcl::PointXYZI> centroids, string variable)
{
    gp_x.init_step();
    gp_y.init_step();

    // Data pre-precessing
    double mean_x;
    double mean_y;
    std::vector<double> vx_raw; 
    std::vector<double> vy_raw; 

    int gp_data_len;
    if(variable == "vel")
    {
        gp_data_len = data_length-2;
              
        for (int k=0; k<=gp_data_len; k++)
        {
            double vel = (centroids[k+1].x - centroids[k].x)/dt_gp;
            vx_raw.push_back(vel);
            mean_x += vel;

            vel = (centroids[k+1].y - centroids[k].y)/dt_gp;
            vy_raw.push_back(vel);
            mean_y += vel;
        }
        mean_x = mean_x/(gp_data_len+1);
        mean_y = mean_y/(gp_data_len+1);
    }
    else if(variable == "pos")
    {
        gp_data_len = data_length-1;

        mean_x = centroids[gp_data_len].x;
        mean_y = centroids[gp_data_len].y;
    }
    
    // Loop through data
    if(variable == "vel")
    {
        for (int k=0; k<=gp_data_len; k++)
        {
            gp_x.update(vx_raw[k] - mean_x);
            gp_y.update(vy_raw[k] - mean_y);
        }
    }
    else if(variable == "pos")
    {
        for (int k=0; k<=gp_data_len; k++)
        {
            gp_x.update(centroids[k].x - mean_x);
            gp_y.update(centroids[k].y - mean_y);
        }
    }

    // Pull out the marginal mean and variance estimates
    Eft_x = gp_x.getEft();
    Eft_y = gp_y.getEft();

    // make PCL::PointXYZI data
    pcl::PointXYZI predicted_centroid;
    predicted_centroid.x = Eft_x[gp_data_len] + mean_x;
    predicted_centroid.y = Eft_y[gp_data_len] + mean_y;
    predicted_centroid.z = 0.0;
    predicted_centroid.intensity = centroids[data_length-1].intensity;
  
    return predicted_centroid;
}

pcl::PointXYZI ObstacleTrack::IHGP_nonfixed(std::vector<pcl::PointXYZI> centroids)
{
    // // Do inference
    // gp_x.init_InfiniteHorizonGP(dt_gp,model_x.getF(),model_x.getH(),model_x.getPinf(),model_x.getR(),model_x.getdF(),model_x.getdPinf(),model_x.getdR());
    // gp_y.init_InfiniteHorizonGP(dt_gp,model_y.getF(),model_y.getH(),model_y.getPinf(),model_y.getR(),model_y.getdF(),model_y.getdPinf(),model_y.getdR());

    // // Loop through data
    // for (int k=0; k<data_length; k++)
    // {
    //     gp_x.update(centroids[k].x - calibration_x);
    //     gp_y.update(centroids[k].y - calibration_y);
    // }
    
    // // Pull out the gradient (account for log-transformation)
    // double logSigma2_x = log(model_x.getSigma2());
    // double logMagnSigma2_x = log(model_x.getMagnSigma2());
    // double logLengthScale_x = log(model_x.getLengthScale());
    // double dLikdlogSigma2_x = model_x.getSigma2() * gp_x.getLikDeriv()(0);
    // double dLikdlogMagnSigma2_x = model_x.getMagnSigma2() * gp_x.getLikDeriv()(1);
    // double dLikdlogLengthScale_x = model_x.getLengthScale() * gp_x.getLikDeriv()(2);

    // double logSigma2_y = log(model_y.getSigma2());
    // double logMagnSigma2_y = log(model_y.getMagnSigma2());
    // double logLengthScale_y = log(model_y.getLengthScale());
    // double dLikdlogSigma2_y = model_y.getSigma2() * gp_y.getLikDeriv()(0);
    // double dLikdlogMagnSigma2_y = model_y.getMagnSigma2() * gp_y.getLikDeriv()(1);
    // double dLikdlogLengthScale_y = model_y.getLengthScale() * gp_y.getLikDeriv()(2);
    
    // // Do the gradient descent step
    // // logSigma2_x = logSigma2_x - 0.1*dLikdlogSigma2_x;
    // logMagnSigma2_x = logMagnSigma2_x - 0.1*dLikdlogMagnSigma2_x;
    // logLengthScale_x = logLengthScale_x - 0.01*dLikdlogLengthScale_x;

    // // logSigma2_y = logSigma2_y - 0.1*dLikdlogSigma2_y;
    // logMagnSigma2_y = logMagnSigma2_y - 0.1*dLikdlogMagnSigma2_y;
    // logLengthScale_y = logLengthScale_y - 0.01*dLikdlogLengthScale_y;
    
    // // Introduce contraints to keep the behavior better in control
    // // if (logSigma2_x < -10) { logSigma2_x = -10; } else if (logSigma2_x > 10) { logSigma2_x = 10; }
    // if (logMagnSigma2_x < -10) { logMagnSigma2_x = -10; } else if (logMagnSigma2_x > 10) { logMagnSigma2_x = 10; }
    // if (logLengthScale_x < -10) { logLengthScale_x = -10; } else if (logLengthScale_x > 10) { logLengthScale_x = 10; }

    // // if (logSigma2_y < -10) { logSigma2_y = -10; } else if (logSigma2_y > 10) { logSigma2_y = 10; }
    // if (logMagnSigma2_y < -10) { logMagnSigma2_y = -10; } else if (logMagnSigma2_y > 10) { logMagnSigma2_y = 10; }
    // if (logLengthScale_y < -10) { logLengthScale_y = -10; } else if (logLengthScale_y > 10) { logLengthScale_y = 10; }

    // // Update the model
    // // model_x.setSigma2(exp(logSigma2_x));
    // model_x.setMagnSigma2(exp(logMagnSigma2_x));
    // model_x.setLengthScale(exp(logLengthScale_x));

    // // model_y.setSigma2(exp(logSigma2_y));
    // model_y.setMagnSigma2(exp(logMagnSigma2_y));
    // model_y.setLengthScale(exp(logLengthScale_y));
    
    // // Check if this went bad and re-initialize
    // if (isnan(model_x.getMagnSigma2()) | isnan(model_x.getLengthScale()) |
    //     isinf(model_x.getMagnSigma2()) | isinf(model_x.getLengthScale())) {
    //     model_x.setMagnSigma2(1.0);
    //     model_x.setLengthScale(1.0);
    //     cout<<"[error] Bad parameters."<<endl;
    // }
    //     if (isnan(model_y.getMagnSigma2()) | isnan(model_y.getLengthScale()) |
    //     isinf(model_y.getMagnSigma2()) | isinf(model_y.getLengthScale())) {
    //     model_y.setMagnSigma2(1.0);
    //     model_y.setLengthScale(1.0);
    //     cout<<"[error] Bad parameters."<<endl;
    // }

    // // Pull out the marginal mean and variance estimates
    // Eft_x = gp_x.getEft();
    // Eft_y = gp_y.getEft();

    // // vel_x = (Eft_x[data_length-3] - 4*Eft_x[data_length-2] + 3*Eft_x[data_length-1])/(2*dt_gp);
    // // vel_y = (Eft_y[data_length-3] - 4*Eft_y[data_length-2] + 3*Eft_y[data_length-1])/(2*dt_gp);
    // vel_x = (Eft_x[data_length-1]-Eft_x[data_length-2])/dt_gp;
    // vel_y = (Eft_y[data_length-1]-Eft_y[data_length-2])/dt_gp;

    // pcl::PointXYZI predicted_centroid;
    // predicted_centroid.x = Eft_x[data_length-1] + calibration_x;
    // predicted_centroid.y = Eft_y[data_length-1] + calibration_y;
    // predicted_centroid.z = 0.0;
    // predicted_centroid.intensity = centroids[data_length-1].intensity;

    // calibration_x = Eft_x[data_length-1] + calibration_x;
    // calibration_y = Eft_y[data_length-1] + calibration_y;
    
    // return predicted_centroid;
    ;
}

float ObstacleTrack::euc_dist(Vector3d P1, Vector3d P2)
{
    return std::sqrt((P1(0)-P2(0))*(P1(0)-P2(0)) + (P1(1)-P2(1))*(P1(1)-P2(1)) + (P1(2)-P2(2))*(P1(2)-P2(2)));
}

pcl::PointXYZI ObstacleTrack::getRelativeCentroid(pcl::PointXYZI centroid)
{
    //set object's tf using map coordinate
    transform.setOrigin( tf::Vector3(centroid.x, centroid.y, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    tf_broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/object"));

    //get transform between base_link and object
    try
    {
        tf_listener.waitForTransform("/base_link", "/object", ros::Time(0), ros::Duration(3.0));
        tf_listener.lookupTransform("/base_link", "/object", ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // get relative_centroid using transform
    pcl::PointXYZI relative_centroid;
    
    relative_centroid.x = stamped_transform.getOrigin().x();
    relative_centroid.y = stamped_transform.getOrigin().y();
    relative_centroid.z = 0.0;
    relative_centroid.intensity = centroid.intensity;

    return relative_centroid;
}
