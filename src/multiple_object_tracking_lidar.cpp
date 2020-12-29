#include "multiple_object_tracking_lidar/multiple_object_tracking_lidar.h"

//TODO: 
// - [x] fix private and public
// - [x] lower removeStatic() computing resource
// - [x] map orientation bug fix

// - [x] PCL Clustering memorized last step
// - [x] multi obstacle ID

// - [x] clean up codes (callback parts)
// - [x] add comments
// - [x] update READ.md

// - [x] add unregisterOldObstacle() method
// - [ ] change IHGP filter to LPF for position

// - [ ] test obstacle edge extraction (realworld Velodyne)
// - [ ] change multiple lidar merging method
// - [ ] solve Occlusion Problem

ObstacleTrack::ObstacleTrack()
{
    ;
}

ObstacleTrack::~ObstacleTrack()
{
    nh_.deleteParam("frequency");

    nh_.deleteParam("cluster_tolerance");
    nh_.deleteParam("min_cluster_size");
    nh_.deleteParam("max_cluster_size");
    nh_.deleteParam("voxel_leaf_size");

    nh_.deleteParam("static_tolarance");
    nh_.deleteParam("id_threshold");

    nh_.deleteParam("logSigma2_x");
    nh_.deleteParam("logMagnSigma2_x");
    nh_.deleteParam("logLengthScale_x");

    nh_.deleteParam("logSigma2_y");
    nh_.deleteParam("logMagnSigma2_y");
    nh_.deleteParam("logLengthScale_y");

    nh_.deleteParam("data_length");
    nh_.deleteParam("param_fix");
    
}

bool ObstacleTrack::initialize()
{ 
    if (ros::ok())
    {
        // update pcl, KF parameters
        updateParam();

        // Create a ROS Publishers 
        obstacle_pub = nh_.advertise<costmap_converter::ObstacleArrayMsg> ("move_base/TebLocalPlannerROS/obstacles", 10); // the state of objects (pos and vel)
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("tracker_viz", 10); // rviz visualization

        // pointcloud publisher for debugging
        pc1 = nh_.advertise<sensor_msgs::PointCloud2>("pc1",10);
        pc2 = nh_.advertise<sensor_msgs::PointCloud2>("pc2",10);
        pc3 = nh_.advertise<sensor_msgs::PointCloud2>("pc3",10);

        // Initialize Subscriber for input Pointcloud2 
        map_sub = nh_.subscribe("/map", 1, &ObstacleTrack::mapCallback, this);
        input_sub = nh_.subscribe("input_pointcloud", 1, &ObstacleTrack::cloudCallback, this);

        time_init = ros::Time::now().toSec(); // for real world test
        std::srand(5323); // rviz colorset random seed

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
    nh_.param<float>("/multiple_object_tracking_lidar/frequency", frequency, 10.0);

    nh_.param<float>("/multiple_object_tracking_lidar/cluster_tolerance", ClusterTolerance, 0.15); // pcl extraction tolerance
    nh_.param<int>("/multiple_object_tracking_lidar/min_cluster_size", MinClusterSize, 5);
    nh_.param<int>("/multiple_object_tracking_lidar/max_cluster_size", MaxClusterSize, 200);
    nh_.param<float>("/multiple_object_tracking_lidar/voxel_leaf_size", VoxelLeafSize, 0.05); // default is same with map resolution

    nh_.param<int>("/multiple_object_tracking_lidar/static_tolarance", tolarance, 2); 
    if (tolarance > 4) {tolarance = 4;} else if (tolarance < 0) {tolarance = 0;} // static_tolarance bounding
    nh_.param<float>("/multiple_object_tracking_lidar/id_threshold", id_thershold, 0.5);

    // log scale hyperparameter
    // nh_.param<double>("/multiple_object_tracking_lidar/smooth_Sigma2", smooth_Sigma2, 0.5);
    // nh_.param<double>("/multiple_object_tracking_lidar/smooth_MagnSigma2", smooth_MagnSigma2, 1.0);
    // nh_.param<double>("/multiple_object_tracking_lidar/smooth_LengthScale", smooth_LengthScale, 1.1);

    nh_.param<double>("/multiple_object_tracking_lidar/logSigma2_x", logSigma2_x, -5.5); // measurement noise
    nh_.param<double>("/multiple_object_tracking_lidar/logMagnSigma2_x", logMagnSigma2_x, -3.5);
    nh_.param<double>("/multiple_object_tracking_lidar/logLengthScale_x", logLengthScale_x, 0.75);

    nh_.param<double>("/multiple_object_tracking_lidar/logSigma2_y", logSigma2_y, -5.5);
    nh_.param<double>("/multiple_object_tracking_lidar/logMagnSigma2_y", logMagnSigma2_y, -3.5);
    nh_.param<double>("/multiple_object_tracking_lidar/logLengthScale_y", logLengthScale_y, 0.75);

    nh_.param<int>("/multiple_object_tracking_lidar/data_length", data_length, 10);
    nh_.param<bool>("/multiple_object_tracking_lidar/param_fix", param_fix, true);
}

void ObstacleTrack::spinNode()
{
    ros::Duration(1/frequency).sleep(); // Synchronize to lidar frequency 
    ros::spinOnce();
}

void ObstacleTrack::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) 
{
    // If this is the first frame, initialize kalman filters for the clustered objects
    if (firstFrame)
    {   
        if (map_init == false)
        {
            return; // if map doesn't be initialized, stop callback
        }
        if (input->header.stamp.toSec() < 1.0e9)
        {
            time_init = 0; // if real world, initialize to 0
        }
        if (input->header.stamp.toSec() - time_init < 0)
        {
            time_init = input->header.stamp.toSec(); // if Gazebo simulator, initialize to start timestamp
        }
        
        // cluster input PointClouds
        std::vector<pcl::PointXYZI> clusterCentroids;
        clusterCentroids = clusterPointCloud(input);

        // if no obstacles, exit callback 
        if (clusterCentroids.empty())
        {
            ROS_INFO("No obstacles around");
            return;
        }
        else // register first clusterCentroid
        {
            for (int i=0; i<clusterCentroids.size(); ++i)
            {
                registerNewObstacle(clusterCentroids[i]);         
            }   
        }   

        dt_gp = 1/frequency; // initialize dt for Infinite Horizon Gaussian Process
        firstFrame = false;
    }

    else
    { 
        // cluster input PointClouds
        std::vector<pcl::PointXYZI> clusterCentroids;
        clusterCentroids = clusterPointCloud(input);

        // exit callback if no obstacles
        if (clusterCentroids.size()==0)
        {
            ROS_INFO("No obstacles around");
            return;
        }

        std::vector<int> this_objIDs; // Obj lists for this timestamp
        for (const auto& obj: clusterCentroids)
        {
            bool registered = false;
            int update_index = 0;
            int objID = 0;

            std::vector<int>::iterator it;
            for (it=objIDs.begin(); it<objIDs.end(); ++it)
            {
                int index = std::distance(objIDs.begin(), it);

                // compare new cluster with final poses of registered objs
                Eigen::Vector3d P_now; // Position (now)
                Eigen::Vector3d P_last; // Position (last observation)
                P_now << obj.x, obj.y, 0;
                P_last << stack_obj[index][data_length-1].x, stack_obj[index][data_length-1].y, 0;

                if (euc_dist(P_now, P_last) < id_thershold)
                {
                    // if there is lost data, linear interpolation for sparse data
                    if (obj.intensity - stack_obj[index][data_length-1].intensity > 3*dt_gp)
                    {
                        fill_with_linear_interpolation(index, obj);
                    }

                    registered = true;
                    update_index = index;
                    objID = *it;
                    break;
                }
            }

            if (registered) // if already registered object, update data
            {
                updateObstacleQueue(update_index, obj);
                this_objIDs.push_back(objID);
            }
            else // if new object detects, register new GP model
            {
                this_objIDs.push_back(next_obj_num);
                registerNewObstacle(obj);
            }            
        }

        // call IHGP
        std::vector<std::vector<pcl::PointXYZI>> pos_vel_s;
        pos_vel_s = callIHGP(this_objIDs);

        // Publish TEB_Obstacle msg & rviz marker
        std::string frame_id = input->header.frame_id;
        publishObstacles(pos_vel_s, frame_id, this_objIDs);   
        publishMarkers(pos_vel_s, frame_id, this_objIDs);

        // unregister old unseen obstacles 
        unregisterOldObstacle(input->header.stamp.toSec() - time_init);
    } 
} 

void ObstacleTrack::mapCallback(const nav_msgs::OccupancyGrid& map_msg)
{
    // map information
    map_info = map_msg.info;
    
    // update map data to Eigen::MatrixXd form
    map_copy.resize(map_info.height, map_info.width);
    for (int i=0; i<map_msg.data.size(); i++) 
    {
        int row = i/map_info.width;
        int column = i%map_info.width;

        map_copy(row, column) = map_msg.data[i];
    }

    map_init = true;
}

void ObstacleTrack::publishObstacles(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, std::string frame_id, std::vector<int> this_objIDs)
{
    costmap_converter::ObstacleArrayMsg obstacle_array;

    // ObstacleArray header
    obstacle_array.header.stamp = ros::Time::now();
    obstacle_array.header.frame_id = frame_id;

    for (int i=0; i<pos_vel_s.size(); i++)
    {
        costmap_converter::ObstacleMsg obstacle;
        obstacle_array.obstacles.push_back(costmap_converter::ObstacleMsg());

        obstacle_array.obstacles[i].id = this_objIDs[i];
        obstacle_array.obstacles[i].radius = 0.3; //obstacle_radius
        obstacle_array.obstacles[i].header.stamp = ros::Time::now();
        obstacle_array.obstacles[i].header.frame_id = frame_id;

        // velocity
        obstacle_array.obstacles[i].velocities.twist.linear.x = pos_vel_s[i][1].x;
        obstacle_array.obstacles[i].velocities.twist.linear.y = pos_vel_s[i][1].y;
        obstacle_array.obstacles[i].velocities.twist.linear.z = 0;
        obstacle_array.obstacles[i].velocities.twist.angular.x = 0;
        obstacle_array.obstacles[i].velocities.twist.angular.y = 0;
        obstacle_array.obstacles[i].velocities.twist.angular.z = 0;

        obstacle_array.obstacles[i].velocities.covariance[0] = .1;
        obstacle_array.obstacles[i].velocities.covariance[7] = .1;
        obstacle_array.obstacles[i].velocities.covariance[14] = 1e9;
        obstacle_array.obstacles[i].velocities.covariance[21] = 1e9;
        obstacle_array.obstacles[i].velocities.covariance[28] = 1e9;
        obstacle_array.obstacles[i].velocities.covariance[35] = .1;

        // Polygon of obstacle
        std::vector<geometry_msgs::Point32> _points(1);
        obstacle_array.obstacles[i].polygon.points = _points;
        obstacle_array.obstacles[i].polygon.points[0].x = pos_vel_s[i][0].x;
        obstacle_array.obstacles[i].polygon.points[0].y = pos_vel_s[i][0].y;
        obstacle_array.obstacles[i].polygon.points[0].z = 0;

        obstacle_pub.publish(obstacle_array);
    }
}

void ObstacleTrack::publishMarkers(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, std::string frame_id, std::vector<int> this_objIDs)
{
    visualization_msgs::MarkerArray obstacleMarkers;

    // pose marker
    visualization_msgs::Marker m;
    m.id = 0;
    m.header.frame_id = frame_id;
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 2*0.25;         m.scale.y = 2*0.25;    
    for (int i=0; i<pos_vel_s.size(); i++)
    {   
        auto iter = std::find(objIDs.begin(), objIDs.end(), this_objIDs[i]);
        int index = std::distance(objIDs.begin(), iter);
        std_msgs::ColorRGBA color;
        color.r = colorset[index].r;
        color.g = colorset[index].g;
        color.b = colorset[index].b;
        color.a = colorset[index].a;

        geometry_msgs::Point p;
        p.x = pos_vel_s[i][0].x;
        p.y = pos_vel_s[i][0].y;
        p.z = 0;

        m.colors.push_back(color);
        m.points.push_back(p);
    }
    obstacleMarkers.markers.push_back(m);

    // velocity text marker
    for (int i=0; i<pos_vel_s.size(); i++)
    {
        visualization_msgs::Marker m;

        m.id = 2*this_objIDs[i]+1;
        // m.id = this_objIDs[i];

        m.header.frame_id = frame_id;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.z = 0.22; // text size
        
        m.color.r=1.0;
        m.color.g=1.0;
        m.color.b=1.0;
        m.color.a=1.0;

        m.pose.position.x = pos_vel_s[i][0].x;
        m.pose.position.y = pos_vel_s[i][0].y;
        m.pose.position.z = 0.0;  

        float velocity = round(sqrt(pow(pos_vel_s[i][1].x, 2) + pow(pos_vel_s[i][1].y, 2))*100) / 100;
        std::ostringstream velocity_3f;
        velocity_3f << std::setprecision(2) << velocity;
        std::string text = velocity_3f.str();
        m.text = text; 

        obstacleMarkers.markers.push_back(m);
    } 

    /*
    // velocity array marker
    for (int i=0; i<pos_vel_s.size(); i++)
    {
        visualization_msgs::Marker m;

        m.id = this_objIDs[i] + 2*pos_vel_s.size();
        // m.id = this_objIDs[i] + 1*pos_vel_s.size();
        //m.id = this_objIDs[i];

        m.header.frame_id = frame_id;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;

        // arrow orientation    arrow width         arrow height
        m.scale.x = 0.08;         m.scale.y = 0.15;         m.scale.z = 0.2;
        
        m.color.r=0.75;
        m.color.g=0.0;
        m.color.b=0.0;
        m.color.a=1.0;

        geometry_msgs::Point arrow_yaw;
        // specify start point for arrow
        arrow_yaw.x = pos_vel_s[i][0].x;
        arrow_yaw.y = pos_vel_s[i][0].y;
        arrow_yaw.z = 0.0;
        m.points.push_back(arrow_yaw);

        // specify start point for arrow
        arrow_yaw.x = pos_vel_s[i][0].x + 1*pos_vel_s[i][1].x;
        arrow_yaw.y = pos_vel_s[i][0].y + 1*pos_vel_s[i][1].y;
        arrow_yaw.z = 0.0;
        m.points.push_back(arrow_yaw);

        obstacleMarkers.markers.push_back(m);
    } */

    marker_pub.publish(obstacleMarkers);
}

std::vector<pcl::PointXYZI> ObstacleTrack::clusterPointCloud(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Process the point cloud 
    // change PointCloud data type (ros sensor_msgs to pcl_Pointcloud)
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg (*input, input_cloud);

    // Voxel Down sampling 
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ> cloud_1;
    vg.setInputCloud (input_cloud.makeShared());
    vg.setLeafSize (1*VoxelLeafSize, 1*VoxelLeafSize, 20*VoxelLeafSize); // Leaf size 0.1m
    vg.filter (cloud_1);

    // Remove static obstacles from occupied grid map msg
    pcl::PointCloud<pcl::PointXYZ> cloud_2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_2 = removeStatic(cloud_1);
    *cloud_filtered = cloud_2;

    // exit callback if no obstacles
    if (cloud_filtered->empty())
    {
        static const std::vector<pcl::PointXYZI> empty;
        return empty;
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
    ec.setClusterTolerance (ClusterTolerance);
    ec.setMinClusterSize (MinClusterSize);
    ec.setMaxClusterSize (MaxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);

    // Extract the clusters out of pc and save indices in cluster_indices.
    ec.extract (cluster_indices); // most of Runtime are used from this step.

    std::vector<pcl::PointXYZI> clusterCentroids;
    clusterCentroids = getCentroid(cluster_indices, *cloud_filtered, *input); 

    // // publish pointcloud for debuging 
    // sensor_msgs::PointCloud2 cloud_debug;
    // pcl::toROSMsg(cloud_1, cloud_debug);
    // cloud_debug.header.frame_id = "map";
    // pc1.publish(cloud_debug);

    // sensor_msgs::PointCloud2 cloud_debug2;
    // pcl::toROSMsg(cloud_2, cloud_debug2);
    // cloud_debug2.header.frame_id = "map";
    // pc2.publish(cloud_debug2);

    return clusterCentroids;
}

void ObstacleTrack::registerNewObstacle(pcl::PointXYZI centroid)
{
    // register objID list
    objIDs.push_back(next_obj_num);
    next_obj_num += 1;

    // fill every data with current input
    std::vector<pcl::PointXYZI> centroids;
    for (int i = 0; i < data_length; ++i)
    { 
        centroids.push_back(centroid);
    }
    stack_obj.push_back(centroids);

    // Set IHGP hyperparameters
    Matern32model model_x;
    Matern32model model_y;
    model_x.setSigma2(exp(logSigma2_x));
    model_x.setMagnSigma2(exp(logMagnSigma2_x)); 
    model_x.setLengthScale(exp(logLengthScale_x));

    model_y.setSigma2(exp(logSigma2_y));
    model_y.setMagnSigma2(exp(logMagnSigma2_y)); 
    model_y.setLengthScale(exp(logLengthScale_y));

    // GP initialization
    GPs_x.push_back(new InfiniteHorizonGP(dt_gp,model_x.getF(),model_x.getH(),model_x.getPinf(),model_x.getR(),model_x.getdF(),model_x.getdPinf(),model_x.getdR()));
    GPs_y.push_back(new InfiniteHorizonGP(dt_gp,model_y.getF(),model_y.getH(),model_y.getPinf(),model_y.getR(),model_y.getdF(),model_y.getdPinf(),model_y.getdR()));

    // Set rviz msg color
    std_msgs::ColorRGBA color;
    color.r = (float)std::rand()/(float)RAND_MAX;
    color.g = (float)std::rand()/(float)RAND_MAX;
    color.b = (float)std::rand()/(float)RAND_MAX;
    color.a = 0.8;
    colorset.push_back(color);
}

void ObstacleTrack::unregisterOldObstacle(double now)
{
    spin_counter += 1;

    // remove old obstacles every period (sec)
    double period = 5; // 5 sec
    if (spin_counter > period * frequency)
    {
        std::vector<int>::iterator it_objID = objIDs.begin();
        std::vector<std::vector<pcl::PointXYZI>>::iterator it_stack = stack_obj.begin();
        std::vector<InfiniteHorizonGP*>::iterator it_GPx = GPs_x.begin();
        std::vector<InfiniteHorizonGP*>::iterator it_GPy = GPs_y.begin();
        std::vector<std_msgs::ColorRGBA>::iterator it_color = colorset.begin();

        for (it_objID=objIDs.begin(); it_objID<objIDs.end();)
        {
            int index = std::distance(objIDs.begin(), it_objID);

            // if obj be unseen for period, remove
            if (now - stack_obj[index][data_length-1].intensity > period)
            {
                it_objID = objIDs.erase(it_objID); // remove objID 
                it_stack = stack_obj.erase(it_stack); // remove stack_obj
                it_GPx = GPs_x.erase(it_GPx); // remove GPs_x 
                it_GPy = GPs_y.erase(it_GPy); // remove GPs_y 
                it_color = colorset.erase(it_color);
            }
            else
            {
                it_objID++;
                it_stack++;
                it_GPx++;
                it_GPy++;
                it_color++;
            }
        }
        
        spin_counter = 0;
    }
}

void ObstacleTrack::updateObstacleQueue(const int i, pcl::PointXYZI centroid)
{
    // now update objects_centroids
    stack_obj[i].erase(stack_obj[i].begin());
    stack_obj[i].push_back(centroid);
}

void ObstacleTrack::fill_with_linear_interpolation(const int i, pcl::PointXYZI centroid)
{
    ROS_INFO_STREAM("obj[" << i << "] tracking loss");

    // linear interpolation
    pcl::PointXYZI last_centroid = stack_obj[i][data_length-1];  
    double dx_total = centroid.x - last_centroid.x; // dx between last timestamp and this timestamp
    double dy_total = centroid.y - last_centroid.y; // dy between last timestamp and this timestamp
    double dz_total = 0;
    double dt_total = centroid.intensity - last_centroid.intensity; // dt between last timestamp and this timestamp
    int lost_num = (int)round(dt_total/dt_gp) - 1; // # of lost data

    // (lost_num) times of linear interpolation
    for (int j=0; j < lost_num; ++j)
    {
        pcl::PointXYZI last_center = stack_obj[i][data_length-1];

        pcl::PointXYZI center;
        center.x = last_center.x + dx_total/lost_num;
        center.y = last_center.y + dy_total/lost_num;
        center.z = last_center.z + dz_total/lost_num;
        center.intensity = last_center.intensity + dt_gp;

        stack_obj[i].erase(stack_obj[i].begin());
        stack_obj[i].push_back(center);
    }
}

std::vector<std::vector<pcl::PointXYZI>> ObstacleTrack::callIHGP(std::vector<int> this_objIDs)
{
    // define vector for position and velocity
    std::vector<std::vector<pcl::PointXYZI>> pos_vel_s; // vector stack for objects pose and velocity
    int output_size = 2*this_objIDs.size();
    pos_vel_s.reserve(output_size);

    // call IHGP
    for (const int& n: this_objIDs)
    {
        // objID index for call GPs_x,y
        auto iter = std::find(objIDs.begin(), objIDs.end(), n);
        int index = std::distance(objIDs.begin(), iter);

        std::vector<pcl::PointXYZI> pos_vel; // vector stack for one object pose and velocity
        pos_vel.reserve(2);

        pcl::PointXYZI pos = LPF_pos(stack_obj[index], index);
        // pcl::PointXYZI pos = IHGP_fixed_pos(stack_obj[index], index);
        pcl::PointXYZI vel = IHGP_fixed_vel(stack_obj[index], index);

        // Nan exception
        if (isnan(vel.x) != 0 || isnan(vel.y) != 0)
        {
            ROS_ERROR("NaN detected in GP. Please restart this node");
        }

        // obstacle velocity bounding
        float obs_max_vx = 1.5; // (m/s)
        float obs_max_vy = 1.5;
        if (vel.x > obs_max_vx) {vel.x = obs_max_vx;}
        else if (vel.x < -obs_max_vx) {vel.x = -obs_max_vx;}
        if (vel.y > obs_max_vy) {vel.y = obs_max_vy;}
        else if (vel.y < -obs_max_vy) {vel.y = -obs_max_vy;}

        pos_vel.push_back(pos);
        pos_vel.push_back(vel); // e.g. pos_vel = [pos3, vel3]
        pos_vel_s.push_back(pos_vel); // e.g. pos_vel_s = [[pos1, vel1], [pos3, vel3], ...]
    }
    
    return pos_vel_s;
}

pcl::PointCloud<pcl::PointXYZ> ObstacleTrack::removeStatic(pcl::PointCloud<pcl::PointXYZ> input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_pre_process;

    // select pointclouds not above occupied cell(static cell)
    for (const auto& point: input_cloud.points)
    {
        bool endflag = 0;

        // get map index (column, row)
        float x_map = point.x - map_info.origin.position.x;
        float y_map = point.y - map_info.origin.position.y;
        float theta = quaternion2eularYaw(map_info.origin.orientation);
        int col = (cos(-theta)*x_map - sin(-theta)*y_map)/map_info.resolution;
        int row = (sin(-theta)*x_map + cos(-theta)*y_map)/map_info.resolution;

        // check neighborhood grid cell whether is occupied
        for (int i=-tolarance; i<=tolarance; ++i)
        {
            for (int j=-tolarance; j<=tolarance; ++j)
            {
                // if grid cell is occupied or unseen, leave out
                if (map_copy(row+i,col+j) > 50 || map_copy(row+i,col+j) == -1)
                {
                    endflag = 1;
                    break;
                }

                if (i==tolarance && j==tolarance)
                {
                    cloud_pre_process.push_back(point);
                }
            }

            if (endflag)
            {
                break;
            }
        }
    }

    return cloud_pre_process;
}

std::vector<pcl::PointXYZI> ObstacleTrack::getCentroid(std::vector<pcl::PointIndices> cluster_indices, const pcl::PointCloud<pcl::PointXYZ> cloud_filtered, const sensor_msgs::PointCloud2 input)
{
    // To separate each cluster out of the vector<PointIndices> we have to 
    // iterate through cluster_indices, create a new PointCloud for each 
    // entry and write all points of the current cluster in the PointCloud. 
    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    std::vector<pcl::PointXYZI> clusterCentroids;
    clusterCentroids.reserve(cluster_indices.size());

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
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

        pcl::PointXYZI centroid; 
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
        clusterCentroids.push_back(centroid);

        // set radius for publishObstacles
        Vector3d V_centroid(centroid.x, centroid.y, centroid.z);
        float obstacle_radius = euc_dist(V_centroid, Pj);
        if (obstacle_radius > 0.3) // obstacle radius constraint
        {
            obstacle_radius = 0.3;
        }
    }

    return clusterCentroids;
} 

pcl::PointXYZI ObstacleTrack::LPF_pos(std::vector<pcl::PointXYZI> centroids, int n)
{
    // TODO:
    // - [ ] change IHGP filter to Bilateral Filter for position

    return centroids[data_length-1];
}

pcl::PointXYZI ObstacleTrack::IHGP_fixed_pos(std::vector<pcl::PointXYZI> centroids, int n)
{
    // initialize AKHA and MF vector 
    GPs_x[n]->init_step();
    GPs_y[n]->init_step();

    // Data pre-precessing
    // set mean as last value
    double mean_x;
    double mean_y;
    int gp_data_len = data_length-1;

    mean_x = centroids[gp_data_len].x;
    mean_y = centroids[gp_data_len].y;

    // update IHGP queue through data
    for (int k=0; k<=gp_data_len; k++)
    {
        GPs_x[n]->update(centroids[k].x - mean_x);
        GPs_y[n]->update(centroids[k].y - mean_y);
    }

    // Pull out the marginal mean and variance estimates
    std::vector<double> Eft_x = GPs_x[n]->getEft();
    std::vector<double> Eft_y = GPs_y[n]->getEft();

    // make return_data as PCL::PointXYZI
    pcl::PointXYZI predicted_centroid;
    predicted_centroid.x = Eft_x[gp_data_len] + mean_x;
    predicted_centroid.y = Eft_y[gp_data_len] + mean_y;
    predicted_centroid.z = 0.0;
    predicted_centroid.intensity = centroids[data_length-1].intensity;
  
    return predicted_centroid;
}

pcl::PointXYZI ObstacleTrack::IHGP_fixed_vel(std::vector<pcl::PointXYZI> centroids, int n)
{
    // initialize AKHA and MF vector 
    GPs_x[n]->init_step();
    GPs_y[n]->init_step();

    // Data pre-precessing
    // set mean as last value
    double mean_x;
    double mean_y;
    std::vector<double> vx_raw; 
    std::vector<double> vy_raw; 

    // calculate mean
    int gp_data_len = data_length-2;

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
   
    
    // update IHGP queue through data
    for (int k=0; k<=gp_data_len; k++)
    {
        GPs_x[n]->update(vx_raw[k] - mean_x);
        GPs_y[n]->update(vy_raw[k] - mean_y);
    }

    // Pull out the marginal mean and variance estimates
    std::vector<double> Eft_x = GPs_x[n]->getEft();
    std::vector<double> Eft_y = GPs_y[n]->getEft();

    // make return_data as PCL::PointXYZI
    pcl::PointXYZI predicted_centroid;
    predicted_centroid.x = Eft_x[gp_data_len] + mean_x;
    predicted_centroid.y = Eft_y[gp_data_len] + mean_y;
    predicted_centroid.z = 0.0;
    predicted_centroid.intensity = centroids[data_length-1].intensity;
  
    return predicted_centroid;
}

pcl::PointXYZI ObstacleTrack::IHGP_nonfixed(std::vector<pcl::PointXYZI> centroids, int n)
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

float ObstacleTrack::quaternion2eularYaw(geometry_msgs::Quaternion q)
{
    double yaw_angle;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw_angle = std::atan2(siny_cosp, cosy_cosp);

    return yaw_angle;
}

float ObstacleTrack::euc_dist(Vector3d P1, Vector3d P2)
{
    return std::sqrt((P1(0)-P2(0))*(P1(0)-P2(0)) + (P1(1)-P2(1))*(P1(1)-P2(1)) + (P1(2)-P2(2))*(P1(2)-P2(2)));
}
