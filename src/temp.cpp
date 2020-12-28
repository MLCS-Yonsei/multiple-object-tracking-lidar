// subscribe callback func when use pointnet for detecting object's center 
void ObstacleTrack::cloudCallback(const geometry_msgs::PoseArray input)
{   
    // timestamp, dt_gp initialization
    if (firstFrame)
    {
        // if gazebo world, reset time_init as 0
        if (input.header.stamp.toSec() < 1.0e9) 
        {
            time_init = 0;
        }
        // if real world rosbag (past data), reset time_init as first input timestamp
        if (input.header.stamp.toSec() < time_init) 
        {
            time_init = input.header.stamp.toSec();
        }

        firstFrame = false;

        // TODO: change dt_gp to adaptive
        dt_gp = 1/frequency;
    }

    else
    {   
        // // if empty data, stop callback
        // if (input.poses.size() == 0)
        // {
        //     ROS_INFO_STREAM("there is no dynamic obstacle(human) or impossible to trace");
        //     return;
        // }

        std::vector<int> this_objIDs = {}; // object ID for latest input msg
        for (int i=0; i<input.poses.size(); i++)
        {
            // get objID
            int objID = (input.poses[i].orientation.w)/2;

            // if already registered obj msg
            if (std::find(objIDs.begin(), objIDs.end(), objID) != objIDs.end() )
            {
                // index of objects_centroids about objID
                int index = std::find(objIDs.begin(), objIDs.end(), objID) - objIDs.begin();

                // if there is lost data, linear interpolation for sparse data
                pcl::PointXYZI last_centroid = objects_centroids[index][data_length-1];
                if (input.header.stamp.toSec() - time_init - last_centroid.intensity > 2*dt_gp) 
                {
                    fill_with_linear_interpolation(i, input, index);
                }

                // now update objects_centroids
                updateObstacleQueue(i, input, index);
                
                // add objID to queue (now observable objects)
                this_objIDs.push_back(objID); 
            }
            // if new object detects, register new GP model
            else 
            {
                // register new GP model with input
                registerNewObstacle(i, input, objID);

                // register new objID & add objID to queue (now observable objects)
                this_objIDs.push_back(objID);
                objIDs.push_back(objID);     
            }
        }

        // call IHGP
        std::vector<std::vector<pcl::PointXYZI>> pos_vel_s;
        pos_vel_s = callIHGP(this_objIDs);

        // Publish TEB_Obstacle msg & rviz marker
        std::string frame_id = input.header.frame_id;
        publishObstacles(pos_vel_s, frame_id, this_objIDs);   
        publishMarkers(pos_vel_s, frame_id, this_objIDs);
    }
}




//---------------------------------------------------------------------------

// Process the point cloud 
// change PointCloud data type (ros sensor_msgs to pcl_Pointcloud)
pcl::PointCloud<pcl::PointXYZ> input_cloud;
pcl::fromROSMsg (*input, input_cloud);

// filter pointcloud only z>0 and update to z=0
// pcl::PointCloud<pcl::PointXYZ> cloud_0;
// for (const auto& point: input_cloud)
// {
//     if(point.z > 0.0)
//     {
//         cloud_0.push_back(point);
//         cloud_0.back().z = 0;
//     }
// }

// Voxel Down sampling 
pcl::VoxelGrid<pcl::PointXYZ> vg;
pcl::PointCloud<pcl::PointXYZ> cloud_1;
vg.setInputCloud (input_cloud.makeShared());
vg.setLeafSize (1*VoxelLeafSize, 1*VoxelLeafSize, 20*VoxelLeafSize); // Leaf size 0.1m
vg.filter (cloud_1);

// // publish pointcloud for debuging 
// sensor_msgs::PointCloud2 cloud_debug;
// pcl::toROSMsg(cloud_1, cloud_debug);
// cloud_debug.header.frame_id = "map";
// pc1.publish(cloud_debug);

// Remove static obstacles from occupied grid map msg
pcl::PointCloud<pcl::PointXYZ> cloud_3;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
cloud_3 = removeStatic(cloud_1);
*cloud_filtered = cloud_3;

// // publish pointcloud for debuging 
// sensor_msgs::PointCloud2 cloud_debug2;
// pcl::toROSMsg(cloud_3, cloud_debug2);
// cloud_debug2.header.frame_id = "map";
// pc2.publish(cloud_debug2);

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
ec.setClusterTolerance (ClusterTolerance);
ec.setMinClusterSize (MinClusterSize);
ec.setMaxClusterSize (MaxClusterSize);
ec.setSearchMethod (tree);
ec.setInputCloud (cloud_filtered);

// Extract the clusters out of pc and save indices in cluster_indices.
ec.extract (cluster_indices); // most of Runtime are used from this step.

if (cluster_indices.empty())
    return;

// Predict obstacle center with circumcenter method
std::vector<pcl::PointXYZI> clusterCentroids;
clusterCentroids = getCentroid(cluster_indices, *cloud_filtered, *input);    