#include "kf_tracker_2/kf_tracker_2.h"

//TODO: 
// 1) PCL Clustering memorized last step
// 2) multi obstacle ID

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
        obstacle_pub = nh_.advertise<costmap_converter::ObstacleArrayMsg> ("move_base/TebLocalPlannerROS/obstacles", 10); // the state of objects (pos and vel)
        // objID_pub = nh_.advertise<std_msgs::Int32MultiArray>("obj_id", 1); // the objID of objects
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("tracker_viz", 10); // rviz visualization

        // Initialize Subscriber
        pointnet_sub = nh_.subscribe("/people_pose", 1, &ObstacleTrack::pointnetCallback, this);

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
    nh_.param<int>("/kf_tracker_2/min_cluster_size", MinClusterSize_, 5);
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

// subscribe callback func when use pointnet for detecting object's center 
// subscribe pointnet x,y -> change to map tf -> ihgp
void ObstacleTrack::pointnetCallback(const geometry_msgs::PoseArray input)
{   
    // TODO:
    // [] fix pointnet's output data to array
    // [] insert input header frame (before: sensor msgs header, now: ros::Time::now() )
    // [] set ihgp params
    // [] how to do when more than 1 object detected? use vector ros tf?
    // [] how to measure object's radius (before: 2 point's euc distance, now : 0.1) 
    // [] is it right to set tf from velodyne to object ? 
    //    --> need check for pointnet's output x,y point when moving velodyne's direction 


    // [] change publishObstacles(), publishMarkers()
    // [] change how to get dt_gp
    // [] add code comments and clean up
    if (firstFrame)
    {
        if (input.header.stamp.toSec() < 1.0e9)
        {
            time_init = 0;
        }
        if (input.header.stamp.toSec() - time_init < 0 && t_init==false)
        {
            time_init = input.header.stamp.toSec();
            t_init = true;
        }

        firstFrame = false;

        // TODO: change dt_gp to adaptive
        dt_gp = 0.1;

        // ROS_INFO_STREAM("0");
    }

    else
    {   
        // cout<<"========== "<<input.poses.size()<<endl;

        // if empty data, return 0
        if (input.poses.size() == 0)
        {
            ROS_INFO_STREAM("0. no dynamic obstacle(human)");
            return;
        }

        std::vector<int> this_objIDs = {}; // object ID
        for (int i=0; i<input.poses.size(); i++)
        {
            // get objID
            int objID = (input.poses[i].orientation.w)/2;
            // cout<<"----------- "<<objID<<endl;

            // already registered obj
            if (std::find(objIDs.begin(), objIDs.end(), objID) != objIDs.end() )
            {
                int index = std::find(objIDs.begin(), objIDs.end(), objID) - objIDs.begin();

                // linear interpolation for sparse data
                pcl::PointXYZI last_centroid = objects_centroids[index][data_length-1];
                if (input.header.stamp.toSec() - last_centroid.intensity > 2*dt_gp) // if there is lost data
                {
                    // ROS_INFO_STREAM("2. linear interpolation");
                    double dx_total = input.poses[i].position.x - last_centroid.x; // dx between last timestamp and this timestamp
                    double dy_total = input.poses[i].position.y - last_centroid.y; // dy between last timestamp and this timestamp
                    double dz_total = input.poses[i].position.z - last_centroid.z; // dz between last timestamp and this timestamp
                    double dt_total = input.header.stamp.toSec() - last_centroid.intensity; // dt between last timestamp and this timestamp
                    int lost_num = (int)round(dt_total/dt_gp) - 1; // # of lost data

                    // (lost_num) times of linear interpolation
                    for (int i=0; i < lost_num; i++)
                    {
                        pcl::PointXYZI last_center = objects_centroids[index][data_length-1];

                        pcl::PointXYZI center;
                        center.x = last_center.x + dx_total/lost_num;
                        center.y = last_center.y + dy_total/lost_num;
                        center.z = last_center.z + dz_total/lost_num;
                        center.intensity = last_center.intensity + dt_total/lost_num;
                    
                        objects_centroids[index].erase(objects_centroids[index].begin());
                        objects_centroids[index].push_back(center);
                    }
                }
                
                // ROS_INFO_STREAM("2. update centroid");
                // now update objects_centroids
                pcl::PointXYZI center;
                center.x = input.poses[i].position.x;
                center.y = input.poses[i].position.y;
                center.z = input.poses[i].position.z;
                center.intensity = input.header.stamp.toSec();

                objects_centroids[index].erase(objects_centroids[index].begin());
                objects_centroids[index].push_back(center);

                this_objIDs.push_back(objID);   
            }
            
            else // if new object detects, register new GP model
            {
                // ROS_INFO_STREAM("1. start");
                std::vector<pcl::PointXYZI> centroids;
                
                pcl::PointXYZI center;
                center.x = input.poses[i].position.x;
                center.y = input.poses[i].position.y;
                center.z = input.poses[i].position.z;
                center.intensity = input.header.stamp.toSec();

                // fill every data with current input
                for (int j = 0; j < data_length; j++)
                {
                    centroids.push_back(center);
                }
                objects_centroids.push_back(centroids);

                // Set IHGP hyperparameters
                Matern32model model_x;
                Matern32model model_y;
                model_x.setSigma2(exp(logSigma2_x_));
                model_x.setMagnSigma2(exp(logMagnSigma2_x_)); 
                model_x.setLengthScale(exp(logLengthScale_x_));

                model_y.setSigma2(exp(logSigma2_y_));
                model_y.setMagnSigma2(exp(logMagnSigma2_y_)); 
                model_y.setLengthScale(exp(logLengthScale_y_));

                // GP initialization
                GPs_x.push_back(new InfiniteHorizonGP(dt_gp,model_x.getF(),model_x.getH(),model_x.getPinf(),model_x.getR(),model_x.getdF(),model_x.getdPinf(),model_x.getdR()));
                GPs_y.push_back(new InfiniteHorizonGP(dt_gp,model_y.getF(),model_y.getH(),model_y.getPinf(),model_y.getR(),model_y.getdF(),model_y.getdPinf(),model_y.getdR()));

                // register objID
                this_objIDs.push_back(objID);
                objIDs.push_back(objID);
                // ROS_INFO_STREAM("1. done");               
            }
        }

        // ROS_INFO_STREAM("3. start");

        // call IHGP
        std::vector<std::vector<pcl::PointXYZI>> pos_vel_s; // vector stack for objects pose and velocity
        int output_size = 2*this_objIDs.size();
        pos_vel_s.reserve(output_size);

        for (const int& n: this_objIDs)
        {
            int index = std::find(objIDs.begin(), objIDs.end(), n) - objIDs.begin();

            std::vector<pcl::PointXYZI> pos_vel;
            pos_vel.reserve(2);

            pcl::PointXYZI pos = IHGP_fixed(objects_centroids[index], index, "pos"); 
            pcl::PointXYZI vel = IHGP_fixed(objects_centroids[index], index, "vel");

            // obstacle velocity bounding
            if (vel.x > 2.0) {vel.x = 2.0;}
            else if (vel.x < -2.0) {vel.x = -2.0;}

            if (vel.y > 2.0) {vel.y = 2.0;}
            else if (vel.y < -2.0) {vel.y = -2.0;}

            // cout<<"["<<n<<"] "<<pos.x<<", "<<pos.y<<endl;
            double velocity = sqrt(pow(vel.x,2) + pow(vel.y,2));
            cout<<"["<<n<<"] "<<velocity<<endl;

            pos_vel.push_back(pos);
            pos_vel.push_back(vel);
            pos_vel_s.push_back(pos_vel);
        }

        // ROS_INFO_STREAM("3. done");

        // Publish state & rviz marker
        // publishObstacles()
        // publishMarkers();
    }
}

void ObstacleTrack::publishObstacles(pcl::PointXYZI position, pcl::PointXYZI velocity)
{
    costmap_converter::ObstacleArrayMsg obstacle_array;
    costmap_converter::ObstacleMsg obstacle;

    // ObstacleArray header
    obstacle_array.header.stamp = ros::Time::now();
    obstacle_array.header.frame_id = "map";
    
    obstacle_array.obstacles.push_back(costmap_converter::ObstacleMsg());
    obstacle_array.obstacles[0].id = 99;
    obstacle_array.obstacles[0].radius = 0.1; //obstacle_radius
    obstacle_array.obstacles[0].header.stamp = ros::Time::now();
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
        m.scale.x=2*0.1;         m.scale.y=2*0.1;         m.scale.z=0.05;
        // m.scale.x=2*obstacle_radius;         m.scale.y=2*obstacle_radius;         m.scale.z=0.05;
        
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

pcl::PointXYZI ObstacleTrack::IHGP_fixed(std::vector<pcl::PointXYZI> centroids, int n, string variable)
{
    GPs_x[n]->init_step();
    GPs_y[n]->init_step();

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
            GPs_x[n]->update(vx_raw[k] - mean_x);
            GPs_y[n]->update(vy_raw[k] - mean_y);
        }
    }
    else if(variable == "pos")
    {
        for (int k=0; k<=gp_data_len; k++)
        {
            GPs_x[n]->update(centroids[k].x - mean_x);
            GPs_y[n]->update(centroids[k].y - mean_y);
        }
    }

    // Pull out the marginal mean and variance estimates
    std::vector<double> Eft_x = GPs_x[n]->getEft();
    std::vector<double> Eft_y = GPs_y[n]->getEft();

    // make PCL::PointXYZI data
    pcl::PointXYZI predicted_data;
    predicted_data.x = Eft_x[gp_data_len] + mean_x;
    predicted_data.y = Eft_y[gp_data_len] + mean_y;
    predicted_data.z = 0.0;
    predicted_data.intensity = centroids[data_length-1].intensity;
  
    return predicted_data;
}
