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
        pointnet_sub = nh_.subscribe("/predicted_centroid", 1, &ObstacleTrack::pointnetCallback, this);

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

// subscribe callback func when use pointnet for detecting object's centroid 
// subscribe pointnet x,y -> change to map tf -> ihgp
void ObstacleTrack::pointnetCallback(const geometry_msgs::PoseArray input_msg)
{   
    // TODO:
    // [] fix pointnet's output data to array
    // [] insert input header frame (before: sensor msgs header, now: ros::Time::now() )
    // [] set ihgp params
    // [] how to do when more than 1 object detected? use vector ros tf?
    // [] how to measure object's radius (before: 2 point's euc distance, now : 0.1) 
    // [] is it right to set tf from velodyne to object ? 
    //    --> need check for pointnet's output x,y point when moving velodyne's direction 

    if (firstFrame)
    {
        transform.setOrigin( tf::Vector3(array.data[0], array.data[1], 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        tf_broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/object"));

        try
        {
            tf_listener.waitForTransform("/map", "/object", ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("/map", "/object", ros::Time(0), stamped_transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // get relative_centroid using transform    
        pointnet_centroid.x = stamped_transform.getOrigin().x();
        pointnet_centroid.y = stamped_transform.getOrigin().y();
        pointnet_centroid.z = 0.0;
        pointnet_centroid.intensity = ros::Time::now().toSec();

        // cout << pointnet_centroid.x << " " <<pointnet_centroid.y<<endl;

        
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
        centroids.push_back(pointnet_centroid);
    }

    else
    {
        // transform.setOrigin( tf::Vector3(array.data[0], array.data[1], 0.0) );
        // transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        // tf_broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/object"));

        // try
        // {
        //     tf_listener.waitForTransform("/map", "/object", ros::Time(0), ros::Duration(3.0));
        //     tf_listener.lookupTransform("/map", "/object", ros::Time(0), stamped_transform);
        // }
        // catch (tf::TransformException &ex) {
        //     ROS_ERROR("%s",ex.what());
        //     ros::Duration(1.0).sleep();
        // }

        // // get relative_centroid using transform    
        // pointnet_centroid.x = stamped_transform.getOrigin().x();
        // pointnet_centroid.y = stamped_transform.getOrigin().y();
        // pointnet_centroid.z = 0.0;
        // pointnet_centroid.intensity = ros::Time::now().toSec();

        
        std::vector<pcl::PointXYZI> centroids; // t centroids

        // centroids array update
        pointnet_centroids.erase(pointnet_centroids.begin());
        pointnet_centroids.push_back(centroids);

        // Predict with GP 
        pcl::PointXYZI predicted_centroid;
        pcl::PointXYZI predicted_velocity;
        if (param_fix == true) 
        { 
            predicted_centroid = IHGP_fixed(centroids, "pos"); 
            predicted_velocity = IHGP_fixed(centroids, "vel");
        }
        //else if(param_fix == false) { predicted_centroid = IHGP_nonfixed(centroids); }

        // obstacle velocity bounding
        double obs_max_vx = 1.0; // (m/s)
        double obs_max_vy = 1.0;
        if (predicted_velocity.x > obs_max_vx) {predicted_velocity.x = obs_max_vx;}
        else if (predicted_velocity.x < -obs_max_vx) {predicted_velocity.x = -obs_max_vx;}

        if (predicted_velocity.y > obs_max_vy) {predicted_velocity.y = obs_max_vy;}
        else if (predicted_velocity.y < -obs_max_vy) {predicted_velocity.y = -obs_max_vy;}

        // Publish state & rviz marker 
        publishObstacles_2(predicted_centroid, predicted_velocity);
        publishMarkers(predicted_centroid); //changed obstacle radius to 0.1
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
