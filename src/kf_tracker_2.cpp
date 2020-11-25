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
    nh_.param<float>("/kf_tracker_2/frequency", frequency, 10.0);

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
void ObstacleTrack::pointnetCallback(const geometry_msgs::PoseArray input)
{   
    // TODO:
    // [] how to measure object's radius (before: 2 point's euc distance, now : 0.3 constant) 
    // [] change how to get dt_gp

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
        // if empty data, stop callback
        if (input.poses.size() == 0)
        {
            ROS_INFO_STREAM("there is no dynamic obstacle(human) or impossible to trace");
            return;
        }

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

        // Publish state & rviz marker
        std::string frame_id = input.header.frame_id;
        publishObstacles(pos_vel_s, frame_id, this_objIDs);   
        publishMarkers(pos_vel_s, frame_id, this_objIDs);
    }
}

void ObstacleTrack::publishObstacles(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, std::string frame_id, std::vector<int> this_objIDs)
{
    costmap_converter::ObstacleArrayMsg obstacle_array;

    // ObstacleArray header
    obstacle_array.header.stamp = ros::Time::now();
    obstacle_array.header.frame_id = "map";

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

        // orientation
        float yaw;
        yaw = atan2(pos_vel_s[i][1].x, pos_vel_s[i][1].y);
        tf2::Quaternion Quaternion;
        Quaternion.setRPY(0,0,yaw);
        geometry_msgs::Quaternion quat_msg;
        quat_msg = tf2::toMsg(Quaternion);
        obstacle_array.obstacles[i].orientation = quat_msg;

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
    for (int i=0; i<pos_vel_s.size(); i++)
    {
        visualization_msgs::Marker m;

        m.id = this_objIDs[i];
        m.header.frame_id = frame_id;
        m.type = visualization_msgs::Marker::CYLINDER;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = 2*0.3;         m.scale.y = 2*0.3;         m.scale.z = 0.05;
        // m.scale.x=2*obstacle_radius;         m.scale.y=2*obstacle_radius;         m.scale.z=0.05;
        
        m.color.r=0.5;
        m.color.g=0.5;
        m.color.b=0.5;
        m.color.a=0.7;

        m.pose.position.x = pos_vel_s[i][0].x;
        m.pose.position.y = pos_vel_s[i][0].y;
        m.pose.position.z = 0.0;  

        obstacleMarkers.markers.push_back(m);
    }

    // velocity text marker
    for (int i=0; i<pos_vel_s.size(); i++)
    {
        visualization_msgs::Marker m;

        m.id = this_objIDs[i] + pos_vel_s.size();
        m.header.frame_id = frame_id;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.z = 0.25; // text size
        
        m.color.r=1.0;
        m.color.g=1.0;
        m.color.b=1.0;
        m.color.a=1.0;

        m.pose.position.x = pos_vel_s[i][0].x;
        m.pose.position.y = pos_vel_s[i][0].y + 0.35;
        m.pose.position.z = 0.0;  

        double velocity = sqrt(pow(pos_vel_s[i][1].x, 2) + pow(pos_vel_s[i][1].y, 2));
        m.text = to_string(velocity);
        //m.text = to_string(this_objIDs[i]);

        obstacleMarkers.markers.push_back(m);
    }

    // velocity array marker
    for (int i=0; i<pos_vel_s.size(); i++)
    {
        visualization_msgs::Marker m;

        m.id = this_objIDs[i] + 2*pos_vel_s.size();
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
    }

    marker_pub.publish(obstacleMarkers);
}

void ObstacleTrack::registerNewObstacle(int i, const geometry_msgs::PoseArray input, const int objID)
{
    std::vector<pcl::PointXYZI> centroids;
    
    pcl::PointXYZI center;
    center.x = input.poses[i].position.x;
    center.y = input.poses[i].position.y;
    center.z = input.poses[i].position.z;
    center.intensity = input.header.stamp.toSec() - time_init;

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

}

void ObstacleTrack::updateObstacleQueue(int i, const geometry_msgs::PoseArray input, int index)
{
    // now update objects_centroids
    pcl::PointXYZI center;
    center.x = input.poses[i].position.x;
    center.y = input.poses[i].position.y;
    center.z = input.poses[i].position.z;
    center.intensity = input.header.stamp.toSec() - time_init;

    objects_centroids[index].erase(objects_centroids[index].begin());
    objects_centroids[index].push_back(center);
}

void ObstacleTrack::fill_with_linear_interpolation(int i, const geometry_msgs::PoseArray input, int index)
{
    ROS_INFO_STREAM("obj[" << i << "] tracking loss");

    pcl::PointXYZI last_centroid = objects_centroids[index][data_length-1];  

    double dx_total = input.poses[i].position.x - last_centroid.x; // dx between last timestamp and this timestamp
    double dy_total = input.poses[i].position.y - last_centroid.y; // dy between last timestamp and this timestamp
    double dz_total = input.poses[i].position.z - last_centroid.z; // dz between last timestamp and this timestamp
    double dt_total = input.header.stamp.toSec()  - time_init - last_centroid.intensity; // dt between last timestamp and this timestamp
    int lost_num = (int)round(dt_total/dt_gp) - 1; // # of lost data

    // (lost_num) times of linear interpolation
    for (int i=0; i < lost_num; i++)
    {
        pcl::PointXYZI last_center = objects_centroids[index][data_length-1];

        pcl::PointXYZI center;
        center.x = last_center.x + dx_total/lost_num;
        center.y = last_center.y + dy_total/lost_num;
        center.z = last_center.z + dz_total/lost_num;
        center.intensity = last_center.intensity + dt_gp;

        objects_centroids[index].erase(objects_centroids[index].begin());
        objects_centroids[index].push_back(center);
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
        int index = std::find(objIDs.begin(), objIDs.end(), n) - objIDs.begin();

        std::vector<pcl::PointXYZI> pos_vel; // vector stack for one object pose and velocity
        pos_vel.reserve(2);

        pcl::PointXYZI pos = IHGP_fixed(objects_centroids[index], index, "pos"); 
        pcl::PointXYZI vel = IHGP_fixed(objects_centroids[index], index, "vel");

        // obstacle velocity bounding
        if (vel.x > 2.0) {vel.x = 2.0;}
        else if (vel.x < -2.0) {vel.x = -2.0;}

        if (vel.y > 2.0) {vel.y = 2.0;}
        else if (vel.y < -2.0) {vel.y = -2.0;}

        pos_vel.push_back(pos);
        pos_vel.push_back(vel); // e.g. pos_vel = [pos3, vel3]
        pos_vel_s.push_back(pos_vel); // e.g. pos_vel_s = [[pos1, vel1], [pos3, vel3], ...]
    }

    return pos_vel_s;
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
