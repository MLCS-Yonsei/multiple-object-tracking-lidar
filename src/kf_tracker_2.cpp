#include "kf_tracker_2/kf_tracker_2.h"

/* TODO
1. [Proceeding] Static/Dynamic Obstacle Filtering >> Low velocity accuracy
2. [Proceeding] KF pos & vel publishing >> Low velocity accuracy
3. Outside Variable Packaging
*/

// RUNTIME DEBUG
clock_t s_1, s_2, s_3, s_4, s_5;
clock_t e_1, e_2, e_3, e_4, e_5;

/* KF init */
int stateDim=4; // [x, y, v_x, v_y] //,w,h]
int measDim=2; // [z_x, z_y, z_w, z_h]
int ctrlDim=0;
cv::KalmanFilter KF0(stateDim,measDim,ctrlDim,CV_32F);
cv::KalmanFilter KF1(stateDim,measDim,ctrlDim,CV_32F);
cv::KalmanFilter KF2(stateDim,measDim,ctrlDim,CV_32F);
cv::KalmanFilter KF3(stateDim,measDim,ctrlDim,CV_32F);
cv::KalmanFilter KF4(stateDim,measDim,ctrlDim,CV_32F);
cv::KalmanFilter KF5(stateDim,measDim,ctrlDim,CV_32F);
cv::Mat state(stateDim,1,CV_32F);
cv::Mat_<float> measurement(2,1); 

std::vector<geometry_msgs::Point> prevClusterCenters;
std::vector<int> objID;// Output of the data association using KF

int obstacle_num_;
float ClusterTolerance_=0.3; // (m) default 0.3
int MinClusterSize_=10; // default 10
int MaxClusterSize_=600; // default 600
bool firstFrame = true;


ObstacleTrack::ObstacleTrack()
{
    // this->reconfigure_server_.reset();
    ;
}

ObstacleTrack::~ObstacleTrack()
{
    clearDataMember();
}

bool ObstacleTrack::initialize()
{ 
    if (ros::ok())
    {
        /** Initialize publisher of obstacles **/
        nh_.param<int>("obstacle_num", obstacle_num_, 6); // maximum obstacle number is 6

        cout<<"DEBUG: obstacle_num_ = "<< obstacle_num_ <<endl;
        // Create a ROS Publishers for the state of objects (pos and vel)
        // objState0_pub = nh_.advertise<geometry_msgs::Twist> ("obj_0",1);
        // objState1_pub = nh_.advertise<geometry_msgs::Twist> ("obj_1",1);
        // objState2_pub = nh_.advertise<geometry_msgs::Twist> ("obj_2",1);
        // objState3_pub = nh_.advertise<geometry_msgs::Twist> ("obj_3",1);
        // objState4_pub = nh_.advertise<geometry_msgs::Twist> ("obj_4",1);
        // objState5_pub = nh_.advertise<geometry_msgs::Twist> ("obj_5",1);
        
        // Create a ROS publisher for the output point cloud
        // cluster0_pub = nh_.advertise<sensor_msgs::PointCloud2>("cluster_0", 1);
        // cluster1_pub = nh_.advertise<sensor_msgs::PointCloud2>("cluster_1", 1);
        // cluster2_pub = nh_.advertise<sensor_msgs::PointCloud2>("cluster_2", 1);
        // cluster3_pub = nh_.advertise<sensor_msgs::PointCloud2>("cluster_3", 1);
        // cluster4_pub = nh_.advertise<sensor_msgs::PointCloud2>("cluster_4", 1);
        // cluster5_pub = nh_.advertise<sensor_msgs::PointCloud2>("cluster_5", 1);

        // Create a ROS Publishers for the objID of objects
        objID_pub = nh_.advertise<std_msgs::Int32MultiArray>("obj_id", 1);
        // Create a ROS Publihsers for rviz visualization
        markerPub = nh_.advertise<visualization_msgs::MarkerArray>("viz", 1);

        /** Initialize Subscriber for input Pointcloud2  **/
        input_sub = nh_.subscribe("input_pointcloud", 1, &ObstacleTrack::cloudCallback, this);

        // Initialize Kalman filter 
        ros::spinOnce();
        
        ROS_INFO_STREAM("ObstacleTrack Initialized");
        return true;
    }
    else
    {
        return false;
    }
}

void ObstacleTrack::spinNode()
{
  ros::spin();
}

void ObstacleTrack::clearDataMember()
{
    ;
}

void ObstacleTrack::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // If this is the first frame, initialize kalman filters for the clustered objects
    if (firstFrame)
    {   
        initKalmanFilters(input);
    }
 
    else
    { 
        /* Process the point cloud */
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        /** Creating the KdTree from input point cloud **/
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::fromROSMsg (*input, *input_cloud);
        tree->setInputCloud (input_cloud);

        /* Here we are creating a vector of PointIndices, which contains the actual index
        * information in a vector<int>. The indices of each detected cluster are saved here.
        * Cluster_indices is a vector containing one instance of PointIndices for each detected 
        * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
        */
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (ClusterTolerance_);
        ec.setMinClusterSize (MinClusterSize_);
        ec.setMaxClusterSize (MaxClusterSize_);
        ec.setSearchMethod (tree);
        ec.setInputCloud (input_cloud);

        s_1 = clock();
        /** Extract the clusters out of pc and save indices in cluster_indices.**/
        ec.extract (cluster_indices); // most of Runtime are used from this step.
        e_1 = (clock()-s_1)*(1e-3);
        cout<<"[RUNTIME] extraction : "<<e_1<<" ms"<<endl;
        
        /* To separate each cluster out of the vector<PointIndices> we have to 
        * iterate through cluster_indices, create a new PointCloud for each 
        * entry and write all points of the current cluster in the PointCloud. 
        */
        // pcl::PointXYZ origin (0,0,0);
        // float mindist_this_cluster = 1000;
        // float dist_this_point = 1000;
        std::vector<pcl::PointIndices>::const_iterator it;
        std::vector<int>::const_iterator pit;

        /** Cluster Centroid **/
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec; // Vector of cluster pointclouds 
        std::vector<pcl::PointXYZ> clusterCentroids; // Cluster centroids 
        std::vector<geometry_msgs::Point> clusterCenters;

        clusterCenters.reserve(6);
        for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            float x=0.0; float y=0.0;
            int numPts=0;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
            {
                cloud_cluster->points.push_back(input_cloud->points[*pit]);

                x+=input_cloud->points[*pit].x;
                y+=input_cloud->points[*pit].y;
                numPts++;

                //dist_this_point = pcl::geometry::distance(input_cloud->points[*pit], origin);
                //mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
            }
            pcl::PointXYZ centroid;
            geometry_msgs::Point centroid_geo;
            centroid.x=x/numPts;
            centroid.y=y/numPts;
            centroid.z=0.0;

            centroid_geo.x=x/numPts;
            centroid_geo.y=y/numPts;
            centroid_geo.z=0.0;

            cluster_vec.push_back(cloud_cluster);
            clusterCentroids.push_back(centroid); // Get the centroid of the cluster
            clusterCenters.push_back(centroid_geo);
        }
        /* Ensure at least obstacle_num_ clusters exist to publish (later clusters may be empty) */
        while (cluster_vec.size() < obstacle_num_){
            pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
            cluster_vec.push_back(empty_cluster);
        }
        while (clusterCentroids.size() < obstacle_num_)
        {
            pcl::PointXYZ centroid;
            geometry_msgs::Point centroid_geo;
            centroid.x=0.0;
            centroid.y=0.0;
            centroid.z=0.0;
            centroid_geo.x=0.0;
            centroid_geo.y=0.0;
            centroid_geo.z=0.0;
            
            clusterCentroids.push_back(centroid);
            clusterCenters.push_back(centroid_geo);
        }

        /* Predict with Kalman filter */
        KFT(clusterCenters);

    }
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
    prevClusterCenters = clusterCenters;
    markerPub.publish(clusterMarkers);
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

void ObstacleTrack::initKalmanFilters(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // cout<<"DEBUG: run before initKalmanFilters"<<endl;
    /** Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.  **/
    // Could be made generic by creating a Kalman Filter only when a new object is detected  
    float dvx = 0.01f; //1.0
    float dvy = 0.01f;//1.0
    float dx = 1.0f;
    float dy = 1.0f;
    KF0.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
    KF1.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
    KF2.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
    KF3.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
    KF4.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
    KF5.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);

    cv::setIdentity(KF0.measurementMatrix);
    cv::setIdentity(KF1.measurementMatrix);
    cv::setIdentity(KF2.measurementMatrix);
    cv::setIdentity(KF3.measurementMatrix);
    cv::setIdentity(KF4.measurementMatrix);
    cv::setIdentity(KF5.measurementMatrix);

    // Process Noise Covariance Matrix Q
    // [ Ex 0  0    0 0    0 ]
    // [ 0  Ey 0    0 0    0 ]
    // [ 0  0  Ev_x 0 0    0 ]
    // [ 0  0  0    1 Ev_y 0 ]
    //// [ 0  0  0    0 1    Ew ]
    //// [ 0  0  0    0 0    Eh ]
    float sigmaP=0.01;
    float sigmaQ=0.1;
    setIdentity(KF0.processNoiseCov, Scalar::all(sigmaP));
    setIdentity(KF1.processNoiseCov, Scalar::all(sigmaP));
    setIdentity(KF2.processNoiseCov, Scalar::all(sigmaP));
    setIdentity(KF3.processNoiseCov, Scalar::all(sigmaP));
    setIdentity(KF4.processNoiseCov, Scalar::all(sigmaP));
    setIdentity(KF5.processNoiseCov, Scalar::all(sigmaP));

    // Meas noise cov matrix R
    cv::setIdentity(KF0.measurementNoiseCov, cv::Scalar(sigmaQ));//1e-1
    cv::setIdentity(KF1.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF2.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF3.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF4.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF5.measurementNoiseCov, cv::Scalar(sigmaQ));

    /* Process the point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *input_cloud);
    tree->setInputCloud (input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.08); 
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (600);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);

    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract (cluster_indices);
    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    /* Cluster Centroid */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec; // Vector of cluster pointclouds
    std::vector<pcl::PointXYZ> clusterCentroids; // Cluster centroids
    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        float x=0.0; 
        float y=0.0;
        int numPts=0;

        for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
        {
            cloud_cluster->points.push_back(input_cloud->points[*pit]);
            x+=input_cloud->points[*pit].x;
            y+=input_cloud->points[*pit].y;
            numPts++;
            //dist_this_point = pcl::geometry::distance(input_cloud->points[*pit], origin);
            //mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
        }
        pcl::PointXYZ centroid;
        centroid.x=x/numPts;
        centroid.y=y/numPts;
        centroid.z=0.0;
        
        cluster_vec.push_back(cloud_cluster);
        clusterCentroids.push_back(centroid); //Get the centroid of the cluster
    }

    // Ensure at least obstacle_num_ clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < obstacle_num_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
        cluster_vec.push_back(empty_cluster);
    }
    while (clusterCentroids.size() < obstacle_num_)
    {
        pcl::PointXYZ centroid;
        centroid.x=0.0;
        centroid.y=0.0;
        centroid.z=0.0;
        
        clusterCentroids.push_back(centroid);
    }

    /* Set initial state */
    KF0.statePre.at<float>(0)=clusterCentroids.at(0).x;
    KF0.statePre.at<float>(1)=clusterCentroids.at(0).y;
    KF0.statePre.at<float>(2)=0;// initial v_x
    KF0.statePre.at<float>(3)=0;// initial v_y

    KF1.statePre.at<float>(0)=clusterCentroids.at(1).x;
    KF1.statePre.at<float>(1)=clusterCentroids.at(1).y;
    KF1.statePre.at<float>(2)=0;// initial v_x
    KF1.statePre.at<float>(3)=0;// initial v_y

    KF2.statePre.at<float>(0)=clusterCentroids.at(2).x;
    KF2.statePre.at<float>(1)=clusterCentroids.at(2).y;
    KF2.statePre.at<float>(2)=0;// initial v_x
    KF2.statePre.at<float>(3)=0;// initial v_y

    KF3.statePre.at<float>(0)=clusterCentroids.at(3).x;
    KF3.statePre.at<float>(1)=clusterCentroids.at(3).y;
    KF3.statePre.at<float>(2)=0;// initial v_x
    KF3.statePre.at<float>(3)=0;// initial v_y

    KF4.statePre.at<float>(0)=clusterCentroids.at(4).x;
    KF4.statePre.at<float>(1)=clusterCentroids.at(4).y;
    KF4.statePre.at<float>(2)=0;// initial v_x
    KF4.statePre.at<float>(3)=0;// initial v_y

    KF5.statePre.at<float>(0)=clusterCentroids.at(5).x;
    KF5.statePre.at<float>(1)=clusterCentroids.at(5).y;
    KF5.statePre.at<float>(2)=0;// initial v_x
    KF5.statePre.at<float>(3)=0;// initial v_y

    firstFrame = false;

    prevClusterCenters.reserve(obstacle_num_);
    for (int i=0; i<obstacle_num_; i++)
    {
        geometry_msgs::Point pt;
        pt.x=clusterCentroids.at(i).x;
        pt.y=clusterCentroids.at(i).y;
        prevClusterCenters.push_back(pt);
    }
}

void ObstacleTrack::KFT(const std::vector<geometry_msgs::Point> clusterCenters)
{
    /** 1. First predict, to update the internal statePre variable **/
    std::vector<cv::Mat> pred{KF0.predict(),KF1.predict(),KF2.predict(),KF3.predict(),KF4.predict(),KF5.predict()};
  
    /** 2. Get the position of the prediction **/
    std::vector<geometry_msgs::Point> KFpredictions;
    KFpredictions.reserve(obstacle_num_);
    int i=0;
    for (auto it=pred.begin(); it!=pred.end(); it++)
    {
        // float vel = (std::sqrt(pow((*it).at<float>(2), 2) + pow((*it).at<float>(3), 2))) * (1e7);
        // if (vel < 20) continue; // if static obstacle, ignore.

        geometry_msgs::Point pt;
        pt.x=(*it).at<float>(0);
        pt.y=(*it).at<float>(1);
        pt.z=(*it).at<float>(2);

        KFpredictions.push_back(pt); 
    }

    /** 3. Find the cluster that is more probable to be belonging to a given KF.**/
    objID.clear(); // Clear the objID vector
    objID.resize(obstacle_num_); // Allocate default elements so that [i] doesnt segfault. Should be done better

    // Copy clusterCenters for modifying it and preventing multiple assignments of the same ID 
    std::vector<geometry_msgs::Point> copyOfClusterCenters(clusterCenters); 
    std::vector<std::vector<float>> distMat; 
    distMat.reserve(obstacle_num_);
    for(int filterN = 0; filterN < obstacle_num_; filterN++)
    {
        std::vector<float> distVec; 
        distVec.reserve(obstacle_num_);
        for(int n=0; n<obstacle_num_; n++)
        {
            distVec.push_back(ObstacleTrack::euclidean_distance(KFpredictions[filterN],copyOfClusterCenters[n]));
        }
        distMat.push_back(distVec);
        
        // Based on distVec instead of distMat (global min). Has problems with the person's leg going out of scope 
        int ID = std::distance(distVec.begin(), min_element(distVec.begin(), distVec.end()));
        objID[filterN]=ID;
        // Prevent assignment of the same object ID to multiple clusters
        copyOfClusterCenters[ID].x=100000;// A large value so that this center is not assigned to another cluster
        copyOfClusterCenters[ID].y=10000;
        copyOfClusterCenters[ID].z=10000;
        
    }

    for(int clusterCount=0; clusterCount < obstacle_num_; clusterCount++)
    {
        // 1. Find min(distMax)==> (i,j);
        std::pair<int,int> minIndex(ObstacleTrack::findIndexOfMin(distMat));

        // 2. objID[i]=clusterCenters[j]; counter++
        objID[minIndex.first]=minIndex.second;

        // 3. distMat[i,:]=10000; distMat[:,j]=10000
        distMat[minIndex.first]=std::vector<float>(6,10000.0);// Set the row to a high number.
        for(int row=0;row<distMat.size();row++)//set the column to a high number
        {
            distMat[row][minIndex.second]=10000.0;
        }

        // 4. if(counter<6) got to 1.
    }
    // countIDs(objID); // for verif/corner cases


    /* 6. KF update part */
    // convert clusterCenters from geometry_msgs::Point to floats 
    std::vector<std::vector<float>> cc;
    cc.reserve(obstacle_num_); 
    for (int i=0; i<objID.size(); i++) 
    {
        vector<float> pt;
        pt.reserve(3);
        pt.push_back(clusterCenters[objID[i]].x);
        pt.push_back(clusterCenters[objID[i]].y);
        pt.push_back(clusterCenters[objID[i]].z);

        cc.push_back(pt);
    }

    // std::vector<cv::Mat> estimated;
    // estimated.reserve(objID.size());
    // for (int i=0; i<objID.size(); i++)
    // {
    //     float meas[2] = {cc[i].at(0), cc[i].at(1)};
    //     cv::Mat measMat = cv::Mat(2,1,CV_32F,meas);
    //     estimated.push_back(KF)
    // }

    float meas0[2]={cc[0].at(0),cc[0].at(1)};
    float meas1[2]={cc[1].at(0),cc[1].at(1)};
    float meas2[2]={cc[2].at(0),cc[2].at(1)};
    float meas3[2]={cc[3].at(0),cc[3].at(1)};
    float meas4[2]={cc[4].at(0),cc[4].at(1)};
    float meas5[2]={cc[5].at(0),cc[5].at(1)};

    /* The update phase */
    cv::Mat meas0Mat=cv::Mat(2,1,CV_32F,meas0);
    cv::Mat meas1Mat=cv::Mat(2,1,CV_32F,meas1);
    cv::Mat meas2Mat=cv::Mat(2,1,CV_32F,meas2);
    cv::Mat meas3Mat=cv::Mat(2,1,CV_32F,meas3);
    cv::Mat meas4Mat=cv::Mat(2,1,CV_32F,meas4);
    cv::Mat meas5Mat=cv::Mat(2,1,CV_32F,meas5);

    // update the predicted state from the measurement
    // if (!(meas0Mat.at<float>(0,0)==0.0f || meas0Mat.at<float>(1,0)==0.0f))
        cv::Mat estimated0 = KF0.correct(meas0Mat);
    // if (!(meas1[0]==0.0f || meas1[1]==0.0f))
        cv::Mat estimated1 = KF1.correct(meas1Mat);
    // if (!(meas2[0]==0.0f || meas2[1]==0.0f))
        cv::Mat estimated2 = KF2.correct(meas2Mat);
    // if (!(meas3[0]==0.0f || meas3[1]==0.0f))
        cv::Mat estimated3 = KF3.correct(meas3Mat);
    // if (!(meas4[0]==0.0f || meas4[1]==0.0f))
        cv::Mat estimated4 = KF4.correct(meas4Mat);
    // if (!(meas5[0]==0.0f || meas5[1]==0.0f))
        cv::Mat estimated5 = KF5.correct(meas5Mat);

    // std::vector<geometry_msgs::Point> KFcorrections;
    // KFcorrections.reserve(obstacle_num_);
    // int i=0;
    // for (auto it=pred.begin(); it!=pred.end(); it++)
    // {
    //     // float vel = (std::sqrt(pow((*it).at<float>(2), 2) + pow((*it).at<float>(3), 2))) * (1e7);
    //     // if (vel < 20) continue; // if static obstacle, ignore.

    //     geometry_msgs::Point pt;
    //     pt.x=(*it).at<float>(0);
    //     pt.y=(*it).at<float>(1);
    //     pt.z=(*it).at<float>(2);

    //     KFpredictions.push_back(pt); 
    // }

    // cout<<"[DEBUG] vel_0 : "<<(std::sqrt(pow(estimated0.at<float>(2), 2) + pow(estimated0.at<float>(3), 2))*(1e3))<<endl;
    // cout<<"[DEBUG] vel_1 : "<<(std::sqrt(pow(estimated1.at<float>(2), 2) + pow(estimated1.at<float>(3), 2))*(1e3))<<endl;
    // cout<<"[DEBUG] vel_2 : "<<(std::sqrt(pow(estimated2.at<float>(2), 2) + pow(estimated2.at<float>(3), 2))*(1e3))<<endl;
    // cout<<"[DEBUG] vel_3 : "<<(std::sqrt(pow(estimated3.at<float>(2), 2) + pow(estimated3.at<float>(3), 2))*(1e3))<<endl;
    // cout<<"[DEBUG] vel_4 : "<<(std::sqrt(pow(estimated4.at<float>(2), 2) + pow(estimated4.at<float>(3), 2))*(1e3))<<endl;
    // cout<<"[DEBUG] vel_5 : "<<(std::sqrt(pow(estimated5.at<float>(2), 2) + pow(estimated5.at<float>(3), 2))*(1e3))<<endl;

    /* 4. Publish the object marker(Display objIDs) */
    publishMarkers(KFpredictions, clusterCenters);

    /* 5. Publish the object IDs */
    publishObjID();

    cout<<"DONE KF_TRACKER"<<endl;  
}

std::pair<int,int> ObstacleTrack::findIndexOfMin(std::vector<std::vector<float>> distMat)
{
    std::pair<int,int>minIndex;
    float minEl=std::numeric_limits<float>::max();

    for (int i=0; i<distMat.size();i++)
    {
        for(int j=0;j<distMat.at(0).size();j++)
        {
            if( distMat[i][j] < minEl)
            {
            minEl = distMat[i][j];
            minIndex = std::make_pair(i,j);
            }
        }
    }
    return minIndex;
}

double ObstacleTrack::euclidean_distance(geometry_msgs::Point& p1, geometry_msgs::Point& p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

