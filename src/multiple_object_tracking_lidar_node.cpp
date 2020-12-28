#include <ros/ros.h>
#include "multiple_object_tracking_lidar/multiple_object_tracking_lidar.h"

int main(int argc, char **argv)
{
  try
  {
    // ROS init
    ros::init (argc,argv,"multiple_object_tracking_lidar");
    ros::NodeHandle nh;
    ObstacleTrack obstacles;
  
    if(!obstacles.initialize())
    {
      ROS_ERROR_STREAM_NAMED("FAILED TO INITIALIZE %s", ros::this_node::getName().c_str());
      exit(1);
    }
    else{
      ROS_INFO_STREAM_NAMED("%s INITIALIZED SUCCESSFULLY!", ros::this_node::getName().c_str());
      while (nh.ok()) {
        obstacles.spinNode();
      }
    }
  }

  catch (ros::Exception& e)
  {
    ROS_ERROR("multiple_object_tracking_lidar_node: Error occured: %s", e.what());
    exit(1);
  }  

  return 0;
}
