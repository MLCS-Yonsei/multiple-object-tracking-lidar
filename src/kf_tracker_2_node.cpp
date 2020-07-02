#include <ros/ros.h>
#include "kf_tracker_2/kf_tracker_2.h"

int main(int argc, char **argv)
{
  try
  {
    // ROS init
    ros::init (argc,argv,"kf_tracker_2");
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
        // ros::Duration(0.05).sleep();
        ros::spinOnce();
      }
    }
  }

  catch (ros::Exception& e)
  {
    ROS_ERROR("kf_tracker_2_node: Error occured: %s", e.what());
    exit(1);
  }  

  return 0;
}
