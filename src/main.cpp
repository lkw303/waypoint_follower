#include "waypoint_follower.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_follower_node");
  ros::NodeHandle ros_node_handle;

  auto config = WayPointFollowerConfig::make();
  auto waypoint_follower = WayPointFollower::make(config);

  if (!waypoint_follower)
  {
    ROS_ERROR("waypoint_follower_node: unable to initialize.");
    return 1;
  }

  return 0;
}