#include "ros/ros.h"
#include <cstdio>

struct WayPointFollowerConfig
{
  // number of waypoints this node will follow
  int max_waypoints = 5;

  // rate at which ros spin once will be called
  double loop_rate = 10.0;

  // time to wait for move base server before we return
  double wait_timeout = 10.0;

  // move base server's name
  std::string move_base_server_name = "move_base";

  // waypoints topic name
  std::string waypoint_topic = "waypoints";

  // clicked point topioc
  std::string clicked_point_topic = "clicked_point";

  std::string move_base_frame_id = "/map";

  WayPointFollowerConfig();

  // for max_waypoints
  void get_param_if_available(
    const ros::NodeHandle& node, const std::string& _key,
    int _param_out);

  // for loop rate
  void get_param_if_available(
    const ros::NodeHandle& node, const std::string& _key,
    double _param_out);

  // for move base server name and waypoint topic
  void get_param_if_available(
    const ros::NodeHandle& node, const std::string& _key,
    std::string _param_out);

  static WayPointFollowerConfig make();

  WayPointFollowerConfig get_config();

  void print_config();

};
