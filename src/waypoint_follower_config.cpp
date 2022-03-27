#include "waypoint_follower_config.h"

  WayPointFollowerConfig::WayPointFollowerConfig() {}

  // for max_waypoints
  void WayPointFollowerConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    int _param_out)
  {
    int tmp_param;
    if(_node.getParam(_key, tmp_param))
    {
      ROS_INFO("setting %s to %d", _key.c_str(), tmp_param);
      _param_out = tmp_param;
    }
  }

  // for loop rate
  void WayPointFollowerConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    double _param_out)
  {
    double tmp_param;
    if(_node.getParam(_key, tmp_param))
    {
      ROS_INFO("setting %s to %d", _key.c_str(), tmp_param);
      _param_out = tmp_param;
    }
  }

  // for move base server name and waypoint topic
  void WayPointFollowerConfig::get_param_if_available(
    const ros::NodeHandle& _node, const std::string& _key,
    std::string _param_out)
  {
    std::string tmp_param;
    if(_node.getParam(_key, tmp_param))
    {
      ROS_INFO("setting %s to %s", _key.c_str(), tmp_param.c_str());
      _param_out = tmp_param;
    }
  }

  WayPointFollowerConfig WayPointFollowerConfig::make()
  {
    WayPointFollowerConfig config;

    ros::NodeHandle node_private_ns("~");
    config.get_param_if_available(
      node_private_ns, "max_waypoints", config.max_waypoints);
    config.get_param_if_available(
      node_private_ns, "loop_rate", config.max_waypoints);
    config.get_param_if_available(
      node_private_ns, "waypoint_topic", config.wait_timeout);
    config.get_param_if_available(
      node_private_ns, "clicked_point_topic", config.wait_timeout);
    config.get_param_if_available(
      node_private_ns, "move_base_frame_id", config.wait_timeout);

    return config;
  }

  WayPointFollowerConfig WayPointFollowerConfig::get_config()
  {
    WayPointFollowerConfig config;
    config.max_waypoints = max_waypoints;
    config.loop_rate = loop_rate;
    config.wait_timeout = wait_timeout;
    config.move_base_server_name = move_base_server_name;
    config.waypoint_topic = waypoint_topic;
    config.clicked_point_topic = clicked_point_topic;
    return config;
  }

  void WayPointFollowerConfig::print_config()
  {
    printf("waypoints to follow: %d", max_waypoints);
    printf("loop rate: %d ms", loop_rate);
    printf("wait timeout for move base server: %d s", wait_timeout);
    printf("move base server's name %s", move_base_server_name.c_str());
    printf("waypoint topic: %s", waypoint_topic.c_str());
    printf("clicked points topic: %s", clicked_point_topic.c_str());
  }
