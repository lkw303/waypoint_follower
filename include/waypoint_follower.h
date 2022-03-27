#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "waypoint_follower_config.h"

#include <memory>
#include <thread>

using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using MoveBaseClientSharedPtr = std::shared_ptr<MoveBaseClient>;

class WayPointFollower
{
public:
  using SharedPtr = std::shared_ptr<WayPointFollower>;

  //default constructor
  WayPointFollower(const WayPointFollowerConfig& config);

  ~WayPointFollower();

  // factory method
  static WayPointFollower::SharedPtr make(const WayPointFollowerConfig& config);


private:


  std::shared_ptr<ros::NodeHandle> nh;

  ros::Subscriber waypoint_sub;

  ros::Subscriber clicked_point_sub;

  std::unique_ptr<ros::Rate> loop_rate;

  std::thread loop_thread;

  std::vector<move_base_msgs::MoveBaseGoal> waypoints;

  MoveBaseClientSharedPtr move_base_client;

  WayPointFollowerConfig config;

  //current waypoint index
  int current_wp;

  // length in waypoints atribute
  int wp_counter;

  // max number of waypoints
  int max_wp;

  // if the robot is enroute to any waypoints
  // bool enroute;

  // if the robot has began its journey
  bool begin;

  // if the waypoints vector is filled
  bool wp_filled;

  void waypoint_cb(const geometry_msgs::PoseStamped& msg);

  void clicked_point_cb(const geometry_msgs::PointStamped& msg);

  void start();

  void set_move_base_client(MoveBaseClientSharedPtr);

  void go_to(const int&);

  void begin_movement();

  void clear_wp();

  void check_goals();

  void loop();

  // send the next goal in the waypoints vector
  void send_next_goal();
};