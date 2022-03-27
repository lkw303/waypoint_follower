#include "waypoint_follower.h"
#include <assert.h>

WayPointFollower::WayPointFollower(const WayPointFollowerConfig& _config)
  : config(_config), wp_counter(0), current_wp(0), begin(false), wp_filled(false)
{
  //
}

WayPointFollower::SharedPtr WayPointFollower::make(const WayPointFollowerConfig& _config)
{
  auto waypoint_follower = std::shared_ptr<WayPointFollower>(new WayPointFollower(_config));
  waypoint_follower->nh.reset(new ros::NodeHandle());

  MoveBaseClientSharedPtr _move_base_client = std::make_shared<MoveBaseClient>
      (_config.move_base_server_name, true);

  // if move base server is not up we will not proceed!
  if (!_move_base_client->waitForServer(ros::Duration(_config.wait_timeout)))
  {
    ROS_ERROR("timed out waiting for action server: %s",
        _config.move_base_server_name.c_str());
    return nullptr;
  }

  ROS_INFO("connected with move base action server: %s",
      _config.move_base_server_name.c_str());

  waypoint_follower->set_move_base_client(_move_base_client);


  waypoint_follower->start();
  return waypoint_follower;
}


WayPointFollower::~WayPointFollower()
{
  if(loop_thread.joinable())
  {
    loop_thread.join();
    ROS_INFO("loop_thread joined.");
  }
}

void WayPointFollower::start()
{
  loop_rate.reset(new ros::Rate(config.loop_rate));
  max_wp = config.max_waypoints;
  waypoint_sub = nh->subscribe(
        config.waypoint_topic, 100,
        &WayPointFollower::waypoint_cb, this);

  clicked_point_sub = nh->subscribe(
        config.clicked_point_topic, 100,
        &WayPointFollower::clicked_point_cb, this);

  // run loop() on a separate thread
  loop_thread = std::thread(std::bind(&WayPointFollower::loop, this));
  // loop();
}

void WayPointFollower::set_move_base_client(MoveBaseClientSharedPtr _client)
{
  move_base_client = _client;
}


void WayPointFollower::go_to(const int& index)
{
  auto goal = waypoints[index];
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client->sendGoal(goal);
  ROS_INFO("on route to waypoint %d. %d / %d waypoints completed", index+1, index, max_wp);
}

// begin the traversal of waypoints
// called once there has been max_wp number of waypoints accumulated
void WayPointFollower::begin_movement()
{
  assert(current_wp == 0);
  go_to(current_wp);
  begin = true;
  // enroute = true;
}

void WayPointFollower::clear_wp()
{
  waypoints.clear();
  wp_counter = 0;
  current_wp = 0;
  begin = false;
  wp_filled = false;
  // enroute =false;

  ROS_INFO("We have finished traversing all %d waypoints!", max_wp);
}

void WayPointFollower::check_goals()
{
  if (!wp_filled)
    return;

  if(!begin)
    begin_movement();

  auto _move_base_state = move_base_client->getState();
  if ( _move_base_state == actionlib::SimpleClientGoalState::SUCCEEDED
    || _move_base_state == actionlib::SimpleClientGoalState::LOST
    || _move_base_state == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (current_wp + 1 == max_wp)
    {
      clear_wp();
      return;
    }

    go_to(++current_wp);
    return;
  }
}

void WayPointFollower::loop()
{
  while (nh->ok())
  {
    ros::spinOnce();
    check_goals();
  }
}

void WayPointFollower::waypoint_cb(const geometry_msgs::PoseStamped& msg)
{
  if (wp_filled)
    return;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header = msg.header;
  goal.target_pose.pose = msg.pose;
  waypoints.push_back(goal);

  if (++wp_counter == max_wp)
  {
    wp_filled = true;
    ROS_INFO("Already have %d Waypoints! Good to go!",wp_counter);
    return;
  };

  ROS_INFO("Received waypoint! Currently %d waypoints cached.", wp_counter);
}

void WayPointFollower::clicked_point_cb(const geometry_msgs::PointStamped& msg)
{
  if (wp_filled)
    return;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header = msg.header;
  goal.target_pose.header.frame_id = config.move_base_frame_id;
  goal.target_pose.pose.position = msg.point;
  goal.target_pose.pose.orientation.w = 0.0;
  waypoints.push_back(goal);

  if (++wp_counter == max_wp)
  {
    wp_filled = true;
    ROS_INFO("Already have %d Waypoints! Good to go!",wp_counter);
    return;
  };

  ROS_INFO("Received waypoint! Currently %d waypoints cached.", wp_counter);

}