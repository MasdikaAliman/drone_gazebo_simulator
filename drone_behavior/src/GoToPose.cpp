
#include "GoToPose.hpp"
BT::NodeStatus GoToPose::onStart()
{
  if (!node_ptr)
  {
    std::cout << "ROS2 Node not registered" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();

  using namespace std::placeholders;

  send_goal_options.result_callback = std::bind(&GoToPose::results_cb, this, _1);

  client_ptr = rclcpp_action::create_client<MoveToPose>(node_ptr, "/move_to_pose_drone");

  // Get Input From Behavior Tree
  Pose targetPos;
  int state;
  getInput<Pose>("Target", targetPos);
  getInput<int>("State_Type", state);

  done_flag_ = false;

  auto goal_msg = MoveToPose::Goal();

  goal_msg.target_pose.x = targetPos.x;
  goal_msg.target_pose.y = targetPos.y;
  goal_msg.target_pose.z = targetPos.z;
  goal_msg.target_pose.yaw = targetPos.theta;
  goal_msg.target_pose.state_type = state;

  client_ptr->async_send_goal(goal_msg, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
  if (done_flag_)
  {
    if (move_to_pose_res == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(node_ptr->get_logger(), "[%s] Goal Reached", this->name().c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_INFO(node_ptr->get_logger(), "[%s] Failed To Reach goal ", this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void GoToPose::results_cb(const GoalHandleClient::WrappedResult &result)
{
  if (result.result)
  {
    done_flag_ = true;
    move_to_pose_res = result.code;
  }
}

void GoToPose::onHalted()
{
  RCLCPP_INFO(node_ptr->get_logger(), "%s NODE GOT HALTED", this->name().c_str());
}