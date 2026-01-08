#ifndef GOTOPOSE_HPP
#define GOTOPOSE_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "drone_msgs/action/move_to_pose_drone.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
// Struct to keep location pose data
struct Pose
{
  double x;
  double y;
  double z;
  double theta;
};

// Template specialization to converts a string to Position2D.
namespace BT
{
  template <>
  inline Pose convertFromString(StringView str)
  {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 4)
    {
      throw RuntimeError("invalid Pose format. Expected x;y;z;theta");
    }

    else
    {
      Pose output;
      output.x = convertFromString<double>(parts[0]);
      output.y = convertFromString<double>(parts[1]);
      output.z = convertFromString<double>(parts[2]);
      output.theta = convertFromString<double>(parts[3]);
      return output;
    }
  }
} // end namespace BT

class GoToPose : public BT::StatefulActionNode
{
private:
  using MoveToPose = drone_msgs::action::MoveToPoseDrone;
  using GoalHandleClient = rclcpp_action::ClientGoalHandle<MoveToPose>;

  rclcpp::Node::SharedPtr node_ptr;
  rclcpp_action::Client<MoveToPose>::SharedPtr client_ptr;
  rclcpp_action::ResultCode move_to_pose_res;

  bool done_flag_ = false;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  void results_cb(const GoalHandleClient::WrappedResult &result);

public:
  GoToPose(const std::string &name,
           const BT::NodeConfig &conf,
           rclcpp::Node::SharedPtr ptr_node)
      : BT::StatefulActionNode(name, conf),
        node_ptr(ptr_node) {}

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<Pose>("Target"),
        BT::InputPort<int>("State_Type")
    };
  }
};



#endif