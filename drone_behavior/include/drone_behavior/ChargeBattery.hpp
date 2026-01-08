#ifndef CHARGEBATTERY_HPP
#define CHARGEBATTERY_HPP

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

class ChargeBattery : public BT::SyncActionNode
{
private:
public:
    ChargeBattery(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)

    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<double>("battery_level")};
    }

    virtual BT::NodeStatus tick() override
    {
        // Here you would update your battery system
        // For now, we just set the blackboard value
        setOutput("battery_level", 100.0);

        return BT::NodeStatus::SUCCESS;
    }
};

#endif