#ifndef DRONECHECKBATTERY_HPP
#define DRONECHECKBATTERY_HPP

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "nav_msgs/msg/odometry.hpp"

class DroneCheckBattery : public BT::ConditionNode

{
private:
    rclcpp::Node::SharedPtr node_ptr_;
    double battery_level_;
    double battery_threhsold_;
    double last_x, last_y, last_z;
    double total_distance_;
    double first_reading;
    nav_msgs::msg::Odometry odom_;

    const double BATTERY_CONSUMPTION_PER_METER = 2.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subs;

public:
    DroneCheckBattery(const std::string &name, const BT::NodeConfig &conf,
                      rclcpp::Node::SharedPtr ptr_node) : BT::ConditionNode(name, conf), node_ptr_(ptr_node),
                                                          battery_level_(100.0), battery_threhsold_(40.0),
                                                          total_distance_(0.0), first_reading(true)
    {

        using namespace std::placeholders;

        odom_subs = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&DroneCheckBattery::odomCallback, this, std::placeholders::_1));
    };

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;

        double x = odom_.pose.pose.position.x;
        double y = odom_.pose.pose.position.y;
        double z = odom_.pose.pose.position.z;

        if (first_reading)
        {
            last_x = x;
            last_y = y;
            last_z = z;
            first_reading = false;
        }
        else
        {
            double dx = x - last_x;
            double dy = y - last_y;
            double dz = z - last_z;

            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            total_distance_ += distance;

            battery_level_ -= distance * BATTERY_CONSUMPTION_PER_METER;

            if (battery_level_ < 0)
                battery_level_ = 0;

            last_x = x;
            last_y = y;
            last_z = z;

            RCLCPP_INFO(node_ptr_->get_logger(),
                        "Odom received. Battery: %.1f, Distance: %.2fm, Pos: (%.2f, %.2f, %.2f)",
                        battery_level_, total_distance_, x, y, z);
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("Battery_Threshold"),
            BT::InputPort<double>("battery_level"), // READ from blackboard
            BT::OutputPort<double>("battery_level") // WRITE to blackboard
        };
    }

    double getBatteryLevel() const { return battery_level_; }

    virtual BT::NodeStatus tick() override
    {

        double blackboard_battery = 100.0; // Default
        // if (getInput<double>("battery_level", blackboard_battery))
        // {
        //     // Update our internal battery level with blackboard value
        //     // This allows <Script code="battery_level:=100.0"/> to work!
        //     battery_level_ = blackboard_battery;
        //     RCLCPP_ERROR(node_ptr_->get_logger(),
        //                  "Read battery from blackboard: %.1f%%", battery_level_);
        // }

        if (!getInput<double>("Battery_Threshold", battery_threhsold_))
        {
            RCLCPP_ERROR(node_ptr_->get_logger(), "ERROR INPUT BATTERY THRESHOLD");
            // return BT::NodeStatus::IDLE;
        }
        if (battery_level_ < battery_threhsold_)
        {

            if (getInput<double>("battery_level", battery_level_))
            {
                RCLCPP_ERROR(node_ptr_->get_logger(), "CAN'T GET THE VALUE BATTERY_LEVEL_");
                double errorX = -1.0 - odom_.pose.pose.position.x;
                double errorY = 0.0 - odom_.pose.pose.position.y;
                double errorZ = 0.55 - odom_.pose.pose.position.z;

                double distError = sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ);
                if (distError < 0.05)
                {
                    // setOutput("battery_level", battery_level_);
                    return BT::NodeStatus::SUCCESS;
                }
                //    setOutput("battery_level", battery_level_);
            }

            RCLCPP_WARN(node_ptr_->get_logger(),
                        "Battery low (%.1f%%). Need to return to charging station! Position: (%.2f, %.2f, %.2f)",
                        battery_level_, odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_ptr_->get_logger(),
                    "Battery OK: %.1f%%. Position: (%.2f, %.2f, %.2f)",
                    battery_level_, odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
        return BT::NodeStatus::SUCCESS;
    }
};

#endif