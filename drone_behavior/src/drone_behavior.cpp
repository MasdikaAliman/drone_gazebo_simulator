#include <memory>

#include "GoToPose.hpp"
#include "CheckBattery.hpp"
#include "ChargeBattery.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
using namespace std::chrono_literals;

const std::string default_bt_xml_file =
    ament_index_cpp::get_package_share_directory("drone_behavior") + "/config/behavior_tree_drone.xml";

enum State
{
    IDLE = 0,
    AUTOMATIC = 1,
    TAKING_OFF = 2,
    LANDING = 3,
    EMERGENCY_STOP = 4
};

namespace drone_behavior
{
    class DroneBehaviorNode : public rclcpp::Node
    {
    private:
    public:
        rclcpp::TimerBase::SharedPtr timer_update_;
        std::string file_bt_xml;
        BT::Tree tree_;
        DroneBehaviorNode() : Node("drone_behavior")
        {
            this->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
            file_bt_xml = this->get_parameter("tree_xml_file").as_string();
        }

        void execute()
        {
            create_bt();
            timer_update_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&DroneBehaviorNode::update_time, this));
            rclcpp::spin(shared_from_this());
            rclcpp::shutdown();
        }

        void create_bt()
        {
            BT::BehaviorTreeFactory factory;
            auto node_ptr = this->shared_from_this();

            // Add Register ENUM
            factory.registerScriptingEnums<State>();

            // METHOD 1: Register node with lambda capturing node_ptr
            factory.registerBuilder<GoToPose>(
                "GoToPose",
                [node_ptr](const std::string &name, const BT::NodeConfig &config)
                {
                    return std::make_unique<GoToPose>(name, config, node_ptr);
                });

            factory.registerBuilder<DroneCheckBattery>(
                "CheckBattery",
                [node_ptr](const std::string &name, const BT::NodeConfig &config)
                {
                    return std::make_unique<DroneCheckBattery>(name, config, node_ptr);
                });

            factory.registerNodeType<ChargeBattery>("ChargeBattery");

            tree_ = factory.createTreeFromFile(file_bt_xml);
        }

        void update_time()
        {
            BT::NodeStatus tree_status = tree_.tickOnce();

            if (tree_status == BT::NodeStatus::RUNNING)
                return;
            if (tree_status == BT::NodeStatus::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Finished with Status SUCCESS");
                timer_update_->cancel();
            }
            else if (tree_status == BT::NodeStatus::FAILURE)
            {
                RCLCPP_INFO(this->get_logger(), "Finished with Status FAILURE");
                timer_update_->cancel();
            }
        }
    };

};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<drone_behavior::DroneBehaviorNode>();
    node->execute();
    return 0;
}
