#include <memory>

#include "PID.hpp"
#include "drone_msgs/msg/target_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "icecream.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
using std::placeholders::_1;

enum State {
  IDLE = 0,
  AUTOMATIC = 1,
  TAKING_OFF = 2,
  LANDING = 3,
  EMERGENCY_STOP = 4
};

namespace drone_controller {
class DroneControllerNode : public rclcpp::Node {
public:
  DroneControllerNode() : Node("drone_controller_node") {

    this->declare_parameter("frequency_loop", 50);
    this->declare_parameter<std::string>("robot_name", "drone");
    this->declare_parameter<double>("takeoff_threshold", 0.04);
    this->declare_parameter<double>("landing_threshold", 0.05);
    this->declare_parameter<double>("takeoff_height", 1.0);

    frequncy_loop_ = this->get_parameter("frequency_loop").as_int();
    robot_name_ = this->get_parameter("robot_name").as_string();
    takeoff_threshold_ = this->get_parameter("takeoff_threshold").as_double();
    landing_threshold_ = this->get_parameter("landing_threshold").as_double();
    takeoff_height_ = this->get_parameter("takeoff_height").as_double();

    auto qos = rclcpp::QoS(10);

    cmd_enable_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("drone/enable", qos);
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", qos);
    // cmd_des_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
    //     "drone/setpoint_pose", qos);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos, std::bind(&DroneControllerNode::odom_callback, this, _1));
    sub_goal_ = this->create_subscription<drone_msgs::msg::TargetPose>(
        "goal_pose", qos,
        std::bind(&DroneControllerNode::goal_callback, this, _1));

    sub_state_ = this->create_subscription<std_msgs::msg::Int32>(
        "drone/state", qos,
        std::bind(&DroneControllerNode::state_callback, this, _1));

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1 / this->frequncy_loop_),
        std::bind(&DroneControllerNode::controller_loop, this));
  }

  void controller_loop();

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Received odom position: [%.2f, %.2f, %.2f]",
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Yaw: %.2f", msg->pose.pose.orientation.z);
    current_pose_ = *msg;
  }

  void goal_callback(const drone_msgs::msg::TargetPose::SharedPtr msg) {

    desired_pose_ = *msg;
    current_state_ = static_cast<State>(msg->state_type);
  }

  void state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    current_state_ = static_cast<State>(msg->data);
  }

private:
  // PID class
  PID pid_x_{1.0, 0.0,  0.2, -1.0, 1.0, -0.5, 0.5, 1.0 / this->frequncy_loop_,
             "X", false};
  PID pid_y_{1.0, 0.0,  0.2, -1.0, 1.0, -0.5, 0.5, 1.0 / this->frequncy_loop_,
             "Y", false};
  PID pid_z_{1.5, 0.0,  0.3, -1.0, 1.0, -0.5, 0.5, 1.0 / this->frequncy_loop_,
             "Z", false};

  PID pid_yaw_{1.5,   0.0,  0.3, -0.5,
               0.5,   -0.5, 0.5, 1.0 / this->frequncy_loop_,
               "yaw", false};

  int frequncy_loop_ = 50;
  double takeoff_threshold_, landing_threshold_, takeoff_height_;
  std::string robot_name_;

  State current_state_ = IDLE;
  nav_msgs::msg::Odometry current_pose_;
  drone_msgs::msg::TargetPose desired_pose_;

  bool takeoff_complete = false;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_enable_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_des_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<drone_msgs::msg::TargetPose>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_state_;
};

void DroneControllerNode::controller_loop() {
  if (this->current_state_ == TAKING_OFF) {
    RCLCPP_INFO(this->get_logger(), "Taking Off..");
    double current_z = this->current_pose_.pose.pose.position.z;
    auto enable_msg = std_msgs::msg::Bool();
    if (not takeoff_complete) {
      if (current_z >= (this->takeoff_height_ - 0.05)) {
        geometry_msgs::msg::Twist msg;

        enable_msg.data = true;
        cmd_enable_publisher_->publish(enable_msg);
        msg.linear.z = pid_z_.update(this->takeoff_height_, current_z);
        cmd_vel_publisher_->publish(msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Takeoff Complete!");
        takeoff_complete = true;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Hovering at takeoff height.");
      geometry_msgs::msg::Twist msg;
      enable_msg.data = true;
      cmd_enable_publisher_->publish(enable_msg);
      msg.linear.z = pid_z_.update(this->takeoff_height_, current_z);
      cmd_vel_publisher_->publish(msg);
    }
  }

  else if (this->current_state_ == LANDING) {
    RCLCPP_INFO(this->get_logger(), "Landing...");
    double current_z = this->current_pose_.pose.pose.position.z;
    auto msg = geometry_msgs::msg::Twist();
    if (current_z > this->landing_threshold_) {
      auto enable_msg = std_msgs::msg::Bool();
      enable_msg.data = true;
      cmd_enable_publisher_->publish(enable_msg);
      msg.linear.z = pid_z_.update(0.0, current_z);
      cmd_vel_publisher_->publish(msg);
    } else {
      RCLCPP_INFO(this->get_logger(), "Landed Success Turrn Off Motors");
      current_state_ = IDLE;
      auto enable_msg = std_msgs::msg::Bool();
      enable_msg.data = false;
      cmd_enable_publisher_->publish(enable_msg);
      cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
    }
  }

  else if (this->current_state_ == AUTOMATIC) {
    RCLCPP_INFO(this->get_logger(), "AUTOMATIC State..");
    // tf2::Quaternion q_goal(
    //     desired_pose_.orientation.x, desired_pose_.orientation.y,
    //     desired_pose_.orientation.z, desired_pose_.orientation.w);

    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q_goal).getRPY(roll, pitch, yaw);

    double g_x, g_y, g_z, g_yaw;

    g_x = desired_pose_.x;
    g_y = desired_pose_.y;
    g_z = desired_pose_.z;
    g_yaw = desired_pose_.yaw;

    // g_x = 1.0;
    // g_y = 0.0;
    // g_z = 1.0;
    // g_yaw = 0.0;

    tf2::Quaternion q(
        current_pose_.pose.pose.orientation.x, current_pose_.pose.pose.orientation.y,
        current_pose_.pose.pose.orientation.z, current_pose_.pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);


    double rad_yaw = g_yaw * M_PI / 180.0;

    // wrap
    double yaw_error = rad_yaw - yaw;
    while (yaw_error > M_PI)
      yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI)
      yaw_error += 2 * M_PI;

    // position error (global)
    double dx = g_x - current_pose_.pose.pose.position.x;
    double dy = g_y - current_pose_.pose.pose.position.y;
    double dz = g_z - current_pose_.pose.pose.position.z;

    // global → body
    double cy = cos(yaw);
    double sy = sin(yaw);

    double ex = cy * dx + sy * dy;
    double ey = -sy * dx + cy * dy;

    auto msg = geometry_msgs::msg::Twist();
    // control
    msg.linear.x = -pid_x_.update(0.0, ex);
    msg.linear.y = -pid_y_.update(0.0, ey);
    msg.linear.z = -pid_z_.update(0.0, dz);
    msg.angular.z = -pid_yaw_.update(0.0, yaw_error);

    IC(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);

    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = true;
    cmd_enable_publisher_->publish(enable_msg);
    cmd_vel_publisher_->publish(msg);
  }

  else if (this->current_state_ == IDLE ||
           this->current_state_ == EMERGENCY_STOP) {
    RCLCPP_INFO(this->get_logger(),
                "IDLE or EMERGENCY STOP State. Motors Off.");
    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = false;
    cmd_enable_publisher_->publish(enable_msg);
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
  }
}

}; // namespace drone_controller

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_controller::DroneControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
