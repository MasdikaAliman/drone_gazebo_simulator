#include "PID.hpp"
#include "drone_msgs/action/move_to_pose_drone.hpp"
#include "functional"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "thread"
#include <memory>

/*
Publisher : drone/cmd_vel (geometry_msgs/msg/Twist), drone/enable
(std_msgs/msg/Bool) Subscriber : odom (nav_msgs/msg/Odometry) Action Server :
move_to_pose_drone (drone_msgs/action/MoveToPoseDrone)


*/

enum State
{
  IDLE = 0,
  AUTOMATIC = 1,
  TAKING_OFF = 2,
  LANDING = 3,
  EMERGENCY_STOP = 4
};

using MoveToPose = drone_msgs::action::MoveToPoseDrone;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

namespace drone_action
{

  class MoveToPoseServer : public rclcpp::Node
  {
  public:
    MoveToPoseServer()
        : Node("move_to_pose_server")

    {
      using namespace std::placeholders;

      action_server_ = rclcpp_action::create_server<MoveToPose>(
          this, "move_to_pose_drone",
          std::bind(&MoveToPoseServer::handle_goal, this, _1, _2),
          std::bind(&MoveToPoseServer::handle_cancel, this, _1),
          std::bind(&MoveToPoseServer::handle_accepted, this, _1));

      enable_vel_publisher_ =
          this->create_publisher<std_msgs::msg::Bool>("drone/enable", 10);

      cmd_vel_publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 10);
      odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&MoveToPoseServer::odom_callback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Action Server Start Now ...");
    }

  private:
    int frequncy_loop_ = 20;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

    nav_msgs::msg::Odometry current_odom_;
    std::shared_ptr<GoalHandleMoveToPose> current_goal_handle_;
    std::mutex goal_mutex_;
    std::atomic<bool> should_stop_{false};
    bool takeoff_complete = false;

    PID pid_x_{1.0, 0.0, 0.2, -1.0, 1.0, -0.5, 0.5, 1.0 / this->frequncy_loop_,
               "X", false};
    PID pid_y_{1.0, 0.0, 0.2, -1.0, 1.0, -0.5, 0.5, 1.0 / this->frequncy_loop_,
               "Y", false};
    PID pid_z_{1.5, 0.0, 0.3, -1.0, 1.0, -0.5, 0.5, 1.0 / this->frequncy_loop_,
               "Z", false};

    PID pid_yaw_{1.5, 0.0, 0.3, -0.5,
                 0.5, -0.5, 0.5, 1.0 / this->frequncy_loop_,
                 "yaw", false};

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Process odometry data
      current_odom_ = *msg;
      // RCLCPP_INFO(this->get_logger(), "Received odometry: Position(%.2f, %.2f,
      // %.2f)",
      //             current_odom_.pose.pose.position.x,
      //             current_odom_.pose.pose.position.y,
      //             current_odom_.pose.pose.position.z);
    }

    // ROS2 ACTION CALLBACKS
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const MoveToPose::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Received goal request with target pose x: %.2f, y: %.2f, z: "
                  "%.2f, yaw: %.2f, state: %d",
                  goal->target_pose.x, goal->target_pose.y, goal->target_pose.z,
                  goal->target_pose.yaw, goal->target_pose.state_type);
      (void)uuid;

      // Signal current goal to stop
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (current_goal_handle_ && current_goal_handle_->is_executing())
        {
          RCLCPP_INFO(this->get_logger(), "Preempting current goal");
          should_stop_.store(true);

          // Wait a bit for current goal to stop
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void
    handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
      std::thread{[this, goal_handle]()
                  { this->execute(goal_handle); }}
          .detach();
    }

    void droneCommand(geometry_msgs::msg::Twist &cmd_vel,
                      std_msgs::msg::Bool &enable)
    {
      cmd_vel_publisher_->publish(cmd_vel);
      enable_vel_publisher_->publish(enable);
    }

    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_ = goal_handle;
        should_stop_.store(false);
      }

      RCLCPP_INFO(this->get_logger(), "Executing Goal...");
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<MoveToPose::Feedback>();
      auto result = std::make_shared<MoveToPose::Result>();

      auto start_time = this->now();
      auto cmd_vel_msg = geometry_msgs::msg::Twist();
      auto enable_vel_msg = std_msgs::msg::Bool();
      enable_vel_msg.data = goal->target_pose.state_type == IDLE ||
                                    goal->target_pose.state_type == EMERGENCY_STOP
                                ? false
                                : true;
      enable_vel_publisher_->publish(enable_vel_msg);
      rclcpp::Rate loop_rate(frequncy_loop_);

      while (rclcpp::ok())
      {

        if (goal->target_pose.state_type == IDLE ||
            goal->target_pose.state_type == EMERGENCY_STOP)
        {
          enable_vel_msg.data = false;
          enable_vel_publisher_->publish(enable_vel_msg);
          return;
        }

        double target_z = fabs(goal->target_pose.z);

        if (goal->target_pose.z <= 0 &&
            goal->target_pose.state_type == TAKING_OFF)
        {
          // Default Target
          target_z = 1.0;
        }

        if ((goal->target_pose.z <= 0) &&
            goal->target_pose.state_type == LANDING)
        {
          target_z = 0.05;
        }

        double error_x = goal->target_pose.x - current_odom_.pose.pose.position.x;
        double error_y = goal->target_pose.y - current_odom_.pose.pose.position.y;
        double error_z = target_z - current_odom_.pose.pose.position.z;
        // double error_z = fabs(goal->target_pose.z) -
        // current_odom_.pose.pose.position.z;

        // Compute errors

        tf2::Quaternion q(current_odom_.pose.pose.orientation.x,
                          current_odom_.pose.pose.orientation.y,
                          current_odom_.pose.pose.orientation.z,
                          current_odom_.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double error_yaw = (goal->target_pose.yaw * M_PI / 180.0) - yaw;

        while (error_yaw > M_PI)
          error_yaw -= 2 * M_PI;
        while (error_yaw < -M_PI)
          error_yaw += 2 * M_PI;

        // RCLCPP_INFO(this->get_logger(),
        //             "Error X: %.2f | Y: %.2f | Z: %.2f | Yaw: %.2f", error_x,
        //             error_y, error_z, error_yaw);

        RCLCPP_INFO(this->get_logger(),
                    "Target X: %.2f | Y: %.2f | Z: %.2f | Yaw: %.2f", goal->target_pose.x,
                    goal->target_pose.y, goal->target_pose.z, goal->target_pose.yaw);

        if (should_stop_.load())
        {
          RCLCPP_INFO(this->get_logger(), "Stopping for new goal");
          result->success = false;
          goal_handle->abort(result);

          cmd_vel_msg = geometry_msgs::msg::Twist();
          droneCommand(cmd_vel_msg, enable_vel_msg);
          return;
        }

        static int stable_counter = 0;
        if ((sqrt(error_x * error_x + error_y * error_y + error_z * error_z) <
                 0.05 &&
             fabs(error_yaw) < 0.08))
        {
          stable_counter++;
          RCLCPP_INFO(this->get_logger(), "Counting Reach: %d", stable_counter);
          if (stable_counter > 5)
          { // 1 second stable
            RCLCPP_INFO(this->get_logger(), "Goal Reached");
            result->success = true;
            goal_handle->succeed(result);
            // is_executing_.store(false);
            pid_x_.reset();
            pid_y_.reset();
            pid_z_.reset();
            pid_yaw_.reset();

            enable_vel_msg.data =
                goal->target_pose.state_type == LANDING ? false : true;

            cmd_vel_msg = geometry_msgs::msg::Twist();
            droneCommand(cmd_vel_msg, enable_vel_msg);
            return;
          }
        }
        else
        {
          stable_counter = 0;
        }

        if (goal_handle->is_canceling())
        {
          RCLCPP_INFO(this->get_logger(), "Goal Canceled");
          // result->success = false;
          goal_handle->canceled(result);
          cmd_vel_msg = geometry_msgs::msg::Twist();
          enable_vel_msg = std_msgs::msg::Bool();
          droneCommand(cmd_vel_msg, enable_vel_msg);

          return;
        }
        // Global -> Body Frame

        double cy = cos(yaw);
        double sy = sin(yaw);

        // PID computations
        double ex = cy * error_x + sy * error_y;
        double ey = -sy * error_x + cy * error_y;

        double control_x = -pid_x_.update(0.0, ex);
        double control_y = -pid_y_.update(0.0, ey);
        double control_z = -pid_z_.update(0.0, error_z);
        double control_yaw = -pid_yaw_.update(0.0, error_yaw);

        // Publish cmd_vel
        cmd_vel_msg.linear.x = control_x;
        cmd_vel_msg.linear.y = control_y;
        cmd_vel_msg.linear.z = control_z;
        cmd_vel_msg.angular.z = control_yaw;

        // Publish feedback
        feedback->error_x = error_x;
        feedback->error_y = error_y;
        feedback->error_z = error_z;

        droneCommand(cmd_vel_msg, enable_vel_msg);
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
      }
    }
  };

} // namespace drone_action

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_action::MoveToPoseServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
