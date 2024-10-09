#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include "DPID.h"
#include <vector>
#include <utility>

using namespace std::chrono_literals;

class GoToMultiplePointsNode : public rclcpp::Node
{
public:
  GoToMultiplePointsNode() 
   : Node("goto_multiple_points_node")
  {
    // Declare and read PID parameters: kp, ki, kd
    this->declare_parameter("kp", 0.1);
    this->declare_parameter("ki", 0.001);
    this->declare_parameter("kd", 0.03);
    kp = this->get_parameter("kp").as_double();
    ki = this->get_parameter("ki").as_double();
    kd = this->get_parameter("kd").as_double();

    // create a PID object
    DPID angularPID(kp, ki, kd);

    // Goal Points
    this->declare_parameter<std::vector<double>>("goal_points", std::vector<double>{});
    auto goal_points_vector = this->get_parameter("goal_points").as_double_array();
    for (size_t i = 0; i < goal_points_vector.size(); i += 2) {
      goal_points_.emplace_back(goal_points_vector[i], goal_points_vector[i + 1]);
    }

    // Create Subscriber for target velocity
    target_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "target_vel", 
      10,
      [this](const geometry_msgs::msg::Twist &msg) { this->target_vel_callback(msg); }
    );

    // Create Subscriber for the Odom data topic from Turtlebot
    odom_sub  = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
      [this](const nav_msgs::msg::Odometry &msg) { this->odom_callback(msg); }
    );

    // Create Publisher for Turtlebot velocity commands
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Timer object that controls how often your command loop function is called
    timer = this->create_wall_timer(
      100ms,                                           // Period of rate that function is called
      [this] (void) { this->command_loop_function(); } // Which function to call
    );
  }

private:

  // Callback function for receiving target velocity data
  void target_vel_callback(const geometry_msgs::msg::Twist &msg)
  {
    ref_velocity = msg.linear.x;
  }

  // Callback function for receiving odom sensor data
  void odom_callback(const nav_msgs::msg::Odometry &msg)
  {
    /*
    If the robot is at the current goal point, update the goal point to the next point in the list.
    Update the error term for the PID controller. Based on current orentation and position of the robot.
    */
    
    // Get the current pose of the robot
    x_curr = msg.pose.pose.position.x;
    y_curr = msg.pose.pose.position.y;
    theta_current = msg.pose.pose.orientation.z;

    // Get the goal position of the robot
    x_goal = goal_points_[goal_point_index].first;
    y_goal = goal_points_[goal_point_index].second;

    // Check if the robot is at the current goal point
    if (sqrt(pow(x_curr - x_goal, 2) + pow(y_curr - y_goal, 2)) < goal_tolerance) {
      // Update the goal point index
      goal_point_index = (goal_point_index + 1) % goal_points_.size();
      // Get the goal position of the robot
      x_goal = goal_points_[goal_point_index].first;
      y_goal = goal_points_[goal_point_index].second;
    }

    // Caluclate theta_goal based on the current goal point and current position of the robot
    theta_goal = atan2(y_goal - y_curr, x_goal - x_curr);

    // Update bounded error based on theta_goal and theta_current
    bounded_error = atan2(sin(theta_goal - theta_current), cos(theta_goal - theta_current));
    
  }

  // PID control loop function
  void command_loop_function(void)
  {

    [CC_Steer_PID, steer_cmd] = CC_Steer_PID.compute(theta_error_bounded, dt);
    steer_angl = min(abs(steer_cmd), steer_angl_max) * sign(steer_cmd);

    // Calculate the PID output
    double steer_cmd = angularPID.compute(bounded_error, 0.1);

    // // bound the steering command
    // steer_cmd = std::min(std::abs(steer_cmd), 0.5) * std::copysign(1.0, steer_cmd);

    // Set the velocity command
    vel_cmd.linear.x = pid_output;
    vel_cmd.angular.z = 0.0;

    // Log the values (can reduce logging frequency)
    if (count % 10 == 0) {  // Logs once every 10 cycles (1 second)
        RCLCPP_INFO(this->get_logger(), "Linear Vel: %0.3f", linear_velocity_x);
    }

    // Increment the counter to reduce logging frequency
    count++;
    if (count > 1000000) {
      count = 0;
    }

    // Publish the velocity command
    vel_pub->publish(vel_cmd);
  }
  
  // ---------------------------------//
  // Variable used by the Node Object //
  // ---------------------------------//

  // ROS subscriber object for target velocity
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_vel_sub;

  // ROS subscriber object for odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  // ROS publisher object for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr timer;

  // Velocity command message
  geometry_msgs::msg::Twist vel_cmd;

  // Variable to hold reference velocity
  double ref_velocity = 0.1;

  // Vector to hold goal points
  std::vector<std::pair<double, double>> goal_points_;

  // Currrent goal point index
  size_t goal_point_index = 0;

  // Goal Tolerance
  double goal_tolerance = 0.2;

  // PID error input
  double bounded_error = 0.0;

  // Tracking variable for logging
  int count = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToMultiplePointsNode>());
  rclcpp::shutdown();

  return 0;
}
