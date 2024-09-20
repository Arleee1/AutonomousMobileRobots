#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <chrono>

using namespace std::chrono_literals;

class WanderNode : public rclcpp::Node
{
public:
  WanderNode() 
      : Node("wander_node")
  {
    // Subscribe to the IR range data topic
    range_sub  = this->create_subscription<sensor_msgs::msg::Range>(
      "ir_range", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
      [this](const sensor_msgs::msg::Range &msg) { this->ir_sensor_callback(msg); }
    );

    // Create Publisher for Turtlebot velocity commands
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("TTB10/cmd_vel", 10);

    // Timer object that controls how often your command loop function is called
    timer = this->create_wall_timer(
      100ms,                                           // Period of rate that function is called
      [this] (void) { this->command_loop_function(); } // Which function to call
    );

    // Constant Speed Command Parameter 
    // Speed set if no obstacle detected; No command sent if speed will be 0.0
    this->declare_parameter<float>("const_speed", 0.3);
    
    // Get the value of Constant Speed Command Parameter 
    this->get_parameter("const_speed", const_speed);

    RCLCPP_INFO(this->get_logger(), "If no object detected using constant speed: %0.3f", const_speed);
  }

private:
  float const_speed;  // Parameter const_speed

  // Callback function for receiving IR sensor data
  void ir_sensor_callback(const sensor_msgs::msg::Range &msg)
  {
    // If the range is less than 0.1 meters, an obstacle is detected
    if (msg.range < 0.1) {
      obstacle_detected = true;
      vel_cmd.angular.z = 0.5;  // Turn the robot
      vel_cmd.linear.x = 0.0;   // Stop moving forward
      vel_pub->publish(vel_cmd);
      RCLCPP_INFO(this->get_logger(), "Object Detected; Angular Vel: %0.3f", 0.5);
    } else {
      obstacle_detected = false;  // No obstacle detected
    }
  }

  // Function called repeatedly by node.
  void command_loop_function(void)
  {
    if(obstacle_detected) {
      vel_cmd.angular.z = 0.5;
      vel_cmd.linear.x = 0.0;
    } else if(const_speed > 0.0) {
      vel_cmd.angular.z = 0.0;
      vel_cmd.linear.x = const_speed;
      RCLCPP_INFO(this->get_logger(), "Speed: %0.3f", const_speed);
    } else {
      return; // No command sent if speed is 0.0
    }

    vel_pub->publish(vel_cmd);
  }

  // ---------------------------------//
  // Variables used by the Node Object //
  // ---------------------------------//

  // ROS subscriber object
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub;

  // ROS publisher object
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

  // Control timing
  rclcpp::TimerBase::SharedPtr timer;

  // Velocity command message
  geometry_msgs::msg::Twist vel_cmd;

  // Variable to track whether an obstacle is detected
  bool obstacle_detected = false;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WanderNode>());
  rclcpp::shutdown();

  return 0;
}
