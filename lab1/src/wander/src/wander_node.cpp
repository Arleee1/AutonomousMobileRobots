#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

//TODO: Publish Object Detected Message

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
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create Publisher for Object Detected
    obj_detected_pub = this->create_publisher<std_msgs::msg::Bool>("object_detected", 10);

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

  // Callback function for receiving IR sensor data
  void ir_sensor_callback(const sensor_msgs::msg::Range &msg)
  {
    // If the range is less than 0.1 meters, an obstacle is detected
    if (msg.range < 0.1) {
      vel_cmd.angular.z = 1.0;   // Turn the robot
      vel_cmd.linear.x = 0.0;    // Stop moving forward
      vel_pub->publish(vel_cmd); // Start turning

      obj_detected.data = true;
      obj_detected_pub->publish(obj_detected); // Publish object detected message

      turn_time = getRandomInt(3, 12);  // 0.3s to 1.2s of obstacle avoidance

      RCLCPP_INFO(this->get_logger(), "Object Detected at %0.3f; Angular Vel: %0.3f", msg.range, 1.0);
    }
  }

  // Function called repeatedly by node.
  void command_loop_function(void)
  {
    if(turn_time > 0) {
      // Turn the robot
      vel_cmd.angular.z = 1.0;
      vel_cmd.linear.x = 0.0;
      turn_time--;
      float seconds = ((float) turn_time)/10.0;
      RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance: %0.1f seconds", seconds);

    } else if(const_speed > 0.0) {
      // No obstacle detected
      obj_detected.data = false;
      obj_detected_pub->publish(obj_detected); // Publish object not detected message

      // Move the robot forward
      vel_cmd.angular.z = 0.0;
      vel_cmd.linear.x = const_speed;
      RCLCPP_INFO(this->get_logger(), "Speed: %0.3f", const_speed);

    } else {
      return; // No command sent if speed is 0.0
    }

    vel_pub->publish(vel_cmd);
  }

  int getRandomInt(int min, int max) {
    // Random number generator
    std::random_device rd;  // Seed for random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine
    std::uniform_int_distribution<> distrib(min, max); // Distribution range

    return distrib(gen); // Returns a random integer between min and max (inclusive)
  }

  // ---------------------------------//
  // Variables used by the Node Object //
  // ---------------------------------//

  // ROS Parameter
  float const_speed;

  // ROS subscriber object
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub;

  // ROS publisher object
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obj_detected_pub;

  // Control timing
  rclcpp::TimerBase::SharedPtr timer;

  // Velocity command message
  geometry_msgs::msg::Twist vel_cmd;

  // Bool message for object detected
  std_msgs::msg::Bool obj_detected;

  // Time to turn to avoid obstacle
  int turn_time = 0;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WanderNode>());
  rclcpp::shutdown();

  return 0;
}
