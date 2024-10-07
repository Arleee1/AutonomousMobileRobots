#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include "../../../lib/include/DPID.h"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/utils.h"


using namespace std::chrono_literals;

class TTBPIDNode : public rclcpp::Node
{
public:
  TTBPIDNode() 
   : Node("ttb_pid_node")
  { 

    this->declare_parameter("kp", 0.1);
    this->declare_parameter("ki", 0.001);
    this->declare_parameter("kd", 0.03);
    kp = this->get_parameter("kp").as_double();
    ki = this->get_parameter("ki").as_double();
    kd = this->get_parameter("kd").as_double();


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
    if (initial == true) {
      init_x = msg.pose.pose.position.x;
      init_y = msg.pose.pose.position.y;
      goal_x = goal_x + init_x; // add relative position to goal (defined below)
      goal_y = goal_y + init_y;

      tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // we only need yaw but good to future proof
      init_yaw = yaw;
      goal_yaw = atan2(goal_y - init_y, goal_x - init_y) * (180.0 / M_PI); // to degrees
      initial = false;
    }

    curr_x = msg.pose.pose.position.x;
    curr_y = msg.pose.pose.position.y;
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // we only need yaw but good to future proof
    curr_yaw = yaw;

  }

  // PID control loop function
  void command_loop_function(void)
  {
      // Use fixed dt of 0.1 seconds (100 ms)
      double dt = 0.1;

      // Set the velocity command
      if (orient == true) {
        vel_cmd.linear.x = 0;
        error = ang_PID.compute(curr_yaw - goal_yaw, dt);
        if(error < .01) { //if goal reached, stop and go!
          vel_cmd.angular.z = 0;
          orient = false;
        } else {
          vel_cmd.angular.z = error;
        }
      } else { 
        vel_cmd.linear.x = ref_velocity; // change to whatever speed you'd like. reference velocity probably
        vel_cmd.angular.z = 0;
      if (curr_x - init_x >= goal_x && curr_y - init_y) { // if goal reached, stop!
          vel_cmd.linear.x = 0;
        }
      }

      // Log the values (can reduce logging frequency)
      if (count % 10 == 0) {  // Logs once every 10 cycles (1 second)
          RCLCPP_INFO(this->get_logger(), "Linear Vel: %0.3f", vel_cmd.linear.x);
          RCLCPP_INFO(this->get_logger(), "Rotational Vel: %0.3f", vel_cmd.angular.z);
          RCLCPP_INFO(this->get_logger(), "Odom X: %0.3f", curr_x);
          RCLCPP_INFO(this->get_logger(), "Odom Y: %0.3f", curr_y);
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

  double kp = .1;
  double ki = .001;
  double kd = .03;
  DPID ang_PID(double kp, double ki, double kd);

  // Get initial pos of odom only once
  bool initial = true;
  bool orient = true;
  double init_x;
  double init_y;
  double init_yaw; 
  double goal_x = 1.5;
  double goal_y = 1.5;
  double goal_yaw;

  // Current position
  double curr_x;
  double curr_y;
  double curr_yaw;
  

  double ref_velocity;
  double error;
  
  // Tracking variable for logging
  int count = 0;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TTBPIDNode>());
  rclcpp::shutdown();

  return 0;
}
