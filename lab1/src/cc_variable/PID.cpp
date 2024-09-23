#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odom.hpp>

#include <chrono>

using namespace std::chrono_literals;

class TurtlebotPID : public rclcpp::Node
{
public:
  TurtlebotPID() 
      : Node("ttb_PID")
  {

    // Create Subscribe to the Odom data topic from Turtlebot
    imu_sub  = this->create_subscription<nav_msgs::msg::odom>(
      "TTB10/odom", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
      [this](const nav_msgs::msg::odom &msg) { this->odom_callback(msg); }
    );

    // Create Publisher for Turtlebot velocity commands
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("TTB10/cmd_vel", 10);

    // Timer object that controls how often your command loop function is called
    timer = this->create_wall_timer(
      100ms,                                           // Period of rate that function is called
      [this] (void) { this->command_loop_function(); } // Which function to call
    );
  }

private:

  // Callback function for receiving odom sensor data
  void odom_callback(const nav_msgs::msg::odom  &msg)
  {
    linear_velocity_x = msg.twist.twist.linear.x;
  }

  // Function called repeatedly by node.
  void command_loop_function(void)
  {
    //setting dt to 100ms
    std::chrono::duration<double> dt = 100ms;

    //calculating error
    double error = ref_velocity - linear_velocity_x;

    //calculating integral
    integral += error * dt.count();

    //calculating derivative
    double derivative = (error - prev_error) / dt.count();

    //calculating pid output
    double pid_output = kp * error + ki * integral + kd * derivative;

    //setting the previous error
    prev_error = error;

    //setting the velocity command
    vel_cmd.linear.x = pid_output;

    //logging the values
    RCLCPP_INFO(this->get_logger(), "Linear Vel: %0.3f", linear_velocity_x);

    //publishing the velocity command
    vel_pub->publish(vel_cmd);
  }
  
  // ---------------------------------//
  // Variable used by the Node Object //
  // ---------------------------------//

  // ROS subsriber object
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

  // ROS publisher object
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

  // Control timing control
  rclcpp::TimerBase::SharedPtr timer;

  // Velocity command message
  geometry_msgs::msg::Twist vel_cmd;

  // Variable to hold angular velocity data
  double angular_velocity_z;

  // Variable to hold linear velocity data
  double linear_velocity_x;

  // Variable to hold reference velocity
  double ref_velocity = 0.1;

  // Variable to hold kp, ki, kd values
  double kp = 0.1;
  double ki = 0.001;
  double kd = 0.03;


  //variable to hold previous error
  double prev_error = 0;

  //variable to hold integral
  double integral = 0;
  
  // Counter
  int count = 0;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotPID>());
  rclcpp::shutdown();

  return 0;
}