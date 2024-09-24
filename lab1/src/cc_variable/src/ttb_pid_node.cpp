#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>

using namespace std::chrono_literals;

class TTBPIDNode : public rclcpp::Node
{
public:
  TTBPIDNode() 
   : Node("ttb_PID_Node"),
     linear_velocity_x(0.0),  
     angular_velocity_z(0.0), 
     ref_velocity(0.1),       
     prev_error(0.0),         
     integral(0.0)           
{
    // Declare and read PID parameters: kp, ki, kd
    this->declare_parameter("kp", 0.1);
    this->declare_parameter("ki", 0.001);
    this->declare_parameter("kd", 0.03);
    kp = this->get_parameter("kp").as_double();
    ki = this->get_parameter("ki").as_double();
    kd = this->get_parameter("kd").as_double();

    // create Subscriber for target velocity
    target_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "target_vel", 
      10,
      [this](const geometry_msgs::msg::Twist &msg) { this->target_vel_callback(msg); }
    );

    // Create Subscribe to the Odom data topic from Turtlebot
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
    linear_velocity_x = msg.twist.linear.x;
  }

  // PID control loop function
  void command_loop_function(void)
  {
      // Use fixed dt of 0.1 seconds (100 ms)
      double dt = 0.1;

      // Calculate error
      double error = ref_velocity - linear_velocity_x;

      // Calculate integral
      integral += error * dt;

      // Calculate derivative
      double derivative = (error - prev_error) / dt;

      // Calculate PID output
      double pid_output = kp * error + ki * integral + kd * derivative;

      // Set the previous error
      prev_error = error;

      // Set the velocity command
      vel_cmd.linear.x = pid_output;
      vel_cmd.angular.z = 0;

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

  // ROS subsriber object
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

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

  // tracking variable for logging
  int count = 0;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TTBPIDNode>());
  rclcpp::shutdown();

  return 0;
}
