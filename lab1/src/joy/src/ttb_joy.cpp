#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>


using namespace std::chrono_literals;


class TurtlebotJoy : public rclcpp::Node
{
public:
  TurtlebotJoy() 
      : Node("ttb_joy")
  {

    // Create Publisher for Turtlebot velocity commands

    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TurtlebotJoy::joy_callback, this, std::placeholders::_1));

      
    timer = this->create_wall_timer(
      100ms,                                           // Period of rate that function is called
      [this] (void) { this->vel_command_loop(); } // Which function to call
    );
    

  }

private:


  // Function called repeatedly by node.
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {

    auto joy_msg = sensor_msgs::msg::Joy();
     // take values from here, make them 

    joy_msg.axes = msg->axes;  
    joy_msg.buttons = msg->buttons; 

    vel_x = joy_msg.axes[1];
    vel_z = joy_msg.axes[0];

  // take joy msg from axes, transfer into velocity, publish that velocity (twist)

  }

  void vel_command_loop(void)
  {
    
    cmd_vel_msg.linear.x = vel_x;
    cmd_vel_msg.angular.z = vel_z;
    RCLCPP_INFO(this->get_logger(), "Linear Vel: %0.3f", cmd_vel_msg.linear.x);
    RCLCPP_INFO(this->get_logger(), "Angular Vel: %0.3f", cmd_vel_msg.angular.z);

    cmd_vel_publisher->publish(cmd_vel_msg);

  }


  // ---------------------------------//
  // Variable used by the Node Object //
  // ---------------------------------//

  // Controller Subscriber

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

  // Velocity Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  
  geometry_msgs::msg::Twist cmd_vel_msg;

  // Control timing control
  rclcpp::TimerBase::SharedPtr timer;

  double vel_x = 0;

  double vel_z = 0;


};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotJoy>());
  rclcpp::shutdown();

  return 0;
}