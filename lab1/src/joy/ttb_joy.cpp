
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

    joy_publisher = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    
    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyController::joy_callback, this, std::placeholders::_1));

    // Timer object that controls how often your command loop function is called
    timer = this->create_wall_timer(
      100ms,                                           // Period of rate that function is called
      [this] (void) { this->command_loop_function(); } // Which function to call
    );
  }

private:


  // Function called repeatedly by node.
  void command_loop_function(void)
  {

    auto joy_msg = sensor_msgs::msg::Joy();
    joy_msg.header.stamp = this->get_clock()->now();

    joy_msg.axes = msg->axes;  
    joy_msg.buttons = msg->buttons; 

    joy_publisher->publish(joy_msg)
    RCLCPP_INFO(this->get_logger(), "Published Joy message with axes: [%f, %f, %f] and buttons: [%d, %d, %d]",
                     joy_msg.axes[0], joy_msg.axes[1], joy_msg.axes[2],
                     joy_msg.buttons[0], joy_msg.buttons[1], joy_msg.buttons[2]);

  }

  // ---------------------------------//
  // Variable used by the Node Object //
  // ---------------------------------//

  // ROS subsriber object
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

  // Controller Publisher/Subscriber
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

  // Control timing control
  rclcpp::TimerBase::SharedPtr timer;


};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotJoy>());
  rclcpp::shutdown();

  return 0;
}