#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <chrono>

using namespace std::chrono_literals;

/*
 * This Node is the state machine for the Turtlebot
    * Subscriptions:
        * TTB10 Remote:
            * joy topic
                * joystick buttons: Used to switch between states
                    * teleop: right bumper
                    * Cruise Control target speed:
                        * Square: 0.1 m/s
                        * Triangle (2): 0.2 m/s
                        * circle: 0.4 m/s
                        * x: 0.0 m/s (stop)
        * Joy:
            * teleop/cmd_vel topic
        * Wander:
            * object_detected topic
                * if true, switch to wander state
            * wander/cmd_vel topic
        * Cruise Control:
            * cc/cmd_vel topic

    * Publishes:
        * cmd_vel topic
            * Publishes velocity commands to the Turtlebot
            * based on the current state and priority:
                * 1) teleop: highest priority
                    * if teleop is active
                        * publish teleop/cmd_vel to cmd_vel
                * 2) Wander: second highest priority
                    * if object_detected is true
                        * publish wander/cmd_vel to cmd_vel
                * 3) Cruise Control: lowest priority
                    * if no other state is active
                        * publish cc/cmd_vel to cmd_vel
            * Sets crusie control target speed based on joystick buttons
                * buttons:
                    * Square: 0.1 m/s
                    * Triangle (2): 0.2 m/s
                    * circle: 0.4 m/s
                    * x: 0.0 m/s (stop)
                * Publishes the target speed to target_vel
 */
class state_machine_Node : public rclcpp::Node
{
public:
  state_machine_Node() 
      : Node("state_machine_Node")
    {

    // Create Publisher for Turtlebot velocity commands
    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("TTB10/cmd_vel", 10);

    // Create Subscriber for Turtlebot joy messages
    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "TTB10/joy", 10, std::bind(&state_machine_Node::joy_callback, this, std::placeholders::_1));

    // Create Subscriber for Turtlebot wander messages
    wander_subscriber = this->create_subscription<sensor_msgs::msg::Range>(
            "TTB10/object_detected", 10, std::bind(&state_machine_Node::wander_callback, this, std::placeholders::_1));

    // Create Subscriber for Turtlebot cruise control messages
    cc_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "TTB10/cc/cmd_vel", 10, std::bind(&state_machine_Node::cc_callback, this, std::placeholders::_1));

    // Create Timer for state machine loop
    timer = this->create_wall_timer(
      100ms,                                           // Period of rate that function is called
      [this] (void) { this->state_machine_loop(); } // Which function to call
    );

    // Initialize state variables
    teleop_active = false;
    wander_active = false;

    // Initialize target velocity
    target_vel.linear.x = 0.0;
    target_vel.angular.z = 0.0;

    }
    
private:

    // Function called repeatedly by node.
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // take values from here, make them 
        auto joy_msg = sensor_msgs::msg::Joy();
        joy_msg.axes = msg->axes;  
        joy_msg.buttons = msg->buttons; 
    
        // take joy msg from axes, transfer into velocity, publish that velocity (twist)
        vel_x = joy_msg.axes[1];
        vel_z = joy_msg.axes[0];
    
        teleop_active = joy_msg.buttons[5];
    }
    
    void wander_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        wander_active = msg->range < 0.5;
    }
    
    void cc_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cc_active = true;
        target_vel = *msg;
    }
    
    void state_machine_loop(void)
    {
        // Set velocity based on state
        if (teleop_active)
        {
        cmd_vel_msg.linear.x = vel_x;
        cmd_vel_msg.angular.z = vel_z;
        }
        else if (wander_active)
        {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        }
        else if (cc_active)
        {
        cmd_vel_msg = target_vel;
        }
    
        // Publish velocity command
        cmd_vel_publisher->publish(cmd_vel_msg);
    }
    
    // ---------------------------------//
    // Variable used by the Node Object //
    // ---------------------------------//
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr wander_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cc_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    
    geometry_msgs::msg::Twist cmd_vel_msg;
    geometry_msgs::msg::Twist target_vel;
    
    bool teleop_active;
    bool wander_active;
    bool cc_active;
    
    float vel_x;
    float vel_z;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WanderNode>());
  rclcpp::shutdown();

  return 0;
}
