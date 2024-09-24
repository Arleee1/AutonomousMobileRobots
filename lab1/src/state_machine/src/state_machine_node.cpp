#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include "std_msgs/msg/bool.hpp"
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
                    * teleop: right bumper (5)
                    * Cruise Control target speed:
                        * Square (3): 0.1 m/s
                        * Triangle (2): 0.2 m/s
                        * circle (1): 0.4 m/s
                        * x (0): 0.0 m/s (stop)
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
                    * Square (3): 0.1 m/s
                    * Triangle (2): 0.2 m/s
                    * circle (1): 0.4 m/s
                    * x (0): 0.0 m/s (stop)
                * Publishes the target speed to target_vel
 */
class state_machine_Node : public rclcpp::Node
{
public:
  state_machine_Node() 
      : Node("state_machine_Node")
  {
    // Subscriptions:

    // Create Subscriber for Turtlebot joy messages
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 
        10, 
        [this](const sensor_msgs::msg::Joy &msg) { this->joy_callback(msg); }
    );

    // Create Subscriber for teleop cmd_vel messages
    teleop_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "teleop/cmd_vel", 
        10, 
        [this](const geometry_msgs::msg::Twist &msg) { this->teleop_callback(msg); }
    );

    // Create Subscriber for Object Detected messages
    obj_detected_sub = this->create_subscription<std_msgs::msg::Bool>(
        "object_detected", 
        10, 
        [this](const std_msgs::msg::Bool &msg) { this->obj_detected_callback(msg); }
    );

    // Create Subscriber for wander cmd_vel messages
    wander_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "wander/cmd_vel", 
        10, 
        [this](const geometry_msgs::msg::Twist &msg) { this->wander_callback(msg); }
    );

    // Create Subscriber for cruise control cmd_vel messages
    cc_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cc/cmd_vel", 
        10, 
        [this](const geometry_msgs::msg::Twist &msg) { this->cc_callback(msg); }
    );

    // Publishers:

    // Create Publisher for Turtlebot velocity commands
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // create publisher for target velocity
    target_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("target_vel", 10);

    // ---------------------------------//

    // Create Timer for state machine loop
    timer = this->create_wall_timer(
      50ms,                                           // Period of rate that function is called
      [this] (void) { this->state_machine_loop(); } // Which function to call
    );

    // Initialize state variables
    teleop_active = false;
    wander_active = false;

    // Initialize cruise control target speed
    target_vel.linear.x = 0.0;
    target_vel.angular.z = 0.0;

    // Initialize velocity command message
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    // Initialize teleop velocity variables
    teleop_x = 0.0;
    teleop_z = 0.0;

    // Initialize wander velocity variables
    wander_x = 0.0;
    wander_z = 0.0;

    // Initialize cruise control velocity variables
    cc_x = 0.0;
    cc_z = 0.0;

  }
    
private:

    /* 
        * Determain state based on joystick input
    */ 
    void joy_callback(const sensor_msgs::msg::Joy &msg) {
        // Check for teleop state
        if (msg->buttons[5] == 1) { // Right bumper
            teleop_active = true;
        }

        // Set target velocity based on joystick input
        if (msg->buttons[3] == 1) { // Square
            target_vel.linear.x = 0.1;
        }
        else if (msg->buttons[2] == 1) { // Triangle
            target_vel.linear.x = 0.2;
        }
        else if (msg->buttons[1] == 1) { // Circle
            target_vel.linear.x = 0.4;
        }
        else if (msg->buttons[0] == 1) { // X
            target_vel.linear.x = 0.0;
        }

        // Publish target velocity
        target_vel_pub->publish(target_vel);
    }
    
    /*
        * get teleop velocity
    */
    void teleop_callback(const geometry_msgs::msg::Twist &msg) {
        teleop_x = msg->linear.x;
        teleop_z = msg->angular.z;
    }

    /*
        * Subscribe to object detected topic
    */
    void obj_detected_callback(const std_msgs::msg::Bool &msg) {
        wander_active = msg->data;
    }

    /*
        * get wander velocity
    */
    void wander_callback(const geometry_msgs::msg::Twist &msg) {
        wander_x = msg->linear.x;
        wander_z = msg->angular.z;
    }
    
    void cc_callback(const geometry_msgs::msg::Twist &msg) {
        cc_x = msg->linear.x;
        cc_z = msg->angular.z;
    }
    
    void state_machine_loop(void) {
        // Set velocity based on state
        if (teleop_active) {
            // Set velocity based on teleop
            cmd_vel_msg.linear.x = teleop_x;
            cmd_vel_msg.angular.z = teleop_z;
        }
        else if (wander_active) {
            // Set velocity based on wander
            cmd_vel_msg.linear.x = wander_x;
            cmd_vel_msg.angular.z = wander_z;
        }
        else {
            // Set velocity based on cruise control
            cmd_vel_msg.linear.x = cc_x;
            cmd_vel_msg.angular.z = cc_z;
        }

        // Publish velocity command
        cmd_vel_pub->publish(cmd_vel_msg);
    }
    
    // ---------------------------------//
    // Variable used by the Node Object //
    // ---------------------------------//

    // ROS publisher object
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr target_vel_pub;

    // ROS subscriber object
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_vel_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obj_detected_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr wander_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cc_vel_sub;

    // Control timing
    rclcpp::TimerBase::SharedPtr timer;

    // Velocity command message
    geometry_msgs::msg::Twist cmd_vel_msg;
    geometry_msgs::msg::Twist target_vel;
    
    // State variables
    bool teleop_active;
    bool wander_active;
    
    // Teleop Velocity variables
    float teleop_x;
    float teleop_z;

    // Wander Velocity variables
    float wander_x;
    float wander_z;

    // Cruise Control Velocity variables
    float cc_x;
    float cc_z;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WanderNode>());
  rclcpp::shutdown();

  return 0;
}
