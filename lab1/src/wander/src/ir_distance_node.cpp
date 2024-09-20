#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"

class IRDistanceNode : public rclcpp::Node
{
public:
    IRDistanceNode() : Node("ir_distance_node")
    {
        // IR Emitter index parameter (0 = left, 6 = right)
        this->declare_parameter<int>("ir_index", 3);
        
        // Get the value of the sensor index parameter
        this->get_parameter("ir_index", ir_index);

        RCLCPP_INFO(this->get_logger(), "Using IR index: %d", ir_index);

        // Subscribe to IR Sensor data topic from Turtlebot
        ir_sub  = this->create_subscription<irobot_create_msgs::msg::IrIntensityVector>(
            "TTB10/ir_intensity", 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
            [this](const irobot_create_msgs::msg::IrIntensityVector &msg) { this->publishRangeData(msg); }
        );

        // Publisher for the IR range data
        range_publisher = this->create_publisher<sensor_msgs::msg::Range>(
            "ir_range", 
            rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort)
        );

    }

private:
    int ir_index;  // Member variable for ir_index

    void publishRangeData(const irobot_create_msgs::msg::IrIntensityVector &msg)
    {
        sensor_msgs::msg::Range range_msg;
        range_msg.header.stamp = this->now();
        
        // Check if ir_index is within bounds
        if (ir_index < msg.readings.size()) {
            range_msg.header.frame_id = msg.readings[ir_index].header.frame_id;
            range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
            range_msg.field_of_view = 0.1; // Field of view in radians
            range_msg.min_range = 0.01;    // Minimum range in meters
            range_msg.max_range = 0.78;     // Maximum range in meters

            int intensity = msg.readings[ir_index].value; // Intensity Value

            range_msg.range = calculateRange(intensity); // Range Value

            range_publisher->publish(range_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid IR index: %d", ir_index);
        }
    }

    float calculateRange(int intensity)
    {
        // Calculate the range value from the intensity value
        float numerator = 54.0;
        float offset = 70.0;
        float denominator = intensity + offset;
        
        if (denominator <= 0.0) {
            return 0.78;
        }

        return numerator / denominator;
    }

    rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr ir_sub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRDistanceNode>());
    rclcpp::shutdown();
    return 0;
}
