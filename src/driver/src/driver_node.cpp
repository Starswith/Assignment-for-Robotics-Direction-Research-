#include "rclcpp/rclcpp.hpp"
#include "driver_msgs/msg/target.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class DriverNode : public rclcpp::Node {
public:
    DriverNode() : Node("driver_node"), count_(0) {
        // Declare and get parameter
        this->declare_parameter("initial_count", 0);
        int initial_count = this->get_parameter("initial_count").as_int();
        if (initial_count < 0 || initial_count > 100) {
            RCLCPP_WARN(this->get_logger(), "Initial count out of range, using default 0.");
        } else {
            count_ = initial_count;
        }

        // Publisher for the custom Target message
        publisher_ = this->create_publisher<driver_msgs::msg::Target>("target", 10);
        
        // Timer callback at 500Hz (2ms)
        timer_ = this->create_wall_timer(2ms, std::bind(&DriverNode::timer_callback, this));
        
        start_time_ = this->now();
    }

private:
    void timer_callback() {
        count_++;
        
        // Create and populate the custom Target message
        auto message = driver_msgs::msg::Target();
        message.name = "targ";
        message.count = count_;
        
        double t = (this->now() - start_time_).seconds();
        message.time = t;
        message.target.x = std::sin(t);
        message.target.y = std::cos(t);
        message.target.z = std::sin(t * 2);

        // Publish the message
        publisher_->publish(message);

        // Log the message info
        RCLCPP_INFO(this->get_logger(), "Message %d published at %.2f seconds", count_, t);
    }

    rclcpp::Publisher<driver_msgs::msg::Target>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    int count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriverNode>());
    rclcpp::shutdown();
    return 0;
}