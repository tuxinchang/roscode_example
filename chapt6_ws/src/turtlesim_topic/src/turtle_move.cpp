#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>

class TurtleMove : public rclcpp::Node
{
public:
    TurtleMove() : Node("turtle_move")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() {
                auto msg = geometry_msgs::msg::Twist();
                msg.linear.x = 2.0;
                msg.angular.z = 0.5;
                publisher_->publish(msg);
            });
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleMove>());
    rclcpp::shutdown();
    return 0;
}