#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <string>
#include <limits>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>

using std::placeholders::_1;


class distance_check: public rclcpp::Node
{
    public:
    distance_check(): Node("distance_check")
    {
        pub_safety = this->create_publisher<std_msgs::msg::Int32>("/safety_status", 10);
        sub_turtle1 = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&distance_check::callback_turtle1, this, _1));
        sub_turtle2 = this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, std::bind(&distance_check::callback_turtle2, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&distance_check::security_check, this));
    }

    double x_1 = 2.0;
    double x_2 = 8.0;
    double y_1 = 2.0;
    double y_2 = 8.0;
    int turtle_id;
    double v, alpha;

    private:

    void security_check(){

        std_msgs::msg::Int32 status;
        double x = std::pow(x_1 - x_2, 2);
        double y = std::pow(y_1 - y_2, 2);
        double distance = std::sqrt(x+y);
        
        if(distance < 1) {
           status.data = 1;
        }else if(x_1 < 1 ||  x_1 > 10 || x_2 < 1 ||  x_2 > 10 || y_1 < 1 ||  y_1 > 10 || y_2 < 1 ||  y_2 > 10){
            status.data = 2;
        }else{
            status.data = 0;
        }

        pub_safety->publish(status);
    }

    void callback_turtle1(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x_1 = msg->x;
        y_1 = msg->y;
    }

    void callback_turtle2(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x_2 = msg->x;
        y_2 = msg->y;
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle1;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle2;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_safety;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist message;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<distance_check>());
    rclcpp::shutdown();
    return 0;
}