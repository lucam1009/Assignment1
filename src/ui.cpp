#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <string>
#include <limits>

using std::placeholders::_1;


class ui: public rclcpp::Node
{
    public:
    ui(): Node("ui")
    {
        pub_turtle1 = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pub_turtle2 = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        sub_turtle1 = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&ui::callback_turtle1, this, _1));
        sub_turtle2 = this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, std::bind(&ui::callback_turtle2, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ui::execute, this));
    }
    int counter = 20;
    double x_1 = 2.0;
    double x_2 = 8.0;
    double y_1 = 2.0;
    double y_2 = 8.0;
    int turtle_id;
    double v, alpha;


    private:

    void execute(){

        if(distance() == true){ 
            
            if (counter >= 20){
               
                
                stop();

                turtle_id = 0;
                while(turtle_id != 1 && turtle_id != 2){
                    std::cout << "Inserire ID tartaruga da muovere (1 o 2): ";
                    std::cin >> turtle_id;
                }

                while (true) {
                    std::cout << "Inserisci la velocità v da assegnare alla tartaruga: ";
                    std::cin >> v;
                    if (std::cin.fail()) {
                        std::cout << "Errore: Input non valido! Per favore inserisci un numero." << '\n';
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    } 
                    else {
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        break; 
                    }
                }

                while (true) {
                    std::cout << "Inserisci la velocità angolare alpha da assegnare alla tartaruga: ";
                    std::cin >> alpha;
                    if (std::cin.fail()) {
                        std::cout << "Errore: Input non valido! Per favore inserisci un numero." << '\n';
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    } 
                    else {
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        break; 
                    }
                }

                if (turtle_id == 1){
                    message.linear.x = v;
                    message.angular.z = alpha;
                    pub_turtle1->publish(message);
                    counter = 0;
                }
                else{
                    message.linear.x = v;
                    message.angular.z = alpha;
                    pub_turtle2->publish(message);
                    counter = 0;
                }

            }else{
                counter += 1;
            }
        }else{
            stop();
            std::cout << "Tartarughe troppo vicine o vicine al bordo \n";
            if(turtle_id == 1){
                if(v == 0 && alpha == 0){
                    message.linear.x = 1;
                }else{
                    message.linear.x = -v;
                    message.angular.z = -alpha;
                }
                pub_turtle1->publish(message);
            }else{
                if(v == 0 && alpha == 0){
                    message.linear.x = 1;
                }else{
                    message.linear.x = -v;
                    message.angular.z = -alpha;
                }
                pub_turtle2->publish(message);
            }
        }
    }

    void stop(){
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        pub_turtle1->publish(message);
        pub_turtle2->publish(message);
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
    
    bool distance()
    {
        double x = std::pow(x_1 - x_2, 2);
        double y = std::pow(y_1 - y_2, 2);
        double distance = std::sqrt(x+y);
        if(distance < 1 || x_1 < 1 ||  x_1 > 10 || x_2 < 1 ||  x_2 > 10 || y_1 < 1 ||  y_1 > 10 || y_2 < 1 ||  y_2 > 10){
            
            return false;
        }

        return true;
    }
    

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle1;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist message;

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ui>());
    rclcpp::shutdown();
    return 0;
}