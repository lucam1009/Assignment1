#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
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
        sub_safety = this->create_subscription<std_msgs::msg::Int32>("/safety_status", 10, std::bind(&ui::callback_safety, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ui::execute, this));
    }
    int counter = 20;
    int safety_status = 0;
    int turtle_id;
    double v, alpha;


    private:

    void callback_safety(const std_msgs::msg::Int32::SharedPtr msg)
    {
        safety_status = msg->data;
    }

    void execute(){

        if(safety_status == 0){ 
            
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
        }else if(safety_status == 1){

            stop();
            std::cout << "Tartarughe troppo vicine \n";
            if(v == 0 && alpha == 0){
                message.linear.x = 1;
            }else{
                if(v < 0){
                    message.linear.x = 1.5;
                    message.angular.z = 0.0;
                }else{
                    message.linear.x = -1.5;
                    message.angular.z = 0.0;
                }
            }
            if(turtle_id == 1){
                pub_turtle1->publish(message);
            }else{
                pub_turtle2->publish(message);
            }
        }else{
            stop();
            std::cout << "Tartarughe vicine al bordo \n";
            if(v == 0 && alpha == 0){
                message.linear.x = 1;
            }else{
                if(v < 0){
                    message.linear.x = 1.5;
                    message.angular.z = 0.0;
                }else{
                    message.linear.x = -1.5;
                    message.angular.z = 0.0;
                }
            }
            if(turtle_id == 1){
                pub_turtle1->publish(message);
            }else{
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

    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_safety;
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