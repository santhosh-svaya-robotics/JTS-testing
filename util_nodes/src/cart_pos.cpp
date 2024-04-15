#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class CartPosition : public rclcpp::Node
{
    public:
        CartPosition() : Node("cart_position")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/cart_pos", 100, std::bind(&CartPosition::topic_callback, this, _1));
        }

    private:
        void topic_callback(const std_msgs::msg::Float64MultiArray & msg) const
        {
//            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
          std::cout<<msg.data[0]<<", "<<msg.data[1]<<", "<<msg.data[2]<<"\n";
        }
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main (int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartPosition>());
    rclcpp::shutdown();
    return 0;
}
