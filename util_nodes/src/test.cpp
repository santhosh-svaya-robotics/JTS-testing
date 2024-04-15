#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_node");
  auto pub = node->create_publisher<std_msgs::msg::String>("/test_topic", 10);


  std_msgs::msg::String msg;
  msg.data = "hello";

  while(rclcpp::ok())
  {

    pub->publish(msg);

    usleep(10000);
  }

  rclcpp::shutdown();

  return 0;

}
