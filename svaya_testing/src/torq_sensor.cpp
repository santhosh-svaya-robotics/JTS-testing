#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>


double grav_torq;
double torque_sensor_data;

void grav_torq_callback(const std_msgs::msg::Float64MultiArray & msg)
{
    grav_torq = msg.data[0];
}

void torq_sensor_callback(const std_msgs::msg::Float64 & msg)
{
    torque_sensor_data = msg.data;
}


int main(int argc, char** argv)
{

    rclcpp::init(argc,argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("joint_torq_error_check");

    auto grav_ = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/grav_torq",100, grav_torq_callback);
    auto sensor = node->create_subscription<std_msgs::msg::Float64>("/svaya/torq_sensor",100, torq_sensor_callback);

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/main_node");
    while (!parameters_client->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
        return 0;
      }
      RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }


    bool stopped = false;
    bool trigger = false;

    auto start_time = rclcpp::Clock().now();
    int k = 0;

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        k++;

        if(k > 10000)
        {
            trigger = true;
            k = 0;
        }
//        RCLCPP_INFO_STREAM(node->get_logger(),"k "<<k);

        if (trigger && !stopped)
        {
            RCLCPP_INFO_STREAM(node->get_logger(),"triggered");
            std::vector<rclcpp::Parameter> param_vec;
            param_vec.push_back(rclcpp::Parameter("stop",1));
            parameters_client->set_parameters(param_vec);
            stopped = true;
            start_time = rclcpp::Clock().now();
            trigger = false;
        }

        if(stopped)
        {
            auto current_time = rclcpp::Clock().now();
            auto duration = current_time - start_time;
            double duration_sec = (double)duration.seconds() + (double)duration.nanoseconds()*1e-9;
            RCLCPP_INFO_STREAM(node->get_logger(),"duration : " <<duration_sec);
            if(duration_sec > 4)
            {
                std::vector<rclcpp::Parameter> param_vec;
                param_vec.push_back(rclcpp::Parameter("stop",0));
                parameters_client->set_parameters(param_vec);
                stopped = false;
                RCLCPP_INFO_STREAM(node->get_logger(),"starting ");
            }
        }

        usleep(1000);
    }


    return 0;

}
