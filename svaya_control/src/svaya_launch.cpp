#include <thread>
#include <memory>

// ROS includes
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <signal.h>


volatile sig_atomic_t stop;

void mySigintHanfdler(int sig)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("signal handle"), "Caught signal : " << sig);
  stop = 1;
  rclcpp::shutdown();
  system("pkill -9 ros2");
  system("pkill -9 svaya_control_n");
  system("pkill -9 move_group");
}

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("svaya_parameters", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));


    std::shared_ptr<rclcpp::Executor> e = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();


    std::thread th([node](){
      rclcpp::spin(node);
    });

//    auto controller_manager = std::make_shared<controller_manager::ControllerManager>(e, "controller_manager");

//    struct sigaction sigIntHandler;

//    sigIntHandler.sa_handler = mySigintHanfdler;
//    sigemptyset(&sigIntHandler.sa_mask);
//    sigIntHandler.sa_flags = 0;

//    sigaction(SIGINT, &sigIntHandler, NULL);

//    std::thread control_loop([controller_manager] (){

//        const rclcpp::Duration dt = rclcpp::Duration::from_seconds(1.0/controller_manager->get_update_rate());

//        auto const period = std::chrono::nanoseconds(1'000'000'000 / controller_manager->get_update_rate());
//                                                         auto const cm_now = std::chrono::nanoseconds(controller_manager->now().nanoseconds());

//                  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
//                  next_iteration_time{cm_now};

//        rclcpp::Time previous_time = controller_manager->now();

//        while (!stop)
//        {
//          auto const current_time = controller_manager->now();
//          auto const measured_period = current_time - previous_time;
//          previous_time = current_time;

//            controller_manager->read(controller_manager->now(), dt);
//            controller_manager->update(controller_manager->now(), dt);
//            controller_manager->write(controller_manager->now(), dt);

//            next_iteration_time += period;
//            std::this_thread::sleep_until(next_iteration_time);
//        }

//    });

//    e->add_node(controller_manager);
//    e->spin();

//    control_loop.join();

 th.join();


//    e->remove_node(controller_manager);

    rclcpp::shutdown();

    return 0;
}
