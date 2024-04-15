#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <strstream>
#include <svaya_trajectory_publisher/trajectory_publisher.hpp>
int main(int argc,char **argv)
{
  int numJoints = 1;
  rclcpp::init(argc,argv);

  sem_t lock;
  sem_init(&lock,0,1);

  int exe_flag = 1;

  bool start_planning = false; // make it false
  bool stop_node = false;

  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("main_node");

  node_->declare_parameter("stop", 0);
  node_->declare_parameter("start", 0);
  node_->declare_parameter("stop_node", 0);


  auto stop_callback = [&exe_flag, &lock](const rclcpp::Parameter &P)
  {
    if (P.as_int() == 1)
    {
      RCLCPP_INFO(rclcpp::get_logger("main function"), " stoping program" );
      exe_flag = -1;
      sem_wait(&lock);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("main function"), " starting program" );
      exe_flag = 1;
      sem_post(&lock);
    }
  };

  auto start_plan_callback = [&start_planning](const rclcpp::Parameter &P)
  {
    RCLCPP_INFO(rclcpp::get_logger("main function"), " Starting Planning ...." );
    if (!start_planning)
      start_planning = true;
  };

  auto stop_node_callback = [&stop_node, &exe_flag](const rclcpp::Parameter &P)
  {
    RCLCPP_INFO(rclcpp::get_logger("main function"), " Stop Node signal recieved ...." );
    exe_flag = -2;
    stop_node = true;
  };

  auto param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
  auto cb_handle1_ = param_subscriber_->add_parameter_callback("stop", stop_callback);
  auto cb_handle3_ = param_subscriber_->add_parameter_callback("start", start_plan_callback);
  auto cb_handle4_ = param_subscriber_->add_parameter_callback("stop_node", stop_node_callback);

  auto f = [node_]()
  {
    rclcpp::spin(node_);

  };
  std::thread thread(f);

 trajectory_publisher::TrajectoryPublisher publisher(&lock, &exe_flag);

 double joint_vel = M_PI/6;

 publisher.startController();

 RCLCPP_INFO(node_->get_logger(),"Controller started");

// while(rclcpp::ok())
// {
//     if(start_planning)
//     {
//         publisher.PublishTrajectory(joint_vel, -90.0, 90.0);
//     }
//     if(stop_node)
//     {
//         break;
//     }
// }
 rclcpp::shutdown();

 thread.join();

}

