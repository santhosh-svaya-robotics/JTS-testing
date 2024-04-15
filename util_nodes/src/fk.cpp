#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <kdl/chainfksolvervel_recursive.hpp>

//#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;

KDL::JntArray jnt_pos;
KDL::JntArray jnt_vel;
KDL::JntArray jnt_acc;

KDL::JntArray jnt_pos_cmd;

void callback_func(const std_msgs::msg::Float64MultiArray & msg)
{
    jnt_pos(0) = msg.data[0] - 0.0*3.1457/180;
    jnt_vel(0) = msg.data[1];
    jnt_acc(0) = -msg.data[2];
    if(fabs(jnt_acc(0))>3.14)
      jnt_acc(0) = 0;
}

int main(int argc, char** argv)
{


  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("util_node");

  std::vector<std::string> joint_names;// = {"joint1","joint2","joint3","joint4","joint5","joint6"};
  std::string robot_desc_string;
  KDL::Tree my_tree;
  KDL::Chain kdl_chain;
  KDL::Frame cart_pos_kdl;
  KDL::Frame cart_pos_kdl_cmd;


  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/robot_state_publisher");
  while (!parameters_client->wait_for_service()) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto parameters = parameters_client->get_parameters({"robot_description"});

  std::stringstream ss;
  // Get a few of the parameters just set.
  for (auto & parameter : parameters)
  {
    if (parameter.get_name() == "robot_description")
    {
        robot_desc_string = parameter.value_to_string();
    }
  }


  jnt_pos.resize(1);
  jnt_vel.resize(1);
  jnt_acc.resize(1);
  jnt_pos_cmd.resize(1);

  if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
  {
      RCLCPP_ERROR(node->get_logger(), "Util_node : Failed to construct kdl tree");
  }

  if (!my_tree.getChain("link1","link2",kdl_chain))
  {
      RCLCPP_ERROR(node->get_logger(), "Util_node : Failed to construct kdl chain");
  }


  KDL::Vector grav_vec = KDL::Vector(0,0,9.81);
  KDL::ChainIdSolver_RNE id_solver(kdl_chain, grav_vec);
  KDL::Wrenches ext_forces(kdl_chain.getNrOfJoints(), KDL::Wrench::Zero());
  KDL::JntArray jnt_torq = KDL::JntArray(1);


//  RCLCPP_INFO_STREAM(node->get_logger(),"COM : "<<kdl_chain.segments[1].)
//  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cart_pos_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/cart_pos",10);w
  auto grav_torq_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/grav_torq",100);
  auto subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/joint_state",10,callback_func);

//  bool test_parameter;
  while (rclcpp::ok())
  {

    std_msgs::msg::Float64MultiArray GravTorq;
    GravTorq.data.resize(1);


    id_solver.CartToJnt(jnt_pos, jnt_vel, jnt_acc, ext_forces, jnt_torq);
    for (unsigned int i = 0 ; i < 1; i++)
    {
        GravTorq.data[i] = jnt_torq(i);
    }
//    GravTorq.data[0] = jnt_torq(1);

//    RCLCPP_INFO_STREAM(node->get_logger(),"pos :"<<jnt_pos(0)<<", vel : "<<jnt_vel(0)<<", acc : "<<jnt_acc(0));
//    RCLCPP_INFO_STREAM(node->get_logger(),"grav : torq : "<<jnt_torq(0)<<", pos: " <<jnt_pos(0));
    grav_torq_pub->publish(GravTorq);


    rclcpp::spin_some(node);
    rclcpp::sleep_for(1ms);
  }

  return 0;
}
