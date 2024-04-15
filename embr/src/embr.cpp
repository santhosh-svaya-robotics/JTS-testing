#include "embr/embr.h"
#include <std_msgs/msg/string.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
// #include <rclcpp/macros.hpp>
// #include <rclcpp/rclcpp.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace embr{
hardware_interface::CallbackReturn embrHW::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  node = rclcpp::Node::make_shared("embrHW");
  initialized = false;

  parameter_reading_done = false;

  virtual_flag = false;


  fts_sensor_data.data.resize(6);

  if (hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  gear_rato = {101};
  enc_count = {1048576};
  rated_torq = {1.58};
  pos_sign = {1};
  vel_sign = {1};
  joint_trajectory_controller_status = false;
  engaged = false;


  state_msg.data.resize(3*NUM_JOINTS);
  state_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/joint_state",500);



//  std::cout<<"164";

  //auto subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/joint_state",100,callback_func);

  auto model_torq_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/model_torq",100);
//  std::cout<<"168";


  cmd_pos.resize(NUM_JOINTS , 0);
  cmd_vel.resize(NUM_JOINTS , 0);
  cmd_acc.resize(NUM_JOINTS , 0);
  cmd_eff.resize(NUM_JOINTS , 0);

  pos.resize(NUM_JOINTS , 0);
  vel.resize(NUM_JOINTS , 0);
  eff.resize(NUM_JOINTS , 0);
  acc.resize(NUM_JOINTS, 0);

  dummy_pos.resize(NUM_JOINTS , 0);
  dummy_vel.resize(NUM_JOINTS , 0);
  dummy_eff.resize(NUM_JOINTS , 0);
  //analog_vec.resize(10,0);

  torq_est.data = 0;
  enc_pos_data.data.resize(2);

  auto n = node->get_node_names();

  parameter_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/svaya_parameters");
  while (!parameter_client->wait_for_service())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  read_parameters();


  if(parameter_reading_done == false)
  {
    RCLCPP_ERROR(node->get_logger(),"Unable to Read the configuration");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  param_subscriber_= std::make_shared<rclcpp::ParameterEventHandler>(node);
  std::cout<<"cp 219                      "<<std::endl;
  controller_handle = param_subscriber_->add_parameter_callback("controller_status", std::bind(&embrHW::controller_callback, this, std::placeholders::_1), "/svaya_parameters");


  master.position = 0;
  std::cout<<"cp 224                      "<<std::endl;

  master.Ethercat_bus_reading();
  std::cout<<"cp 225                      "<<std::endl;

  Slave_obj_assignment(master,drive);
  master.activate();

//  svaya_driver = new svaya_driver::SvayaDriver(gear_rato,enc_count,rated_torq);
//  svaya_driver->addPrintCallback(std::bind(&SvayaHW::call_back_fun,this,std::placeholders::_1));

  std::cout<<"Drive size: "<<drive.size()<<"\n\n\n\n\n";
  Initialisation_Flag = false;
  analog_in = 0;

  if(!virtual_flag)
  {
//  svaya_driver->init();
  }

  return hardware_interface::CallbackReturn::SUCCESS;

 }


std::vector<hardware_interface::StateInterface> embrHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t jnt_cnt = 0; jnt_cnt < info_.joints.size(); jnt_cnt++)
  {

    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_POSITION, &pos[jnt_cnt]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_VELOCITY, &vel[jnt_cnt]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_ACCELERATION, &acc[jnt_cnt]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_EFFORT, &eff[jnt_cnt]));

  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> embrHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t jnt_cnt = 0; jnt_cnt < info_.joints.size(); jnt_cnt++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_POSITION, &cmd_pos[jnt_cnt]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel[jnt_cnt]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_ACCELERATION, &cmd_acc[jnt_cnt]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_EFFORT, &cmd_eff[jnt_cnt]));
  }

  return command_interfaces;
}
hardware_interface::return_type embrHW::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
// drive[0]->enable_motor = true;
 rclcpp::spin_some(node);
  rclcpp::sleep_for(1ms);
  return hardware_interface::return_type::OK;

}
hardware_interface::return_type embrHW::write(const rclcpp::Time & time, const rclcpp::Duration & period){
//  std::cout<<"write"<<std::endl;
  drive[0]->brake_status = 2;

  bool temp_check = false;
  bool temp_ready = false;
master.update(&temp_check, &temp_ready);
  return hardware_interface::return_type::OK;

}

hardware_interface::return_type embrHW::Slave_obj_assignment(knexo_ethercat::Master &master, std::vector<knexo_ethercat::Synapticon_DC1K_D3 *> &drive)
{
  std::cout<<"cranking 90s"<<std::endl;
    map<int, std::vector<knexo_ethercat::Slave *>>::iterator itr;
    std::string slavename;
    int drive_index = 0;
    printf("master.slave_obj_map size %d\n", master.slave_obj_map.size());
    printf(" Drive initialized Size  %d \n", drive.size());
    for (itr = master.slave_obj_map.begin(); itr != master.slave_obj_map.end(); ++itr)
    {
      switch (itr->first)
      {
      case 1:
        printf("object assigned SOMANET CiA402 Drive %d \n", drive_index);
        for (auto obj = itr->second.begin(); obj != itr->second.end(); obj++)
        {
          printf("drive index %d\n", drive_index);
          printf("addres  %d\n", (*obj));
          // drive.push_back(((knexo_ethercat::Synapticon_DC1K_D3*)(*obj)));
          drive.push_back(((dynamic_cast<knexo_ethercat::Synapticon_DC1K_D3 *>(*obj))));
          printf("  drive index %d address %d\n", drive_index, drive[drive_index]);
          drive_index++;
        }

        break;
      case 2:
        printf("SAMD51 EtherCAT Slave\n");
//        for (auto obj = itr->second.begin(); obj != itr->second.end(); obj++)
//        {
//          if (master.position == 0)
//          {
//            left_sigConditionBoard = (((knexo_ethercat::SAMD51Lan9252 *)*(obj)));
//            RCLCPP_INFO_STREAM(node->get_logger(),"  left SAMD51 EtherCAT Slave address "<< left_sigConditionBoard);
//          }
//          else if (master.position == 1)
//          {
//            right_sigConditionBoard = (((knexo_ethercat::SAMD51Lan9252 *)*(obj)));
//            RCLCPP_INFO_STREAM(node->get_logger(),"  right SAMD51 EtherCAT Slave address "<< right_sigConditionBoard);
//          }
//        }

        break;

      case 3:
        printf("FT Sensor Slave\n");

        for (auto obj = itr->second.begin(); obj != itr->second.end(); obj++)
        {

          continue;
        }
        break;
      }
    }
}
void embrHW::controller_status()
{

  std::vector<std::string> param_list = {"controller_status"};
  rclcpp::spin_some(node);

  auto param = parameter_client->get_parameters(param_list);

  joint_trajectory_controller_status =  param[0].as_bool();

  RCLCPP_INFO_STREAM(node->get_logger(),"controller status : "<<joint_trajectory_controller_status);

}
void embrHW::read_parameters()
{

  RCLCPP_INFO_STREAM(node->get_logger(),"\n\nRead Params\n\n");

  while (!parameter_client->wait_for_service())
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
  std::vector<std::string> param_list = {"gear_ratio","encoder_cnt","joint_rated_torque","stiffness", "strain_gauge_factor","link_length","mass","f"};
  rclcpp::spin_some(node);

  std::cout<<"Reading Parameters\n";

  auto param = parameter_client->get_parameters(param_list);
  for (size_t i=0; i < param.size(); i++)
  {
    if(param[i].get_name() == "gear_ratio")
    {
      gear_rato = param[i].as_integer_array();
//      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter gear ratio: "<<gear_rato[0]<<" "<<gear_rato[1]<<" "<<gear_rato[2]<<" "<<gear_rato[3]<<" "<<gear_rato[4]<<" "<<gear_rato[5]);

    }
    if(param[i].get_name() == "encoder_cnt")
    {
      enc_count = param[i].as_integer_array();
//      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter encoder count: "<<enc_count[0]<<" "<<enc_count[1]<<" "<<enc_count[2]<<" "<<enc_count[3]<<" "<<enc_count[4]<<" "<<enc_count[5]);
    }
    if(param[i].get_name() == "joint_rated_torque")
    {
      rated_torq = param[i].as_double_array();
//      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter Joint Rated Torque: "<<rated_torq[0]<<" "<<rated_torq[1]<<" "<<rated_torq[2]<<" "<<rated_torq[3]<<" "<<rated_torq[4]<<" "<<rated_torq[5]);
    }
//    if(param[i].get_name() == "stiffness")
//    {
//      stiffness = param[i].as_double();
////      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter Joint Rated Torque: "<<rated_torq[0]<<" "<<rated_torq[1]<<" "<<rated_torq[2]<<" "<<rated_torq[3]<<" "<<rated_torq[4]<<" "<<rated_torq[5]);
//    }

  }
  parameter_reading_done = true;
  RCLCPP_INFO_STREAM(node->get_logger(),"Read Params done: "<<parameter_reading_done);

}
void embrHW::controller_callback(const rclcpp::Parameter &P)
{
  joint_trajectory_controller_status = P.as_bool();
}
}

PLUGINLIB_EXPORT_CLASS(embr::embrHW,hardware_interface::SystemInterface)


