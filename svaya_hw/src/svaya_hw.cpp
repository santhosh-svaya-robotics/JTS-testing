#include "svaya_hw/svaya_hw.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include<sensor_msgs/msg/joint_state.hpp>
//#include <urdf/model.h>
#include <fstream>
#include <cmath>

// #include <hardware_interface/handle.hpp>
// #include <hardware_interface/hardware_info.hpp>
// #include <hardware_interface/system_interface.hpp>
// #include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
// #include <rclcpp/macros.hpp>
// #include <rclcpp/rclcpp.hpp>
#include "pluginlib/class_list_macros.hpp"


double th;
double th_dot;
double th_ddot;
double th_dot_fil;
double th_dot_prev = 0;
double th_fil = 0;
double th_prev = 0;


double th2;
double th2_dot;
double th2_ddot;

double strain_gauge_factor;
double damping_coeff;
double link_length;
double mass;
double inertia;
double g = 9.81;
double f;
double th1_prev = 0.0;
double T_set = 0.01;
double th1 = 0.0;
double th1_dot;
double th1_dot_prev = 0.0;
double torq = 0.0;
double torq_prev = 0.0;
double torq_fil = 0.0;
double sensor_torq;
rclcpp::Time curr_time;
rclcpp::Time prev_time;
double C = 0.6272;
double delta_t = 0.0;

bool initialised = false;

double analog_fil = 00.0;
double analog_prev = 0.00;
double intercept = 32106.00;
double slope = 58.760;
double stiffness = 2000.00;
double COG_offset = 1.85;
double k1 = 200;
double k2 = 1800;
double set_time1 = 0.5;
double set_time2 = 0.08;
double loading_rate = 0.0;
double stiffness_corr = 0;
double stiffness_corr2 = 0;
double Fk = 0;
double Fk_dot = 0;
double Fk_ddot = 0;
double str_dot_prev = 0;
double torq_nf = 0.0;
double torq_nf1 = 0.0;
double torq_nf0 = 0.0;

double torq_fil_lp = 0.0;
double torq_fil_lp1 = 0.0;
double torq_fil_lp2 = 0.0;

double jt_dot = 0.0;
double jt_ddot = 0.0;

double st_nf = 0.0;
double st_nf1 = 0.0;
double st_nf0 = 0.0;

double st= 0.0;
double st1= 0.0;
double st2= 0.0;

double st_dot = 0.0;
double st_ddot = 0.0;

double w = 160;
double a = 1.2;
double z = 0.0001;
bool gchold = false;
double gc_sens = 7.0;
vector <double> torq_buffer;
double support_torq;
double support_torq_prev;
double support_torq_smooth;
bool gcstart = true;
bool startup = true;
double tension = 1.0;
double compression = 1.0;
double counter = 0.0;
int control_mode = 9;
namespace svaya_hw
{
void change_op_mode(int exe_flag, bool reached){
  if(reached || exe_flag == -1 || exe_flag == -2){
    std::cout<<"changing to control mode 9";
  }
}

void calibrate_func(const std_msgs::msg::Float64MultiArray & msg){
//    std::cout<<msg.data[0];
    slope = msg.data[0];
    intercept = msg.data[1];
    stiffness = msg.data[2];
    COG_offset = msg.data[3];
}

void set_gc_flag(const std_msgs::msg::Bool &flag){
  gchold = flag.data;
}
void set_gc_sens(const std_msgs::msg::Float64 &flag){
  gc_sens = flag.data;
}
void tc_callback(const trajectory_msgs::msg::JointTrajectory &trajmsg){
//  std::cout<<"recieved trajectory msg; setting to position control \n\n"<<std::endl;
//  control_mode = 8;
}
hardware_interface::CallbackReturn SvayaHW::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  node = rclcpp::Node::make_shared("svaya_hw");
  initialized = false;

  parameter_reading_done = false;

  virtual_flag = false;
  gchold = false;

  sensor_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/sensor_data",10);

  fts_sensor_data.data.resize(6);

  if (hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  gear_rato = {101};
  enc_count = {1048576, 1048576,1048576, 524288, 524288, 524288};
  rated_torq = {3.45, 3.45, 1.58};
  pos_sign = {1,1,1,1,1,1};
  vel_sign = {1,1,1,1,1,1};
  joint_trajectory_controller_status = false;
  engaged = false;


  state_msg.data.resize(3*NUM_JOINTS);
  torq_sensor_data.data = 0;
  state_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/joint_state",500);
  torq_sens = node->create_publisher<std_msgs::msg::Float64>("/svaya/torq_sensor",500);
  analog_signal = node->create_publisher<std_msgs::msg::Float64>("/svaya/analog_signal",500);
 analog_scaled_signal = node->create_publisher<std_msgs::msg::Float64>("/svaya/analog_scaled_signal",500);
  motor_torq_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/motor_torq",500);

  model_torq_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/model_torq",500);
  dc_link_vol_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/dc_lv",500);
  target_torq_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/target_torq",500);

  calibration_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/calibrate",50, calibrate_func);
  error_pub = node->create_publisher<std_msgs::msg::Float64>("/svaya/torq_error",100);
  gc_flag_sub = node->create_subscription<std_msgs::msg::Bool>("/svaya/gc_flag",100, set_gc_flag);
  gc_sens_sub = node->create_subscription<std_msgs::msg::Float64>("/svaya/gc_sens",100, set_gc_sens);

  trajectory_controller_sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 100, tc_callback);

//  std::cout<<"164";

  //auto subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/joint_state",100,callback_func);

//  auto model_torq_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/model_torq",100);
//  std::cout<<"168";

  enc_pos_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/enc_pos",500);
  torq_est_pub = node->create_publisher<std_msgs::msg::Float64>("/svaya/torq_est",500);

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

  fout.open("output.csv", std::ios::out);
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
  controller_handle = param_subscriber_->add_parameter_callback("controller_status", std::bind(&SvayaHW::controller_callback, this, std::placeholders::_1), "/svaya_parameters");


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

std::vector<hardware_interface::StateInterface> SvayaHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t jnt_cnt = 0; jnt_cnt < info_.joints.size(); jnt_cnt++)
  {
    std::cout<<info_.joints[jnt_cnt].name<<std::endl;
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_POSITION, &pos[jnt_cnt]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_VELOCITY, &vel[jnt_cnt]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_ACCELERATION, &acc[jnt_cnt]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[jnt_cnt].name, hardware_interface::HW_IF_EFFORT, &eff[jnt_cnt]));

  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SvayaHW::export_command_interfaces()
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

hardware_interface::return_type SvayaHW::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,const std::vector<std::string>& stop_interfaces)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SvayaHW::perform_command_mode_switch(const std::vector<std::string>& /*start_interfaces*/,const std::vector<std::string>& /*stop_interfaces*/)
{

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SvayaHW::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

//   std::cout<<"Read beg : "<<drive[0]->scaled_analog_<<"\n";

// RCLCPP_INFO_STREAM(node->get_logger(), drive[0]->initialized());
//  controller_status();
  if(!virtual_flag)
  {
    if(joint_trajectory_controller_status)
    {
      if( drive.size() != 0)//&& drive[0]->initialized() )
      {
        if(gchold)
        {
          drive[0]->mode_of_operation_ = 10;
//          std::cout<<"mode "<<10<<std::endl;

        }
        else{
          for(size_t i=0; i<NUM_JOINTS; i++){
            drive[i]->mode_of_operation_ = 9;
//             drive[i]->mode_of_operation_ = 8;
          }


        }
        for(size_t i=0; i<NUM_JOINTS; i++){
          drive[i]->enable_motor = true;
        }


      }

    }
    else
    {
      if(drive.size() != 0 )//&& drive[0]->initialized())
      {
        for(size_t i=0; i<NUM_JOINTS; i++){
          drive[i]->enable_motor = false;
        }
      }
    }
  }

  std::vector<double>jpos;
  jpos.resize(NUM_JOINTS);
  std::vector<double>jvel;
  jvel.resize(NUM_JOINTS);
  std::vector<int> prim_jpos_inc;
  std::vector<int> sec_jpos_inc;
  std_msgs::msg::Float64MultiArray motor_torq;
  std_msgs::msg::Float64 error;
  std_msgs::msg::Float64MultiArray dc_lv;
  std_msgs::msg::Float64MultiArray target_torque;

  std_msgs::msg::Float64 analog_data;


  prim_jpos_inc.resize(1);
  sec_jpos_inc.resize(1);

  motor_torq.data.resize(NUM_JOINTS);
  dc_lv.data.resize(NUM_JOINTS);
  target_torque.data.resize(NUM_JOINTS);
  if(virtual_flag)
  {
    for (size_t jnt_cnt = 0; jnt_cnt < NUM_JOINTS; jnt_cnt++)
    {
      pos[jnt_cnt] = dummy_pos[jnt_cnt];
      vel[jnt_cnt] = dummy_vel[jnt_cnt];
      eff[jnt_cnt] = dummy_eff[jnt_cnt];

      state_msg.data[jnt_cnt] = dummy_pos[jnt_cnt];
      state_msg.data[jnt_cnt + NUM_JOINTS] = dummy_vel[jnt_cnt];
      state_msg.data[jnt_cnt + 2*NUM_JOINTS] = cmd_acc[jnt_cnt];
    }

  }
  else if(drive.size()!=0 )
  {
    for(size_t joint=0; joint<NUM_JOINTS; joint++){
      jpos[joint] = static_cast<double>(drive[joint]->position_)/enc_count[joint]*2*M_PI;
      jvel[joint] = static_cast<double>(drive[joint]->velocity_)/100/60*2*M_PI;
      eff[joint] = static_cast<double>(drive[joint]->torque_)/1000*rated_torq[joint]*gear_rato[0];
    }

//      jpos[0] = static_cast<double>(drive[0]->position_)/enc_count[0]*2*M_PI;
//      jvel[0] = static_cast<double>(drive[0]->velocity_)/100/60*2*M_PI;
//      eff[0] = static_cast<double>(drive[0]->torque_)/1000*rated_torq[0]*gear_rato[0];

//      jpos[1] = static_cast<double>(drive[1]->position_)/524288*2*M_PI;
//      jvel[1] = static_cast<double>(drive[1]->velocity_)/100/60*2*M_PI;
//      eff[1] = static_cast<double>(drive[1]->torque_)/1000*rated_torq[0]*gear_rato[0];
//      std::cout<<static_cast<double>(drive[1]->position_)<<std::endl;
//      analog_data.data = static_cast<double>(drive[0]->analog_input_1_);
//      std::cout<<(drive[0]->scaled_analog_)<<std::endl;
      st = static_cast<double>(drive[0] -> scaled_analog_);


//    svaya_driver->getJointPos(jpos);
//    svaya_driver->getJointVel(jvel);
//    svaya_driver->getJointTor(eff);
//    svaya_driver->getAnalogIn1(analog_in);

      prim_jpos_inc[0] = drive[0]->position_;
      sec_jpos_inc[0] = drive[0]->second_position_;
      for(size_t joint = 0; joint<NUM_JOINTS; joint++){
        motor_torq.data[joint] = eff[joint];
        dc_lv.data[joint] = static_cast<double>(drive[joint]->dc_linkVoltage)/1000;
        target_torque.data[0]= static_cast<double>(drive[joint] -> torque_demand)*183/1000;

      }
//      motor_torq.data = eff[0];
//      dc_lv.data = static_cast<double>(drive[0]->dc_linkVoltage)/1000;
//      target_torque.data= static_cast<double>(drive[0] -> torque_demand)*183/1000;
//      std::cout<<(drive[0] -> torque_demand)<<std::endl;

//      RCLCPP_INFO_STREAM(node->get_logger(),"effort : "<<drive[0]->torque_);

//    svaya_driver->getPrimaryJointPosInc(prim_jpos_inc);
//    svaya_driver->getSecondaryJointPosInc(sec_jpos_inc);

    enc_pos_data.data[0] = prim_jpos_inc[0];
    enc_pos_data.data[1] = sec_jpos_inc[0];

//    RCLCPP_INFO_STREAM(node->get_logger(),"p pos : "<<prim_jpos_inc[0]<<", sec pos : "<<sec_jpos_inc[0]);
//    std::cout<<"199\n";

    for (int i=0; i<jpos.size();i++)
    {
      pos[i] = jpos[i] * pos_sign[i];
      vel[i] = jvel[i] * vel_sign[i];
    }

    double diff = (double)(prim_jpos_inc[0] + 19) -((double)(sec_jpos_inc[0] - 24899500))/101;

    if(vel[0] > 0)
    {
      torq_est.data = 9.99*1e-2* diff + 15.212;
    }
    else {
     torq_est.data = 0.1046* diff  -9.7321;
    }

    //        if(!initialized)
    //          std::cout<<jpos.size()<<"\n\n "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" "<<pos[3]<<" "<<pos[4]<<" "<<pos[5]<<" \n";
    //        torq_est.data = 0.1046* diff +0;
  }
//  std::cout<<"220\n";

  for (size_t jnt_cnt = 0; jnt_cnt < NUM_JOINTS; jnt_cnt++)
  {

    state_msg.data[jnt_cnt] = pos[jnt_cnt];
    state_msg.data[jnt_cnt + NUM_JOINTS] = vel[jnt_cnt];
    state_msg.data[jnt_cnt + 2*NUM_JOINTS] = cmd_acc[jnt_cnt];
    if(fabs(cmd_acc[jnt_cnt]) > 3.14)
       state_msg.data[jnt_cnt + 2*NUM_JOINTS] = 0;
  }
//  std::cout<<"230\n";

//  if(drive[0]->initialized())// && svaya_driver->driverStatus() != svaya_driver::DriverMessage::OK)
//  {
//    fout.close();
//    if(svaya_driver->driverStatus() == svaya_driver::DriverMessage::ERROR)
//      RCLCPP_INFO_STREAM(node->get_logger(), RED_MSG   <<"Drive in Error"<< RESET_MSG);
//    else if(svaya_driver->driverStatus() == svaya_driver::DriverMessage::LINKDOWN)
//      RCLCPP_INFO_STREAM(node->get_logger(), RED_MSG  << "link is down" << RESET_MSG);
//      virtual_flag = true;
//      svaya_driver->disableDriver();
//  }
//  std::cout<<"240\n";

  if(drive.size() != 0  && drive[0]->initialized() )//)
  {
      //std::cout<<"375";

//      std::cout<<"Init beg \n";

//      double analog_in1 = fabs((double)analog_in);
//      add_data(analog_vec, drive[0]->scaled_analog_);
//      analog_in1 = mean(analog_vec);
//      analog_fil = 0.001/T_set*analog_in1 + (T_set - 0.001)*analog_prev/T_set;
//      torq_sensor_data.data = ((analog_fil-intercept)*43.1215/slope);//*7.62939E-5*107.4-191-3.7))*0.976;  //43.12/5424;//*7.62939E-5*107.4-191-4.5);
//      analog_prev = analog_fil;
//
    curr_time = rclcpp::Clock().now();

    auto duration = curr_time - prev_time;
    delta_t = duration.seconds();
    if(delta_t > 0.01){
        delta_t = 0.01;
    }
    //RCLCPP_INFO_STREAM(node->get_logger(),"dt "<< delta_t);

    prev_time = curr_time;




//      std::cout<<st_nf;


      std::vector<double> R0;
     R0.resize(8,350);
     std::vector<double> R;
     R.resize(8,350);
     double R1,R2,R3,R4;

     Torq.data.resize(2);

   //  std::cout<<"147\n";

   //        std::cout<<"150\n";
    COG_offset = -1.3581;
     if(startup){
       th = pos[0]+COG_offset*M_PI/180;
       th2 = th;
       startup = false;
     }
     th = th+th_dot*delta_t;
//     th = pos[0]+COG_offset*M_PI/180;
//     th = th*360.0/double(pow(2,20));
     th_dot =  vel[0];
//     RCLCPP_INFO_STREAM(node->get_logger(),"vel"<< th_dot);


//     th_fil = delta_t*th/T_set + (T_set - delta_t)*th_prev/T_set;
//     th_dot_fil = delta_t*th_dot/T_set + (T_set - delta_t)*th_dot_prev/T_set;
//     th_dot_prev = th_dot_fil;
//     th_prev = th_fil;
     th_ddot = 0;
 //    for (int i = 0; i<len-1; i++){
 //        v_buffer[i] = v_buffer[i+1];
 //        th1_buffer[i] = th1_buffer[i+1];
 //    }

     if(initialised == false)
     {
       th2 = th;
       th1 = th;
       th_fil = th;
       th1_dot = th_dot;
       th2_dot = th_dot;
       th_dot_fil = th_dot;
       th2_ddot = th_ddot;
 //      for (int i = 0; i<len; i++){
 //          v_buffer[i] = th_dot;
 //          th1_buffer[i] = th;
 //      }
       initialised = true;
     }
//     k2 = 1;
//     C = 1.25;
//     damping_coeff =75.0;
;//1.4*sqrt(inertia*stiffness);
//     std::cout<<"F " <<(mass)*g*link_length*sin(th) + 2.6*g*0.22*sin(th)<<"\n";
//     if(abs(vel[0])>0.01){
////       std::cout<<acc[0]<<std::endl;
//       if(eff[0]*th2_dot>0){
////         std::cout<<"k2=0.99"<<std::endl;
//         k2 = 1;
//         damping_coeff = C*sqrt(inertia*stiffness*k2);
//        }
//       else{
//         k2 = 1;
//          damping_coeff = C*sqrt(inertia*stiffness*k2);

//        }
//     }
     if(abs(loading_rate) > 400.0){
       counter = 1500;
       stiffness_corr = 0;
     }
     if(counter <1){
       if(abs(vel[0]) >0.02){
         double T1 = 0.5;
         double T2 = 0.5;
         stiffness_corr = 0.35*delta_t/T1 + (T1-delta_t)*stiffness_corr/T1;
         set_time2 = 0.008*delta_t/T2 + (T2 - delta_t)*set_time2/T2;
       }else{
         stiffness_corr = (20-delta_t)*stiffness_corr/20;
         double T3 = 2;
         set_time2 = 0.09*delta_t/T3 + (T3 - delta_t)*set_time2/T3;
       }
     }
     else{
       counter = counter-1;
     }
     loading_rate = ((abs(st_nf1) - abs(st_nf0))/delta_t)*delta_t/0.1 + (0.1-delta_t)*loading_rate/0.1;
     double T4 = 0.001;
     if(loading_rate >20){
       stiffness_corr2 = 0.4*delta_t/T4 + (T4 - delta_t)*stiffness_corr2/T4;
     }else if (loading_rate < -20){
       stiffness_corr2 = -0.4*delta_t/T4 + (T4 - delta_t)*stiffness_corr2/T4;
     }else {
       stiffness_corr2 = (T4-delta_t)*stiffness_corr2/T4;
     }
     double str = (th - th2);
     double str_dot = (th_dot - th2_dot);
     double str_ddot = (str_dot - str_dot_prev)/delta_t;
     str_dot_prev = str_dot;
     double c1 = k1*set_time1;
     double c2 = k2*set_time2;
     Fk_ddot = (stiffness*(1-0*(stiffness_corr + stiffness_corr2))*str
                + (stiffness*(1-0*(stiffness_corr+stiffness_corr2)) + k1)*str_dot*c1/k1
                + (stiffness*(1-0*(stiffness_corr+stiffness_corr2)) + k2)*str_dot*c2/k2
                + (stiffness*(1-0*(stiffness_corr+stiffness_corr2)) + k1 + k2)*c1*c2*str_ddot/(k1*k2)
                - (c1/k1 + c2/k2)*Fk_dot - Fk)*k1*k2/(c1*c2);
//     std::cout<<97.5*sin(th)<<std::endl;
     Fk_dot = Fk_dot + Fk_ddot*delta_t;
     Fk = Fk + Fk_dot*delta_t;
     th2_ddot = ((mass)*g*link_length*sin(th) + 6.94*g*0.241*sin(th) + Fk)/inertia;
     th1_dot_prev = th1_dot;

     th2 = th2 + th2_dot * delta_t + 0.5*th2_ddot*delta_t*delta_t;//+ 0.5 * th2_ddot * delta_t * delta_t;
     th2_dot = th2_dot +  th2_ddot * delta_t;


       double delta_th = th- th2;
       tension = 1.0;
       compression = 1.0;
       if(delta_th > 0){
         tension = 1.0;
         compression = 0.9;
          }
       if(delta_th < 0){
         tension = 0.9;
         compression = 1.0;
          }
       R[0] = R0[0] + R0[0]*strain_gauge_factor*( delta_th  )*tension;
       R[1] = R0[0] + R0[0]*strain_gauge_factor*(-delta_th)*compression;
       R[2] = R0[0] + R0[0]*strain_gauge_factor*(delta_th)*tension;
       R[3] = R0[0] + R0[0]*strain_gauge_factor*( -delta_th )*compression;
       R[4] = R0[0] + R0[0]*strain_gauge_factor*( delta_th)*tension;
       R[5] = R0[0] + R0[0]*strain_gauge_factor*( -delta_th )*compression;
       R[6] = R0[0] + R0[0]*strain_gauge_factor*( delta_th )*tension;
       R[7] = R0[7] + R0[0]*strain_gauge_factor*( -delta_th )*compression;
       R1 = R[0]*R[6]/(R[0] + R[6]);
       R2 = R[1]*R[7]/(R[1] + R[7]);
       R3 = R[3]*R[5]/(R[3] + R[5]);
       R4 = R[4]*R[2]/(R[4] + R[2]);

//       torq = (-97.84*8.34156*stiffness/1350)*((R1*R4-R2*R3)/((R1+R2)*(R3+R4))) ;

       torq = (-320.33*stiffness/605)*((R1*R4-R2*R3)/((R1+R2)*(R3+R4))) ;
       torq_fil_lp = delta_t*torq/T_set + (T_set-delta_t)*torq_prev/T_set;

       jt_dot = (torq_fil_lp - torq_fil_lp1)/delta_t;
       jt_ddot = (torq_fil_lp + torq_fil_lp2 - 2.0*torq_fil_lp1)/(delta_t*delta_t);

       torq_fil_lp2 = torq_fil_lp1;
       torq_fil_lp1 = torq_fil_lp;

       torq_nf = jt_ddot + 2*z*w*jt_dot + w*w*torq_fil_lp - torq_nf1*( -2/(delta_t*delta_t) + w*w + w*(a*a + 1)/(a*delta_t) ) - torq_nf0*( 1/(delta_t*delta_t) - w*(a*a + 1)/(a*delta_t) );
       torq_nf = torq_nf*delta_t*delta_t;
       torq_nf0 = torq_nf1;
        torq_nf1 = torq_fil_lp;

      // std::cout<<"torq_fil" <<torq<<"\n";



   //    std::cout<<"Delta theta: "<<delta_th<<"\n";
   //    std::cout<<"Eq resistance: "<<(R1*R4-R2*R3)/((R1+R2)*(R3+R4))<<"\n";

   //    std::cout<<"f "<<f<<"\n";


       torq_prev = torq_fil_lp;

       Torq.data[0] = torq;




   //RCLCPP_INFO_STREAM(node->get_logger(),"462                             ");
      analog_data.data=st;
      analog_signal->publish(analog_data);

        st = 0.001*slope*(st-intercept);
        st_dot = (st - st1)/delta_t;
        st_ddot = (st + st2 - 2.0*st1)/(delta_t*delta_t);

        st2 = st1;
        st1 = st;
  //      torq_nf = jt_ddot + 2*z*w*jt_dot + w*w*torq_fil_lp - torq_nf1*( -2/(delta_t*delta_t) + w*w + w*(a*a + 1)/(a*delta_t) ) - torq_nf0*( 1/(delta_t*delta_t) - w*(a*a + 1)/(a*delta_t) );

        st_nf = st_ddot + 2*z*w*st_dot + w*w*st - st_nf1*( -2/(delta_t*delta_t) + w*w + w*(a*a + 1)/(a*delta_t) ) - st_nf0*( 1/(delta_t*delta_t) - w*(a*a + 1)/(a*delta_t) );
        st_nf = st_nf*delta_t*delta_t;
        st_nf0 = st_nf1;
         st_nf1 = st_nf;
      torq_buffer.push_back(-st_nf);




           if(torq_buffer.size()>1){
//             analog_scaled_data.data = 0.0295235546*torq_buffer[0]-947.0232602;//+ 0.25*cos(2*(th-18*3.1415/180));
             analog_scaled_data.data =(torq_buffer[0]);//+ 0.25*cos(2*(th-18*3.1415/180));
//             analog_scaled_data.data = (torq_buffer[0]);//+ 0.25*cos(2*(th-18*3.1415/180));

             analog_scaled_signal->publish(analog_scaled_data);

             fout<<th << "," 
                << vel[0] << "," 
                <<Torq.data[0]<< "," 
                << analog_scaled_data.data<<","
               <<delta_t<<","
               <<torq_buffer[0]<<","
               <<dc_lv.data[0]<<","
               <<motor_torq.data[0]<<","
               <<cmd_vel[0]<<","
               <<cmd_acc[0]<<"\n";
             torq_buffer.erase(torq_buffer.begin());
           }
//           if(th_dot > 0){
//             Torq.data[0] = torq - 1.5;
//           }

      torq_sens->publish(torq_sensor_data);
      error.data = (Torq.data[0] - analog_scaled_data.data);
      error.data = (Torq.data[0] - analog_scaled_data.data)*(1-exp(-1.4*abs(vel[0])));
//      std::cout<<counter<<std::endl;
//      if(abs(error.data)>30){
////        counter = 5000;
//        error.data = (Torq.data[0] - analog_scaled_data.data)*(1-exp(-0.3*abs(vel[0])));

//      }

//      if(counter<1){
//        error.data = (Torq.data[0] - analog_scaled_data.data)*(1-exp(-2.0*abs(vel[0])));

//      }else{
//        counter = counter - 1;
//        error.data = (Torq.data[0] - analog_scaled_data.data)*(1-exp(-0.3*abs(vel[0])));
//      }
      error_pub->publish(error);
      state_pub->publish(state_msg);
      model_torq_pub->publish(Torq);

      enc_pos_pub->publish(enc_pos_data);
      torq_est_pub->publish(torq_est);
      motor_torq_pub->publish(motor_torq);
      dc_link_vol_pub->publish(dc_lv);
      target_torq_pub->publish(target_torque);
//      std::cout<<"Init end \n";

   }

//  FTS->sample_rate = 3;
//  FTS->low_pass_filter_fc = 1;


// fts_sensor_data.data[0] = static_cast<double>(FTS->fx_)/1000000;
// fts_sensor_data.data[1] = static_cast<double>(FTS->fy_)/1000000;
// fts_sensor_data.data[2] = static_cast<double>(FTS->fz_)/1000000;
// fts_sensor_data.data[3] = static_cast<double>(FTS->tx_)/1000000;
// fts_sensor_data.data[4] = static_cast<double>(FTS->ty_)/1000000;
// fts_sensor_data.data[5] = static_cast<double>(FTS->tz_)/1000000;

// sensor_publisher->publish(fts_sensor_data);

  rclcpp::spin_some(node);
  rclcpp::sleep_for(1ms);

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type SvayaHW::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
//    std::cout<<"Write beg \n";
  std::vector<double> jpos, jvel;

  jpos.resize(NUM_JOINTS);
  jvel.resize(NUM_JOINTS,0);
  if(virtual_flag)
  {

    for (size_t jnt_cnt = 0; jnt_cnt < info_.joints.size(); jnt_cnt++)
    {
      dummy_pos[jnt_cnt] = cmd_pos[jnt_cnt];
      dummy_vel[jnt_cnt] = cmd_vel[jnt_cnt];
      dummy_eff[jnt_cnt] = cmd_eff[jnt_cnt];
    }
    //      RCLCPP_INFO_STREAM(node->get_logger(),"Joint Pos Write "<<cmd_pos[4]<<" Joint Vel "<<cmd_vel[4]<<" \n");
  }
  else if(drive.size()!=0 && drive[0]->initialized())
  {
    if(!initialized )
    {
//      svaya_driver->getJointPos(jpos);
      //          std::cout<<jpos.size()<<" "<<jpos[0]<<" "<<jpos[1]<<" "<<jpos[2]<<" "<<jpos[3]<<" "<<jpos[4]<<" "<<jpos[5]<<" \n";
      //          svaya_driver->writeJointCommands(jpos,svaya_driver::ControlMode::Position);

      for(size_t i= 0; i<NUM_JOINTS; i++){
        jpos[i] = static_cast<double>(drive[0]->position_)/enc_count[0]*2*M_PI;
        jvel[i] = 0;
//        std::cout<<jpos[i]<<std::endl;
      }

//      drive[0]->target_position_ = drive[0]->position_;

      RCLCPP_INFO_STREAM(node->get_logger(),"Pos: "<<drive[0]->position_);
      for(int i=0; i<cmd_pos.size(); i++)
      {
        cmd_pos[i] = pos_sign[i] * jpos[i];
        cmd_vel[i] = vel_sign[i] * 0;
      }
      initialized = true;
    }
    else if(initialized)
    {
      //          jpos[0] = cmd_pos[0];
      for(int i=0; i<cmd_pos.size(); i++)
      {
        jpos[i] = pos_sign[i] * cmd_pos[i];
        jvel[i] = vel_sign[i]* cmd_vel[i];
      }


    }
    if(gchold){
      support_torq = -(gc_sens*Torq.data[0] - (gc_sens-1)*analog_scaled_data.data)*1000.0/(rated_torq[0]*100);

      if(gcstart){
        support_torq_prev = support_torq;
        gcstart = false;
      }
      double T_st_lp = 0.05;
      support_torq_smooth = delta_t*support_torq/T_st_lp + (T_st_lp-delta_t)*support_torq_prev/T_st_lp;

         drive[0]-> target_torque_ = support_torq_smooth;
         support_torq_prev = support_torq_smooth;

    }else{
      for(size_t joint=0; joint<NUM_JOINTS; joint++){
        if(control_mode == 8){
          drive[joint]->target_position_ = jpos[joint]*enc_count[joint]/(2*M_PI);
        }
        else if (control_mode == 9) {
          drive[joint]->target_velocity_ = jvel[joint]/(2*M_PI)*60*100;

        }
        drive[joint]->target_velocity_ = jvel[joint]/(2*M_PI)*60*100;

//        std::cout<<jpos[joint]<<std::endl;
//        std::cout<<drive[0]->position_<<std::endl;

      }
//      drive[0]->target_velocity_ = jvel[0]/(2*M_PI)*60*100;
//      drive[1]->target_velocity_ = jvel[1]/(2*M_PI)*60*100;
      gcstart = true;
    }

//    std::cout<<drive[0]->target_torque_<<std::endl;
//    drive[0]->target_position_ = drive[0]->position_;

//    RCLCPP_INFO_STREAM(node->get_logger(),"jvel : "<< jvel[0]);
//    RCLCPP_INFO_STREAM(node->get_logger(),"Actual vel: "<< drive[0]->velocity_);
//    RCLCPP_INFO_STREAM(node->get_logger(),"Target vel: "<< drive[0]->target_velocity_);
    //        std::cout<<jpos.size()<<"\n\n "<<jpos[0]<<" "<<jpos[1]<<" "<<jpos[2]<<" "<<jpos[3]<<" "<<jpos[4]<<" "<<jpos[5]<<" \n";
//    svaya_driver->writeJointCommands(jpos,svaya_driver::ControlMode::Position);
//    svaya_driver->writeJointCommands(jvel,svaya_driver::ControlMode::Velocity);
//    svaya_driver->writePositionCommandsWithOffset(jpos, jvel);
  }
//  std::cout<<"Write end \n";

//  master.update()
  bool temp_check = false;
  bool temp_ready = false;
master.update(&temp_check, &temp_ready);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SvayaHW::Slave_obj_assignment(knexo_ethercat::Master &master, std::vector<knexo_ethercat::Synapticon_DC1K_D3 *> &drive)
{
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

            FTS = (((knexo_ethercat::ft_sensor *)*(obj)));

        }
        break;
      }
    }
}

void SvayaHW::controller_status()
{

  std::vector<std::string> param_list = {"controller_status"};
  rclcpp::spin_some(node);

  auto param = parameter_client->get_parameters(param_list);

  joint_trajectory_controller_status =  param[0].as_bool();

  RCLCPP_INFO_STREAM(node->get_logger(),"controller status : "<<joint_trajectory_controller_status);

}

void SvayaHW::read_parameters()
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
//    if(param[i].get_name() == "encoder_cnt")
//    {
//      enc_count = param[i].as_integer_array();
////      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter encoder count: "<<enc_count[0]<<" "<<enc_count[1]<<" "<<enc_count[2]<<" "<<enc_count[3]<<" "<<enc_count[4]<<" "<<enc_count[5]);
//    }
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
    if(param[i].get_name() == "strain_gauge_factor")
    {
      strain_gauge_factor = param[i].as_double();
//      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter Joint Rated Torque: "<<rated_torq[0]<<" "<<rated_torq[1]<<" "<<rated_torq[2]<<" "<<rated_torq[3]<<" "<<rated_torq[4]<<" "<<rated_torq[5]);
    }
    if(param[i].get_name() == "link_length")
    {
      link_length = param[i].as_double();
//      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter Joint Rated Torque: "<<rated_torq[0]<<" "<<rated_torq[1]<<" "<<rated_torq[2]<<" "<<rated_torq[3]<<" "<<rated_torq[4]<<" "<<rated_torq[5]);
    }
    if(param[i].get_name() == "mass")
    {
      mass = param[i].as_double();
//      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter Joint Rated Torque: "<<rated_torq[0]<<" "<<rated_torq[1]<<" "<<rated_torq[2]<<" "<<rated_torq[3]<<" "<<rated_torq[4]<<" "<<rated_torq[5]);
    }
    if(param[i].get_name() == "f")
    {
      f = param[i].as_double();
//      RCLCPP_INFO_STREAM(node->get_logger(), "Reading parameter Joint Rated Torque: "<<rated_torq[0]<<" "<<rated_torq[1]<<" "<<rated_torq[2]<<" "<<rated_torq[3]<<" "<<rated_torq[4]<<" "<<rated_torq[5]);
    }
  }
  inertia = mass*link_length*link_length + 0.634;
  parameter_reading_done = true;
  RCLCPP_INFO_STREAM(node->get_logger(),"Read Params done: "<<parameter_reading_done);

}

double SvayaHW::mean(std::vector<double> &diff_vec)
{
    double avg = 0;
    for(int l =0; l < diff_vec.size(); l++)
    {
        avg = avg + diff_vec[l];
    }
    avg = avg/diff_vec.size();
    return avg;
}

void SvayaHW::add_data(std::vector<double> &diff_vec, double data)
{
    for (int l=0; l < diff_vec.size() -1 ; l++)
    {
      diff_vec[l] = diff_vec[l+1];
    }
    diff_vec.back() = data;
}

void SvayaHW::controller_callback(const rclcpp::Parameter &P)
{
  joint_trajectory_controller_status = P.as_bool();
}

} //namespace svaya_hw



PLUGINLIB_EXPORT_CLASS(svaya_hw::SvayaHW,hardware_interface::SystemInterface)
