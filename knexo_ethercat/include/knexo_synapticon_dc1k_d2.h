#ifndef SIMPLECAT_Synapticon_DC1K_D3_H_
#define SIMPLECAT_Synapticon_DC1K_D3_H_

//---------------------Shared Memory and Semaphores headers---------------------
#include <semaphore.h>
#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <sys/shm.h>
#include <sys/mman.h>
#include <error.h>
#include <math.h>
#include <iostream>
#include <typeinfo>
//------------------------------------------------------------------------------
#include <knexo_ethercat_slave.h>
#include <global_shared_variables.h>
#include<fstream>
#include <ctime>
#define SOMANET_ID 0x000022d2, 0x00000301

#define CAN_OD_CONTROL_WORD     0x6040      /* RX; 16 bit */
#define CAN_OD_MODES            0x6060      /* RX; 8 bit */
#define CAN_OD_TORQUE_TARGET    0x6071      /* RX; 16 bit */
#define CAN_OD_POSITION_TARGET  0x607A      /* RX; 32 bit */
#define CAN_OD_VELOCITY_TARGET  0x60ff      /* RX; 32 bit */
//#define CAN_OD_BRAKE_STATUS     0x2004

#define CAN_OD_STATUS_WORD      0x6041      /* TX; 16 bit */
#define CAN_OD_MODES_DISPLAY    0x6061      /* TX; 8 bit */
#define CAN_OD_TORQUE_VALUE     0x6077      /* TX; 16 bit */
#define CAN_OD_POSITION_VALUE   0x6064      /* TX; 32 bit */
#define CAN_OD_VELOCITY_VALUE   0x606C      /* TX; 32 bit */

/*Controlword Bits*/
#define SWITCH_ON_CONTROL                   0x1
#define ENABLE_VOLTAGE_CONTROL              0x2
#define QUICK_STOP_CONTROL                  0x4
#define ENABLE_OPERATION_CONTROL            0x8
#define OPERATION_MODES_SPECIFIC_CONTROL    0x70  /*3 bits*/
#define FAULT_RESET_CONTROL                 0x80
#define HALT_CONTROL                        0x100
#define OPERATION_MODE_SPECIFIC_CONTROL     0x200
#define RESERVED_CONTROL                    0x400
#define MANUFACTURER_SPECIFIC_CONTROL       0xf800

/*Statusword Bits*/
#define READY_TO_SWITCH_ON_STATE            0X1
#define SWITCHED_ON_STATE                   0X2
#define OPERATION_ENABLED_STATE             0X4
#define FAULT_STATE                         0X8
#define VOLTAGE_ENABLED_STATE               0X10
#define QUICK_STOP_STATE                    0X20
#define SWITCH_ON_DISABLED_STATE            0X40
#define WARNING_STATE                       0X80
#define MANUFACTURER_SPECIFIC_STATE         0X100
#define REMOTE_STATE                        0X200
#define TARGET_REACHED_OR_RESERVED_STATE    0X400
#define INTERNAL_LIMIT_ACTIVE_STATE         0X800
#define OPERATION_MODE_SPECIFIC_STATE       0X1000  // 12 CSP/CSV/CST  13
#define MANUFACTURER_SPECIFIC_STATES        0XC000  // 14-15


#define DRIVE_TEMPERATURE                   0x2031
#define ERROR_CODE                          0x603F


/*!
* \brief knexo_ethercat : create a namespace.
*/
namespace knexo_ethercat {

/*!
* \brief Synapticon_DC1K_D3 : create a class name Synapticon_DC1K_D3 and it inherited with Slave class.
*/
class Synapticon_DC1K_D3 : public Slave
{

public:
  int hex_;
  float scaled_analog_;
  /*!
  * \brief Synapticon_DC1K_D3() : create a constructor.
  */
   Synapticon_DC1K_D3() : Slave() {



   }
  /*!
  * \brief ~Synapticon_DC1K_D3() : create a Destructor.
  */
  virtual ~Synapticon_DC1K_D3() {}

  /** Returns true if Elmo has reached "operation enabled" state.
       *  The transition through the state machine is handled automatically. */
  bool initialized() const {return initialized_;}
 /*!
  * \brief config_sdo() : It will configure the PDO.
  */
  void  config_sdo()
  {

  }

/*!
* \brief processData() : it process the data and write in domain.
*/
  virtual int processData(size_t index, uint8_t* domain_address)
  {
    // DATA READ WRITE
    int sub_id,id;
    id=channels_[index].index;
    switch(id)
    //switch(index)
    {
    case 0x6040:
      control_word_ = EC_READ_U16(domain_address);
      cwRead_=control_word_;
      // ROS_INFO_STREAM(" control_word_   "<<control_word_<<" "<<std::hex<<id<<" "<<index<<" "<<std::endl);

      if ( control_word_ < 7 && enable_motor == false )
      {
        control_word_ =  set_control2(state_, control_word_);

      }
      else if (enable_motor == true)
      {
        // it should make drie full operational
        control_word_ =  set_control(state_, control_word_);
      }
      else if(*estop_triggered)
        control_word_ = 2;
      else if( control_word_  >= 7 && enable_motor == false)
      {
        std::cout<<"Making Control word 0\n";
        control_word_ = 0;
        if(status_word_ & 0b1000 != 0b1000)
          control_word_ = 0x80;
      }

       // ROS_INFO_STREAM(" Sent control_word_ "<<control_word_<<std::endl);
      EC_WRITE_U16(domain_address, control_word_);

      break;
    case 0x6060:
     // ROS_INFO_STREAM(" mode_of_operation_   "<<std::hex<<id<<" "<<index<<" "<<mode_of_operation_<<std::endl);
      EC_WRITE_S8(domain_address, mode_of_operation_);
      break;
    case 0x6071:
     // ROS_INFO_STREAM(" target_torque_   "<<std::hex<<id<<" "<<index<<" "<<target_torque_<<std::endl);
      EC_WRITE_S16(domain_address, target_torque_);
      break;

//    case 0x2004:
//      EC_WRITE_U8(domain_address, brake_status);
//      std::cout<<(domain_address)<<std::endl;
//      if(erro_code==0x3331 || erro_code==0x2220)
//      {
//        printf("DRIVE %d brake status %d\n",slave_index,brake_status);
//      }
//    break;
    case 0x607a:
     // ROS_INFO_STREAM(" target_position_  "<<std::hex<<id<<" "<<index<<" "<<target_position_<<std::endl);
      EC_WRITE_S32(domain_address, target_position_);
     // Ethercat_data.joint_count_sent[slave_index]=target_position_;
      break;
    case 0x60ff:
    // ROS_INFO_STREAM(" target_velocity_   "<<std::hex<<id<<" "<<index<<" "<<target_velocity_<<std::endl);
      EC_WRITE_S32(domain_address, target_velocity_);

      break;
    case 0x60b2:
    // ROS_INFO_STREAM(" torque_offset_   "<<std::hex<<id<<" "<<index<<" "<<torque_offset_<<std::endl);
      EC_WRITE_S16(domain_address,torque_offset_);

      break;
    case 0x2701:
    // ROS_INFO_STREAM(" tunning_command_  "<<std::hex<<id<<" "<<index<<" "<<tunning_command_<<std::endl);
      EC_WRITE_U32(domain_address,tunning_command_);

      break;

    // case 7:
    //   EC_WRITE_S8(domain_address,digital_output_[0]);

    //   break;
    // case 8:
    //   //offset
    //   break;
    // case 9:
    //   EC_WRITE_S8(domain_address,digital_output_[1]);
    //   break;
    // case 10:
    //   //offset
    //   break;
    // case 11:
    //   //std::cout<<"do3 "<<digital_output_[2]<<" daddress "<<std::hex<<*domain_address<<std::endl;
    //   EC_WRITE_S8(domain_address,digital_output_[2]);
    //   break;
    // case 12:
    //   //offset
    //   break;
    // case 13:

    //   //std::cout<<"do4 "<<digital_output_[3]<<" daddress "<<std::hex<<*domain_address<<std::endl;
    //   EC_WRITE_S8(domain_address,digital_output_[3]);
    //   break;
    //   //   case 14:
    //   //       //offset
    //   //       break;
    // case 14:
    //   //offset
    //   break;
    case 0x2703:
      // ROS_INFO_STREAM(" user_mosi_   "<<std::hex<<id<<" "<<index<<" "<<user_mosi_<<std::endl);
      EC_WRITE_U32(domain_address, user_mosi_);
      break;
    case 0x60b1:
     // ROS_INFO_STREAM(" velocity_offset_   "<<std::hex<<id<<" "<<index<<" "<<velocity_offset_<<std::endl);
      EC_WRITE_S32(domain_address, velocity_offset_);

      break;

    case 0x6041:
      status_word_ = EC_READ_U16(domain_address);
      // ROS_INFO_STREAM(" status_word_   "<<std::hex<<id<<" "<<index<<" "<<status_word_<<std::endl);
      state_ = deviceState(status_word_);
       // ROS_INFO_STREAM(" state_   "<<state_<<std::endl);
      break;
    case 0x6061:
       mode_of_operation_display_ = EC_READ_S8(domain_address);
        // ROS_INFO_STREAM(" mode_of_operation_display_   "<<std::hex<<id<<" "<<index<<" "<<mode_of_operation_display_<<std::endl);

      break;
    case 0x6064:
      position_ = EC_READ_S32(domain_address);
        // Ethercat_data.joint_drive_encoder[slave_index]=position_;
      // ROS_INFO_STREAM(" position_   "<<id<<" "<<slave_index<<" "<<Ethercat_data.joint_drive_encoder[slave_index]<<std::endl);

      break;
    case 0x606c:
      velocity_ = EC_READ_S32(domain_address);
      // ROS_INFO_STREAM(" velocity_   "<<std::hex<<id<<" "<<index<<" "<<velocity_<<std::endl);

      break;
    case 0x6077:
      torque_ = EC_READ_S16(domain_address);
//      std::cout<<torque_<<std::endl;
//       RCLCPP_INFO_STREAM(node->get_logger()," torque_   "<<std::hex<<id<<" "<<index<<" "<<torque_<<std::endl);
      // --------------Low Pass Final Torque----------------
//      torque_=(double)torque_*(dt/(dt+1/(2*M_PI*t_fc))) + prev_torq*((1/(2*M_PI*t_fc))/(dt+1/(2*M_PI*t_fc)));
      prev_torq=torque_;
      // ROS_INFO_STREAM(" prev_torq  "<<prev_torq<<std::endl);
      //--------------------------------------------------

      break;
    case 0x230a:
      second_position_ = EC_READ_S32(domain_address);
      // ROS_INFO_STREAM(" second_position_   "<<std::hex<<id<<" "<<index<<" "<<second_position_<<std::endl);

      break;
    case 0x230b:
      second_velocity_ = EC_READ_S32(domain_address);
      // ROS_INFO_STREAM(" second_velocity_   "<<std::hex<<id<<" "<<index<<" "<<second_velocity_<<std::endl);

      break;
    case 0x2038:
         //std::cout<<std::hex<<EC_READ_U32(domain_address)<<"\n";

       //  ss<<std::hex<<EC_READ_U32(domain_address);
         //std::cout<<"String: "<<ss.str()<<"\n\n";
         hex_ = EC_READ_U32(domain_address);
         scaled_analog_ = *((float*)&hex_);
         break;
     case 0x2401:
       analog_input_1_ = EC_READ_U16(domain_address);
       //std::cout<<"analog_input_1_ "<<analog_input_1_<<" "<<EC_READ_U16(domain_address)<<"\n";

      break;
     case 0x2402:
       analog_input_2_ = EC_READ_U16(domain_address);
       //std::cout<<"analog_input_2_ "<<analog_input_2_<<" "<<EC_READ_U16(domain_address)<<"\n";

      break;
     case 0x2403:
       analog_input_3_ = EC_READ_U16(domain_address);


       break;

     case 0x2404:
       analog_input_4_ = EC_READ_U16(domain_address);

       break;

    case 0x2702:
      tunning_status_ = EC_READ_U32(domain_address);
       // ROS_INFO_STREAM(" tunning_status_   "<<std::hex<<id<<" "<<index<<" "<<tunning_status_<<std::endl);

      break;
    // case 29:
    //     digital_input_[0] = EC_READ_S8(domain_address);

    //   break;
    // case 30:

    //   break;
    // case 31:
    //   digital_input_[1] = EC_READ_S8(domain_address);
    //   break;
    // case 32:

    //   break;
    // case 33:
    //   digital_input_[2] = EC_READ_S8(domain_address);
    //   break;
    // case 34:

    //   break;
    // case 35:
    //   digital_input_[3] = EC_READ_S8(domain_address);
    //   break;
    // case 36:

    //   break;
    case 0x2704:
      user_miso_ = EC_READ_U32(domain_address);
       // ROS_INFO_STREAM(" user_miso_   "<<std::hex<<id<<" "<<index<<" "<<user_miso_<<std::endl);
      break;
    case 0x20f0:
      time_stamp_ = EC_READ_U32(domain_address);
       // ROS_INFO_STREAM(" time_stamp_   "<<std::hex<<id<<" "<<index<<" "<<time_stamp_<<std::endl);

      break;
    case 0x60fc:
      position_demand_internal_value = EC_READ_S32(domain_address);
       // ROS_INFO_STREAM(" position_demand_internal_value   "<<std::hex<<id<<" "<<index<<" "<<position_demand_internal_value<<std::endl);

      break;
    case 0x606b:
      velocity_demand_value = EC_READ_S32(domain_address);
       // ROS_INFO_STREAM(" velocity_demand_value   "<<std::hex<<id<<" "<<index<<" "<<velocity_demand_value<<std::endl);
       break;
    case 0x6074:
      torque_demand = EC_READ_S16(domain_address);
       // ROS_INFO_STREAM(" torque_demand   "<<std::hex<<id<<" "<<index<<" "<<torque_demand<<std::endl);
      break;
    case 0x20e1:


      for(int i=0;i<100;i++)
      {
        resolution_data[i]= EC_READ_U8(domain_address);
        domain_address++;
      }
       // ROS_INFO_STREAM(" resolution_data   "<<std::hex<<id<<" "<<index<<" "<<resolution_data[0]<<std::endl);
      // break;
    // case 43:


    //   for(int i=25;i<50;i++)
    //   {
    //     resolution_data[i]= EC_READ_U8(domain_address);
    //     domain_address++;
    //   }
    //   break;
    // case 44:


    //   for(int i=50;i<75;i++)
    //   {
    //     resolution_data[i]= EC_READ_U8(domain_address);
    //     domain_address++;
    //   }
    //   break;
    // case 45:


    //   for(int i=75;i<100;i++)
    //   {
    //     resolution_data[i]= EC_READ_U8(domain_address);
    //     domain_address++;
    //   }

      ptr=resolution_data;

      high_res_time_smp=(*(uint8_t*)(ptr));
      for(int i=0;i<4;i++)
      {
        high_res_pos[i]=(*(int*)(ptr+4+(4*i)));
      }

      for(int i=0;i<4;i++)
      {
        high_res_vel[i]=(*(int*)(ptr+20+(4*i)));
      }

      for(int i=0;i<16;i++)
      {
        IA[i]=(*(int16_t*)(ptr+36+(4*i)));
        IB[i]=(*(int16_t*)(ptr+36+(4*i+2)));
      }
      for(int i=0;i<4;i++)
      {

        filtered_pos_buf[i]=high_res_pos[i]*(pdt/(1/(2*M_PI*p_fc)+pdt))+prev_pos*(1/(2*M_PI*p_fc)/(1/(2*M_PI*p_fc)+pdt));
        prev_pos=filtered_pos_buf[i];
        filtered_vel_buf[i]=high_res_vel[i]*(pdt/(1/(2*M_PI*v_fc)+pdt))+prev_vel*(1/(2*M_PI*v_fc)/(1/(2*M_PI*v_fc)+pdt));
        prev_vel=filtered_vel_buf[i];
        //        if (slave_index == 0)
        //        {
        //          std::cout<<"filtered_vel_buf[i] : "<<filtered_vel_buf[i]<<", high_res_vel[i] : "<<high_res_vel[i]<<std::endl;
        //          std::cout<<"1st term : "<<high_res_vel[i]*(pdt/(1/(2*M_PI*v_fc)+pdt))<<", coeff : "<<(pdt/(1/(2*M_PI*v_fc)+pdt))<<std::endl;
        //          std::cout<<"2nd term : "<<prev_vel*(1/(2*M_PI*v_fc)/(1/(2*M_PI*v_fc)+pdt))<<", coeff : "<<(1/(2*M_PI*v_fc)/(1/(2*M_PI*v_fc)+pdt))<<std::endl;
        //        }

      }
      filter_position=(int)prev_pos;
      filter_velocity=(int)prev_vel;

      //      if (slave_index == 0)
      //      {
      //        std::cout<<"filter_velocity : "<<filter_velocity<<std::endl;
      //      }



      //      for(int i=0;i<16;i++)
      //      {

      //        filtered_torque_buf[i]=kt[slave_index]*sqrt(IA[i]^2+IB[i]^2+(IA[i]+IB[i])^2);
      //        filtered_torque_buf[i]=filtered_torque_buf[i]*(tdt/(1/(2*M_PI*t_fc)+tdt))+prev_torq*(1/(2*M_PI*t_fc)/(1/(2*M_PI*t_fc)+tdt));
      //        prev_torq=filtered_torque_buf[i];
      //      }
      //      filter_torque=prev_torq;

         return 4;
      break;
      case 0x603F:
         erro_code=  EC_READ_U16(domain_address);

          // ROS_INFO_STREAM(" erro_code   "<<std::hex<<id<<" "<<index<<" "<<erro_code<<std::endl);
         if(erro_code!=0)
         {
          printf("DRIVE ERROR CODE  %d %x\n",slave_index,erro_code);
           std::cout << "Drive "<< slave_index << " STATE: " << device_state_str_[state_] <<"control_word "<<std::hex<<control_word_<<"status_word_ "<<status_word_<< std::endl;
         }

       break;
      case 0x6079:
         dc_linkVoltage =  EC_READ_U32(domain_address);

         // std::cout<<"dc_linkVoltage"<<std::hex<<id<<" "<<index<<" "<<" "<<dc_linkVoltage<<"\n";
         if(erro_code==0x3331 || erro_code==0x2220)
         {
           printf("DRIVE %d Bus Voltage %d in mv\n",slave_index,dc_linkVoltage);
         }
       break;

//       std::cout<<brake_status<<"\n";

    default:
      std::cout<< "WARNING. Synapticon_DC1K_D3 pdo index out of range." <<std::endl;
    }

    // CHECK FOR STATE CHANGE
    if (index==27) //if last entry  in domain
    {
      log.control_word_=control_word_;
      log.status_word_=status_word_;
      log.cwRead_=cwRead_;
      log.state_=state_;

      if (status_word_ != last_status_word_){
        state_ = deviceState(status_word_);
        // status word change does not necessarily mean state change
        // http://ftp.beckhoff.com/download/document/motion/ax2x00_can_manual_en.pdf
        // std::bitset<16> temp(status_word_);
        // std::cout << "STATUS WORD: " << temp << std::endl;
        if (state_ != last_state_)
        {

          if(state_ == STATE_OPERATION_ENABLED)
          {
            //ENABLED
            control_word_ = 0;


          }
          else if(state_ == STATE_FAULT || state_ == STATE_FAULT_REACTION_ACTIVE)
          {
            //FAULT
            drive_fault_state.set(slave_index,true);
            drive_warn_state.set(slave_index,false);
          }
          // else if(state_ == STATE_WARN)
          // {
          //   drive_warn_state.set(slave_index,true);
          //   drive_fault_state.set(slave_index,false);
          // }
          else if(state_ == STATE_READY_TO_SWITCH_ON)
          {
            //This is part of enabled state
          }
          else if(state_==STATE_SWITCH_ON_DISABLED)
          {
            //DISABLED
            drive_fault_state.set(slave_index,false);
            drive_warn_state.set(slave_index,false);
          }


          std::cout << "Drive "<< slave_index << " STATE: " << device_state_str_[state_] << std::endl;
        }
      }
//      std::cout<<"Current State: "<<state_<<"Previous State: "<<last_state_<<"\n";
//      if (state_ == STATE_SWITCH_ON || state_ == STATE_OPERATION_ENABLED )//
     if((state_ == STATE_OPERATION_ENABLED)&&(last_state_ == STATE_OPERATION_ENABLED))
      {
        initialized_ = true;
      }
     else
      {
        initialized_ = false;
      }
      last_status_word_ = status_word_;
      last_state_ = state_;

    }
      return 1;
  }
/** Sync manager configuration information(syncs).
 *
 * This can be use to configure multiple sync managers including the PDO
 * assignment and PDO mapping. It is used as an input parameter type in
 * ecrt_slave_config_pdos().
 */
  virtual const ec_sync_info_t* syncs() { return &syncs_[0]; }

/** number of elements in the syncs array. */
  virtual size_t syncSize() {
    return sizeof(syncs_)/sizeof(ec_sync_info_t);
  }
/*!
* \brief channels() : pdo entry configuration
*/
  virtual const ec_pdo_entry_info_t* channels() {
    return channels_;
  }

  virtual void domains(DomainMap& domains) const {
    domains = domains_;
  }



  uint8_t *ptr;
  int slave_index = 0;
  int cwRead_;
  double filter_torque;
  int filter_position;
  int  filter_velocity;
  double filtered_torque_buf[16];
  int16_t IA[16];
  int16_t IB[16];
  int32_t high_res_pos[4];
  int32_t high_res_vel[4];
  uint32_t high_res_time_smp;

  int filtered_pos_buf[4];
  double filtered_vel_buf[4];
  double prev_pos=0;
  double prev_vel=0,prev_torq=0;

  //1st order Low Pass : y(i)=x(i)*(dt/(dt+RC)) + y(i-1)*(RC/(dt+RC))
  //-----------Low Pass cutoff Hz-------------
  double t_fc;
  double p_fc;
  double v_fc;
  //------------------------------------------


  //  double kt[6]={0.146,0.146,0.146,0.103,0.103,0.103};
  double dt=1/(1000.0);
  double pdt=1/(4000.0);
  //  double tdt=1/(16000.0);

  uint16_t control_word_              = 0; // write
  int8_t   mode_of_operation_         = 0; // write (use enum ModeOfOperation for convenience)
  int16_t  target_torque_             = 0; // write (max torque (max current) = 1000)
  int32_t  target_position_           = 0; // write
  int32_t  target_velocity_           = 0; // write
  int16_t  torque_offset_             = 0;
  int32_t  tunning_command_           = 0;
  bool     digital_output_[4]          = {0,0,0,0};
  // bool     digital_output_2_          = 0;
  // bool     digital_output_3_          = 0;
  // bool     digital_output_4_          = 0;
  int32_t velocity_offset_ = 0;
  int32_t  user_mosi_                 = 0;


  uint16_t status_word_               = 0; // read
  int8_t   mode_of_operation_display_ = 0; // read
  int32_t  position_                  = 0; // read
  int32_t  velocity_                  = 0; // read
  int16_t  torque_                    = 0; // read
  int32_t  second_position_           = 0;
  int32_t  second_velocity_           = 0;
  uint16_t  analog_input_1_            = 0;
  uint16_t  analog_input_2_            = 0;
  uint16_t  analog_input_3_            = 0;
  uint16_t  analog_input_4_            = 0;
  int32_t  tunning_status_            = 0;
  bool     digital_input_[4]           = {0,0,0,0};
  uint8_t  resolution_data[100];

  // bool     digital_input_2_           = 0;
  // bool     digital_input_3_           = 0;
  // bool     digital_input_4_           = 0;
  bool     user_miso_                 = 0;
  uint32_t time_stamp_                = 0;
  int32_t position_demand_internal_value = 0;
  int32_t velocity_demand_value = 0;
  int32_t torque_demand = 0;
  int16_t erro_code=0;
  uint32_t dc_linkVoltage = 0;
  uint8_t brake_status = 1;
  enum DeviceState{
    STATE_UNDEFINED = 0,
    STATE_START = 1,
    STATE_NOT_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON_DISABLED,
    STATE_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON,
    STATE_OPERATION_ENABLED,
    STATE_QUICK_STOP_ACTIVE,
    STATE_FAULT_REACTION_ACTIVE,
    STATE_FAULT,
   // STATE_WARN
  };

  std::map<DeviceState,std::string> device_state_str_ = {
    {STATE_START,                  "Start"},
    {STATE_NOT_READY_TO_SWITCH_ON, "Not Ready to Switch On"},
    {STATE_SWITCH_ON_DISABLED,     "Switch on Disabled"},
    {STATE_READY_TO_SWITCH_ON,     "Ready to Switch On"},
    {STATE_SWITCH_ON,              "Switch On"},
    {STATE_OPERATION_ENABLED,      "Operation Enabled"},
    {STATE_QUICK_STOP_ACTIVE,      "Quick Stop Active"},
    {STATE_FAULT_REACTION_ACTIVE,  "Fault Reaction Active"},
    {STATE_FAULT,                  "Fault"},
    {STATE_UNDEFINED,               "STATE_UNDEFINED"}
    //{STATE_WARN,                   "Warning"}
  };

 DeviceState state_ = STATE_START;
/*!
* \brief errorlog() : it is a structure to get the error logs.
*/
  struct errorlog{
   uint16_t  cwRead_;
   uint16_t control_word_ ;
   uint16_t status_word_;
   DeviceState state_;

  };
/*!
* \brief Logging() : it is a structure to store the data of error logs.
*/
  struct Logging{
     errorlog data;
  };
  errorlog prev_log,log;

  bool enable_motor = false;
  enum ModeOfOperation
  {
    MODE_NO_MODE                = 0,
    MODE_PROFILED_POSITION      = 1,
    MODE_PROFILED_VELOCITY      = 3,
    MODE_PROFILED_TORQUE        = 4,
    MODE_HOMING                 = 6,
    MODE_INTERPOLATED_POSITION  = 7,
    MODE_CYCLIC_SYNC_POSITION   = 8,
    MODE_CYCLIC_SYNC_VELEOCITY  = 9,
    MODE_CYCLIC_SYNC_TORQUE     = 10
  };

  ec_sdo_request_t *request_[3];


/*!
* \brief channels_() : Here we are do the pdo entry.
*/
// private:
  ec_pdo_entry_info_t channels_[32] = {



    {0x6040, 0x00, 16}, /* Controlword */
    {0x6060, 0x00, 8}, /* Op Mode */
    {0x6071, 0x00, 16}, /* Target Torque */
    {0x607a, 0x00, 32}, /* Target Position */
    {0x60ff, 0x00, 32}, /* Target Velocity */
    {0x60b2, 0x00, 16}, /* Torque offset */
    {0x2701, 0x00, 32}, /* Tuning command */
    {0x2703, 0x00, 32}, /* User MOSI */
    {0x60b1, 0x00, 32}, /* Velocity offset */

    // {0x2601, 0x00, 1}, /* Digital Output 1 */
    // {0x2007, 0x00, 7},
    // {0x2602, 0x00, 1}, /* Digital Output 2 */
    // {0x2007, 0x00, 7},
    // {0x2603, 0x00, 1}, /* Digital Output 3 */
    // {0x2007, 0x00, 7},
    // {0x2604, 0x00, 1}, /* Digital Output 4 */
    // {0x2007, 0x00, 7},



    {0x6041, 0x00, 16}, /* Statusword */
    {0x2038, 0x01, 32}, /* Op Mode Display */
    {0x6064, 0x00, 32}, /* Position Value */
    {0x606c, 0x00, 32}, /* Velocity Value */
    {0x6077, 0x00, 16}, /* Torque Value */
    {0x230a, 0x00, 32}, /* Secondary position value */
    {0x230b, 0x00, 32}, /* Secondary velocity value */
    {0x2702, 0x00, 32}, /* Tuning status */
    {0x6079, 0x00, 32}, /*DC link circuit voltage*/
//    {0x2004, 0x07, 8},
    //
    // {0x2501, 0x00, 1}, /*Digital Input 1 */
    // {0x2007, 0x00, 7},
    // {0x2502, 0x00, 1}, /* Digital Input 2 */
    // {0x2007, 0x00, 7},
    // {0x2503, 0x00, 1}, /* Digital Input 3 */
    // {0x2007, 0x00, 7},
    // {0x2504, 0x00, 1}, /* Digital Input 4 */
    // {0x2007, 0x00, 7},

    {0x2704, 0x00, 32}, /* User MISO */
    {0x20f0, 0x00, 32}, /* Timestamp */
    {0x60fc, 0x00, 32}, /* Position demand internal value */
    {0x606b, 0x00, 32}, /* Velocity demand value */
    {0x6074, 0x00, 16}, /* Torque demand */
    {0x20e1, 0x01,200}, /*High resolution data*/
    {0x20e1, 0x02,200}, /*High resolution data*/
    {0x20e1, 0x03,200}, /*High resolution data*/
    {0x20e1, 0x04,200}, /*High resolution data*/
    {0x603F, 0x00, 16}, /*Error code of Drive*/

    {0x2401, 0x00, 16}, /* Analog input 1 */
    {0x2402, 0x00, 16}, /* Analog input 2 */
    {0x2403, 0x00, 16}, /* Analog input 3 */
    {0x2404, 0x00, 16}, /* Analog input 4 */

  };
/*!
* \brief pdos_ : it will tell how much pdo register for which sync manager.
*/

  ec_pdo_info_t pdos_[4] = {
    {0x1600, 9, channels_ + 0},
    {0x1a00, 9, channels_ + 9},
    {0x1a01, 10, channels_ + 18},
    {0x1a02, 4, channels_ + 28},
  };


/*!
* \brief slave_0_syncs : it will tell how many input and output sync manager there in domain.
*/

  ec_sync_info_t syncs_[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 3, pdos_ + 1, EC_WD_DISABLE},
    {0xff}
  };



  DomainMap domains_ = {
    {0,{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}} };



  /** returns device state based upon the status_word **/
  DeviceState deviceState(uint16_t status_word)
  {
    if      ((status_word & 0b01001111) == 0b00000000){
      return STATE_NOT_READY_TO_SWITCH_ON;
    }
    else if ((status_word & 0b01001111) == 0b01000000){
      return STATE_SWITCH_ON_DISABLED;
    }
    else if ((status_word & 0b01101111) == 0b00100001){
      return STATE_READY_TO_SWITCH_ON;
    }
    else if ((status_word & 0b01101111) == 0b00100011){
      return STATE_SWITCH_ON;
    }
    else if ((status_word & 0b01101111) == 0b00100111){
      return STATE_OPERATION_ENABLED;
    }
    else if ((status_word & 0b01101111) == 0b00000111){
      return STATE_QUICK_STOP_ACTIVE;
    }
    else if ((status_word & 0b01001111) == 0b00001111){
      return STATE_FAULT_REACTION_ACTIVE;
    }
    else if ((status_word & 0b01001111) == 0b00001000){
      return STATE_FAULT;
    }
    // else if ((status_word & 0b10000000) == 0b10000000){
    //   return STATE_WARN;
    // }

    return STATE_UNDEFINED;
  }

  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word)
  {
    switch(state)
    {
    case STATE_START:                   // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:      // -> STATE_READY_TO_SWITCH_ON
      return ((control_word & 0b01111110) | 0b00000110);
    case STATE_READY_TO_SWITCH_ON:      // -> STATE_SWITCH_ON
      return ((control_word & 0b01110111) | 0b00000111);
    case STATE_SWITCH_ON:               // -> STATE_OPERATION_ENABLED
      return ((control_word & 0b01111111) | 0b00001111);
    case STATE_OPERATION_ENABLED:       // -> GOOD
      return control_word;
    case STATE_QUICK_STOP_ACTIVE:       // -> STATE_OPERATION_ENABLED
      return ((control_word & 0b01111111) | 0b00001111);
    case STATE_FAULT_REACTION_ACTIVE:   // -> STATE_FAULT (automatic)
      return control_word;
    case STATE_FAULT:                   // -> STATE_SWITCH_ON_DISABLED
      return ((control_word & 0b11111111) | 0b1000000);
    default:
      break;
    }
    return control_word;
  }

  uint16_t disable(DeviceState state, uint16_t control_word)
  {
    switch(state)
    {
    case STATE_START:                   // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:      // -> STATE_READY_TO_SWITCH_ON
      return ((control_word & 0b01111110) | 0b00000110);
    case STATE_READY_TO_SWITCH_ON:      // -> STATE_SWITCH_ON
      return ((control_word & 0b01110111) | 0b00000111);
    case STATE_SWITCH_ON:               // -> STATE_OPERATION_ENABLED
      return ((control_word & 0b00001111) | 0b00001111);
    case STATE_OPERATION_ENABLED:       // -> GOOD
      return control_word;
      /*case STATE_QUICK_STOP_ACTIVE:       // -> STATE_OPERATION_ENABLED
              return ((control_word & 0b00001111) | 0b00001111);
          case STATE_FAULT_REACTION_ACTIVE:   // -> STATE_FAULT (automatic)
              return control_word;
          case STATE_FAULT:                   // -> STATE_SWITCH_ON_DISABLED
              return ((control_word & 0b00001111) | 0b10000000); */
    default:
      break;
    }
    return control_word;
  }

  uint16_t set_control(DeviceState state, uint16_t control_word)
  {
    switch(state)
    {
    case STATE_START:                   // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:      // -> STATE_READY_TO_SWITCH_ON
      return ((control_word & 0b01111110) | 0b00000110);
    case STATE_READY_TO_SWITCH_ON:      // -> STATE_SWITCH_ON
      return ((control_word & 0b01110111) | 0b00000111);
    case STATE_SWITCH_ON:               // -> STATE_OPERATION_ENABLED
      return ((control_word & 0b01111111) | 0b00001111);
    case STATE_OPERATION_ENABLED:       // -> GOOD
      return control_word;
    case STATE_QUICK_STOP_ACTIVE:
     if(last_state_==STATE_OPERATION_ENABLED)
      return ((control_word & 0b11111111) | 0b10000000);  // -> STATE_SWITCH_ON_DISABLED
     else
      return ((control_word & 0b01111111) | 0b00001111); // -> STATE_OPERATION_ENABLED
    case STATE_FAULT_REACTION_ACTIVE:   // -> STATE_FAULT (automatic)
      return ((control_word & 0b01111111) | 0b00001000);
    case STATE_FAULT:                   // -> STATE_SWITCH_ON_DISABLED
      return ((control_word & 0b11111111) | 0b10000000);
    // case STATE_UNDEFINED:                   // -> STATE_SWITCH_ON_DISABLED
    //   return ((control_word & 0b11111111) | 0b10000000);
    default:
      break;
    }
    return control_word;
  }


  uint16_t set_control2(DeviceState state, uint16_t control_word)
  {
    switch(state)
    {
    case STATE_START:                   // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:      // -> STATE_READY_TO_SWITCH_ON
      return ((control_word & 0b01111110) | 0b00000110);
    case STATE_READY_TO_SWITCH_ON:      // -> STATE_SWITCH_ON
      return control_word;
             // case STATE_SWITCH_ON:               // -> STATE_OPERATION_ENABLED
             //     return ((control_word & 0b01111111) | 0b00001111);
             // case STATE_OPERATION_ENABLED:       // -> GOOD
             //     return control_word;
             // case STATE_QUICK_STOP_ACTIVE:       // -> STATE_OPERATION_ENABLED
             //     return ((control_word & 0b01111111) | 0b00001111);
             // case STATE_FAULT_REACTION_ACTIVE:   // -> STATE_FAULT (automatic)
             //     return control_word;
    case STATE_FAULT:                   // -> STATE_SWITCH_ON_DISABLED
      return ((control_word & 0b11111111) | 0b10000000);
    // case STATE_UNDEFINED:                   // -> STATE_SWITCH_ON_DISABLED
    //   return ((control_word & 0b11111111) | 0b10000000);

    default:

      break;
    }
    return control_word;
  }


  uint16_t set_start(DeviceState state, uint16_t control_word)
  {
    switch(state)
    {
    case STATE_START:                   // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:      // -> STATE_READY_TO_SWITCH_ON
      return ((control_word & 0b01111110) | 0b00000110);
    case STATE_READY_TO_SWITCH_ON:      // -> STATE_SWITCH_ON
      return ((control_word & 0b01110111) | 0b00000111);
    default:
      break;
    }
    return control_word;
  }

  int last_status_word_ = -1;
  DeviceState last_state_ = STATE_START;

  int cnt = 0;

  bool initialized_ = false;

};

}

#endif
