#ifndef SIMPLECAT_MASTER_H_
#define SIMPLECAT_MASTER_H_

#include <ecrt.h>
#include <rclcpp/rclcpp.hpp>
//#include <ros/ros.h>


#include <time.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <knexo_ethercat_slave.h>
#include <knexo_synapticon_dc1k_d2.h>
#include <knexo_synapticon_io.h>
#include <aiu_ati_axia80.h>
#include <global_shared_variables.h>
#include <SAMD51Lan9252_ADC.h>


#define HALT_ENABLE 0x0100
#define DISABLE_OPERATION 0xfff7
#define Synapticon_DC1K_D3_CONF 0x000022d2, 0x00000301

namespace knexo_ethercat {

class Master
{
public:
  Master(int master = 0);
  virtual ~Master();
 
  /** \brief add a slave device to the master
      * alias and position can be found by running the following command
      * /opt/etherlab/bin$ sudo ./ethercat slaves
      * look for the "A B:C STATUS DEVICE" (e.g. B=alias, C=position)
      */
    void addSlave(uint16_t alias, uint16_t position, Synapticon_DC1K_D3* slave);
    void  slave_config( int position);
    void Ethercat_bus_reading();
    void addSlave(uint16_t alias, uint16_t position, Synapticon_IO* slave);
    void addSlave(uint16_t alias, uint16_t position, ft_sensor* slave);
     void addSlave(uint16_t alias, uint16_t position,  SAMD51Lan9252* slave);
    void sdo_request_read(unsigned int domain = 0);

  uint8_t* read_sdo(ec_sdo_request_t *sdo,int i);
  int write_sdo(ec_sdo_request_t *req, unsigned data);

  /** Configure distributed clocks.
     *
     * Sets the AssignActivate word and the cycle and shift times for the sync
     * signals.
     *
     * The AssignActivate word is vendor-specific and can be taken from the XML
     * device description file (Device -> Dc -> AssignActivate). Set this to zero,
     * if the slave shall be operated without distributed clocks (default).
     *
     * This method has to be called in non-realtime context before
     * ecrt_master_activate().
     *
     * \attention The \a sync1_shift time is ignored.
     */

  /** call after adding all slaves, and before update */
  void deactivate();
   void master_reset();
  void activate();
  bool sync_check();
  void slave_reconfig();
  /** perform one EtherCAT cycle, passing the domain to the slaves */
  virtual void update(bool *temp,bool *ready ,unsigned int domain = 0);

  // virtual void sdo_update(unsigned int domain = 0);

  /** run a control loop of update() and user_callback(), blocking.
     *  call activate and setThreadHighPriority/RealTime first. */
  typedef void (*SIMPLECAT_CONTRL_CALLBACK)(void);
  virtual void run(SIMPLECAT_CONTRL_CALLBACK user_callback, double frequency);

  /** stop the control loop. use within callback, or from a seperate thread. */
  virtual void stop() {running_ = false;}

  /** time of last ethercat update, since calling run. stops if stop called.
     *  returns actual time. use elapsedCycles()/frequency for discrete time at last update. */
  virtual double elapsedTime();

  /** number of EtherCAT updates since calling run. */
  virtual unsigned long long elapsedCycles();

  /** add ctr-c exit callback.
      * default exits the run loop and prints timing */
  typedef void (*SIMPLECAT_EXIT_CALLBACK)(int);
  static void setCtrlCHandler(SIMPLECAT_EXIT_CALLBACK user_callback = NULL);

  /** set the thread to a priority of -19
     *  priority range is -20 (highest) to 19 (lowest) */
  static void setThreadHighPriority();

  /** set the thread to real time (FIFO)
     *  thread cannot be preempted.
     *  set priority as 49 (kernel and interrupts are 50) */
  static void setThreadRealTime();

 double drive_temperature_[7]={-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};
  int drive_erro_code_[7]={0,};
  double ft_values[6];

  /** check for change in the master state */
  //  void checkMasterState();

  /** check for change in the slave states */
  //    void checkSlaveStates();
  ec_master_t *master_ = NULL;
  std::map< int,std::vector<Slave*>>slave_obj_map ;
  std::map< std::string,int>slave_name_map;
  ec_master_info_t master_info;
  bool slave_pre_op_state;
  void checkMasterBusScaning();
  /** check for change in the domain state */
  void checkDomainState(unsigned int domain);

  /** check for change in the master state */
  void checkMasterState();

  /** check for change in the slave states */
  bool checkSlaveStates();


  ec_master_state_t master_state_ = {};
   
  /** data for a single domain */
  struct DomainInfo
  {
    DomainInfo(ec_master_t* master);
    ~DomainInfo();

    ec_domain_t *domain = NULL;
    ec_domain_state_t domain_state = {};
    uint8_t *domain_pd = NULL;

    /** domain pdo registration array.
         *  do not modify after active(), or may invalidate */
    std::vector<ec_pdo_entry_reg_t> domain_regs;

    /** slave's pdo entries in the domain */
    struct Entry {
      Slave* slave               = NULL;
      int num_pdos               = 0;
      unsigned int* offset       = NULL;
      unsigned int* bit_position = NULL;
    };

    std::vector<Entry> entries;
  };

  /** map from domain index to domain info */
  std::map<unsigned int, DomainInfo*> domain_info_;
  bool sync_flag;
  struct timespec receive_t, send_t;
  struct timespec prev_receive_t, prev_send_t;

  int position ;

  typedef enum master_Position{
    left_side = 0;
    right_side = 1;

  };


private:



  /** true if running */
  volatile bool running_ = false;

  /** start and current time */
  std::chrono::time_point<std::chrono::system_clock> start_t_, curr_t_;

  // EtherCAT Control

  /** register a domain of the slave */
  // struct DomainInfo;
  void registerPDOInDomain(uint16_t alias, uint16_t position,
                           std::vector<unsigned int>& channel_indices,
                           DomainInfo* domain_info,
                           Slave* slave);

  
  /** print warning message to terminal */
  static void printWarning(const std::string& message);

  /** EtherCAT master data */

  


  /** data needed to check slave state */
  // struct SlaveInfo {
  //     Slave*     slave               = NULL;
  //     ec_slave_config_t*      config              = NULL;
  //     ec_slave_config_state_t config_state        = {0};
  // };

  struct SlaveInfo {
    Synapticon_DC1K_D3*     slave_joint            = NULL;
    Synapticon_IO*          slave_tool            = NULL;
    SAMD51Lan9252*          left_sigConditionBoard   = NULL;
    SAMD51Lan9252*          right_sigConditionBoard   = NULL;
    ft_sensor*              slave_sensor          = NULL;
    std::string             slave_type          ="";
    ec_slave_config_t*      config              = NULL;
    ec_slave_config_state_t config_state        = {0};
  };

  std::vector<SlaveInfo> slave_info_;

  /** counter of control loops */
  unsigned long long update_counter_ = 0;

  /** frequency to check for master or slave state change.
     *  state checked every frequency_ control loops */
  unsigned int check_state_frequency_ = 100;

  int pos_home = 0;


  bool temp_wait=false;
  bool temp_ready=false;
  int temp_count=0;
  uint8_t *result[8];
  uint8_t *sdo_errorcode[8];
  uint8_t *ft_result[6];
  bool success[8]={false,false,false,false,false,false,false,false};
  bool ft_received[6] = {false, false, false, false, false, false};
  ec_sdo_request_t *sdo_req[8];
  ec_sdo_request_t *sdo_err_req[8];
  ec_sdo_request_t *ft_sdo[6];
  //ec_slave_config_t *slave_config[8];


};

}

#endif







// #ifndef SIMPLECAT_MASTER_H_
// #define SIMPLECAT_MASTER_H_
//
// #include <ecrt.h>
// #include <ros/ros.h>
//
// #include <time.h>
// #include <string>
// #include <vector>
// #include <map>
// #include <chrono>
// #include <knexo_ethercat_slave.h>
// #include <knexo_synapticon_dc1k_d2.h>
// #include <knexo_synapticon_io.h>
// #include <aiu_ati_axia80.h>
//
//
// #define HALT_ENABLE 0x0100
// #define DISABLE_OPERATION 0xfff7
// #define Synapticon_DC1K_D3_CONF 0x000022d2, 0x00000201
//
// namespace knexo_ethercat {
//
// class Master
// {
// public:
//     Master(int master = 0);
//     virtual ~Master();
//
//     /** \brief add a slave device to the master
//       * alias and position can be found by running the following command
//       * /opt/etherlab/bin$ sudo ./ethercat slaves
//       * look for the "A B:C STATUS DEVICE" (e.g. B=alias, C=position)
//       */
//     void addSlave(uint16_t alias, uint16_t position, Synapticon_DC1K_D3* slave);
//     void addSlave(uint16_t alias, uint16_t position, Synapticon_IO* slave);
//     void addSlave(uint16_t alias, uint16_t position, ft_sensor* slave);
//
//     void sdo_request_read(unsigned int domain = 0);
//
//     uint8_t* read_sdo(ec_sdo_request_t *sdo,int i);
//     int write_sdo(ec_sdo_request_t *req, unsigned data);
//
//     /** Configure distributed clocks.
//      *
//      * Sets the AssignActivate word and the cycle and shift times for the sync
//      * signals.
//      *
//      * The AssignActivate word is vendor-specific and can be taken from the XML
//      * device description file (Device -> Dc -> AssignActivate). Set this to zero,
//      * if the slave shall be operated without distributed clocks (default).
//      *
//      * This method has to be called in non-realtime context before
//      * ecrt_master_activate().
//      *
//      * \attention The \a sync1_shift time is ignored.
//      */
//
//     /** call after adding all slaves, and before update */
//     void activate();
//
//     /** perform one EtherCAT cycle, passing the domain to the slaves */
//     virtual void update(bool *temp,bool *ready,unsigned int domain = 0);
//
//     virtual void sdo_update(unsigned int domain = 0);
//
//     /** run a control loop of update() and user_callback(), blocking.
//      *  call activate and setThreadHighPriority/RealTime first. */
//     typedef void (*SIMPLECAT_CONTRL_CALLBACK)(void);
//     virtual void run(SIMPLECAT_CONTRL_CALLBACK user_callback, double frequency);
//
//     /** stop the control loop. use within callback, or from a seperate thread. */
//     virtual void stop() {running_ = false;}
//
//     /** time of last ethercat update, since calling run. stops if stop called.
//      *  returns actual time. use elapsedCycles()/frequency for discrete time at last update. */
//     virtual double elapsedTime();
//
//     /** number of EtherCAT updates since calling run. */
//     virtual unsigned long long elapsedCycles();
//
//     /** add ctr-c exit callback.
//       * default exits the run loop and prints timing */
//     typedef void (*SIMPLECAT_EXIT_CALLBACK)(int);
//     static void setCtrlCHandler(SIMPLECAT_EXIT_CALLBACK user_callback = NULL);
//
//     /** set the thread to a priority of -19
//      *  priority range is -20 (highest) to 19 (lowest) */
//     static void setThreadHighPriority();
//
//     /** set the thread to real time (FIFO)
//      *  thread cannot be preempted.
//      *  set priority as 49 (kernel and interrupts are 50) */
//     static void setThreadRealTime();
//
//     double drive_temperature_[8]={-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};
//     double ft_values[6];
//
//     /** check for change in the master state */
//   //  void checkMasterState();
//
//     /** check for change in the slave states */
// //    void checkSlaveStates();
// ec_master_t *master_ = NULL;
// private:
//
//     /** true if running */
//     volatile bool running_ = false;
//
//     /** start and current time */
//     std::chrono::time_point<std::chrono::system_clock> start_t_, curr_t_;
//
//     // EtherCAT Control
//
//     /** register a domain of the slave */
//     struct DomainInfo;
//     void registerPDOInDomain(uint16_t alias, uint16_t position,
//                              std::vector<unsigned int>& channel_indices,
//                              DomainInfo* domain_info,
//                              Slave* slave);
//
//     /** check for change in the domain state */
//     void checkDomainState(unsigned int domain);
//
//     /** check for change in the master state */
//     void checkMasterState();
//
//     /** check for change in the slave states */
//     void checkSlaveStates();
//
//     /** print warning message to terminal */
//     static void printWarning(const std::string& message);
//
//     /** EtherCAT master data */
//
//     ec_master_state_t master_state_ = {};
//
//     /** data for a single domain */
//     struct DomainInfo
//     {
//         DomainInfo(ec_master_t* master);
//         ~DomainInfo();
//
//         ec_domain_t *domain = NULL;
//         ec_domain_state_t domain_state = {};
//         uint8_t *domain_pd = NULL;
//
//         /** domain pdo registration array.
//          *  do not modify after active(), or may invalidate */
//         std::vector<ec_pdo_entry_reg_t> domain_regs;
//
//         /** slave's pdo entries in the domain */
//         struct Entry {
//             Slave* slave               = NULL;
//             int num_pdos               = 0;
//             unsigned int* offset       = NULL;
//             unsigned int* bit_position = NULL;
//         };
//
//         std::vector<Entry> entries;
//     };
//
//     /** map from domain index to domain info */
//     std::map<unsigned int, DomainInfo*> domain_info_;
//
//     /** data needed to check slave state */
//     // struct SlaveInfo {
//     //     Slave*     slave               = NULL;
//     //     ec_slave_config_t*      config              = NULL;
//     //     ec_slave_config_state_t config_state        = {0};
//     // };
//
//     struct SlaveInfo {
//         Synapticon_DC1K_D3*     slave_joint            = NULL;
//         Synapticon_IO*          slave_tool            = NULL;
//         ft_sensor*              slave_sensor          = NULL;
//         std::string             slave_type          ="";
//         ec_slave_config_t*      config              = NULL;
//         ec_slave_config_state_t config_state        = {0};
//     };
//
//     std::vector<SlaveInfo> slave_info_;
//
//     /** counter of control loops */
//     unsigned long long update_counter_ = 0;
//
//     /** frequency to check for master or slave state change.
//      *  state checked every frequency_ control loops */
//     unsigned int check_state_frequency_ = 100;
//
//     int pos_home = 0;
//
//
//     bool temp_wait=false;
//     bool temp_ready=false;
//     int temp_count=0;
//     uint8_t *result[8];
//     uint8_t *ft_result[6];
//     bool success[8]={false,false,false,false,false,false,false,false};
//     bool ft_received[6] = {false, false, false, false, false, false};
//     ec_sdo_request_t *sdo_req[8];
//     ec_sdo_request_t *ft_sdo[6];
//     ec_slave_config_t *slave_config[8];
//
//
// };
//
// }
//
// #endif
