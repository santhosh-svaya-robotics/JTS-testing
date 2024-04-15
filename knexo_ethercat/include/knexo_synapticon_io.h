/**
 * This file is part of SimpleECAT.
 *
 * SimpleECAT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimplECAT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with SimpleECAT.  If not, see <http://www.gnu.org/licenses/>.
 *
 * \class Elmo_GoldWhistle
 *
 * \ingroup SimplECAT
 *
 * \brief Elmo Gold Whistle
 *
 * Brushed or brushless DC servo drive.
 */

/*
Synapticon Vendor ID : 0x22d2
            product code : 0x0201
            revision number : 0xa000002
*/

/*
Synapticon
  { 0, 0, 0x22d2, 0x0201, 0x1600, 1, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1600, 2, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1600, 3, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1600, 4, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1600, 5, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1a00, 1, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1a00, 2, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1a00, 3, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1a00, 4, 0x0, 0x0  },
  { 0, 0, 0x22d2, 0x0201, 0x1a00, 5, 0x0, 0x0  },

*/

#ifndef SIMPLECAT_SYNAPTICON_DC100_H_
#define SIMPLECAT_SYNAPTICON_DC100_H_


#define SOMANET_ID 0x000022d2, 0x00000301
#define CAN_OD_CONTROL_WORD     0x6040      /* RX; 16 bit */
#define CAN_OD_STATUS_WORD      0x6041      /* TX; 16 bit */
#define CAN_OD_MODES            0x6060      /* RX; 8 bit */
#define CAN_OD_MODES_DISPLAY    0x6061      /* TX; 8 bit */
#define CAN_OD_TORQUE_TARGET    0x6071      /* RX; 16 bit */
#define CAN_OD_TORQUE_VALUE     0x6077      /* TX; 16 bit */
#define CAN_OD_POSITION_TARGET  0x607A      /* RX; 32 bit */
#define CAN_OD_POSITION_VALUE   0x6064      /* TX; 32 bit */
#define CAN_OD_VELOCITY_TARGET  0x60ff      /* RX; 32 bit */
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



#include <knexo_ethercat_slave.h>
#include <global_shared_variables.h>

/*!
* \brief knexo_ethercat : create a namespace.
*/
namespace knexo_ethercat {

/*!
* \brief Synapticon_IO : create a class name Synapticon_IO and it inherited with Slave class.
*/
class Synapticon_IO : public Slave
{

public:
/*!
  * \brief Synapticon_IO() : create a constructor .
  */ 
  Synapticon_IO() : Slave() {}
  /*!
  * \brief ~Synapticon_IO() : create a destructor .
  */ 
  virtual ~Synapticon_IO() {}

/*!
* \brief processData : it process the data and write in the domain.
*/
  virtual int processData(size_t index, uint8_t* domain_address){
    // DATA READ WRITE
    // EC_WRITE_S16(domain_address, color_mode);
/*!
* \brief EC_WRITE_U16 : it will write the data in the given pdo and is of 16 bit.
*/
    switch(index)
    {
    case 0:
      EC_WRITE_U16(domain_address, control_word_);
      break;
    case 1:
      EC_WRITE_S8(domain_address, mode_of_operation_);
      break;
    case 2:
      EC_WRITE_S16(domain_address, target_torque_);
      break;
    case 3:
      EC_WRITE_S32(domain_address, target_position_);
      break;
    case 4:
      EC_WRITE_S32(domain_address, target_velocity_);
      break;
    case 5:
      EC_WRITE_S16(domain_address,torque_offset_);
      break;
    case 6:
      EC_WRITE_S32(domain_address,tunning_command_);
      break;
    case 7:
      EC_WRITE_S16(domain_address,digital_output_1_);
      break;
    case 8:
      EC_WRITE_S16(domain_address,digital_output_2_);
      break;
    case 9:
      EC_WRITE_S16(domain_address,digital_output_3_);
      break;
    case 10:
      EC_WRITE_S16(domain_address,digital_output_4_);
      break;
    case 11:
      EC_WRITE_S32(domain_address, user_mosi_);
      break;
    case 12:
      status_word_ = EC_READ_U16(domain_address);
      break;
    case 13:
      mode_of_operation_display_ = EC_READ_S8(domain_address);
      break;
    case 14:
      position_ = EC_READ_S32(domain_address);
      break;
    case 15:
      velocity_ = EC_READ_S32(domain_address);
      break;
    case 16:
      torque_ = EC_READ_S16(domain_address);
      break;
    case 17:
      second_position_ = EC_READ_S32(domain_address);
      break;
    case 18:
      second_velocity_ = EC_READ_S32(domain_address);
      break;
    case 19:
      analog_input_1_ = EC_READ_S16(domain_address);
      break;
    case 20:
      analog_input_2_ = EC_READ_S16(domain_address);
      break;
    case 21:
      analog_input_3_ = EC_READ_S16(domain_address);
      break;
    case 22:
      analog_input_4_ = EC_READ_S16(domain_address);
      break;
    case 23:
      tunning_status_ = EC_READ_S32(domain_address);
      break;
    case 24:
      digital_input_1_ = EC_READ_S32(domain_address);
      break;
    case 25:
      digital_input_2_ = EC_READ_S32(domain_address);
      break;
    case 26:
      digital_input_3_ = EC_READ_S32(domain_address);
      break;
    case 27:
      digital_input_4_ = EC_READ_S32(domain_address);
      break;
    case 28:
      user_miso_ = EC_READ_S32(domain_address);
      break;
    case 29:
      time_stamp_ = EC_READ_S32(domain_address);
      break;

    default:
      std::cout << "WARNING. Synapticon_DC1K_D3 pdo index out of range." << std::endl;
    }
      return 1;
  }


/** Sync manager configuration information.
 *
 * This can be use to configure multiple sync managers including the PDO
 * assignment and PDO mapping. It is used as an input parameter type in
 * ecrt_slave_config_pdos().
 */
  virtual const ec_sync_info_t* syncs() { return &syncs_[0]; }


/*!
* \brief syncSize : it will give the size of sync manager.
*/
  virtual size_t syncSize() {
    return sizeof(syncs_)/sizeof(ec_sync_info_t);
  }


/** PDO entry configuration information.
 *
 * This is the data type of the \a entries field in ec_pdo_info_t.
 *
 * see ecrt_slave_config_pdos().
 */
  virtual const ec_pdo_entry_info_t* channels() {
    return channels_;
  }

  virtual void domains(DomainMap& domains) const {
    domains = domains_;
  }

  uint16_t control_word_              = 0; // write
  int8_t   mode_of_operation_         = 0; // write (use enum ModeOfOperation for convenience)
  int16_t  target_torque_             = 0; // write (max torque (max current) = 1000)
  int32_t  target_position_           = 0; // write
  int32_t  target_velocity_           = 0; // write
  int16_t  torque_offset_             = 0;
  int32_t  tunning_command_           = 0;
  bool     digital_output_1_          = 0;
  bool     digital_output_2_          = 0;
  bool     digital_output_3_          = 0;
  bool     digital_output_4_          = 0;
  int32_t  user_mosi_                 = 0;


  uint16_t status_word_               = 0; // read
  int8_t   mode_of_operation_display_ = 0; // read
  int32_t  position_                  = 0; // read
  int32_t  velocity_                  = 0; // read
  int16_t  torque_                    = 0; // read
  int32_t  second_position_           = 0;
  int32_t  second_velocity_           = 0;
  int16_t  analog_input_1_            = 0;
  int16_t  analog_input_2_            = 0;
  int16_t  analog_input_3_            = 0;
  int16_t  analog_input_4_            = 0;
  int32_t  tunning_status_            = 0;
  bool     digital_input_1_           = 0;
  bool     digital_input_2_           = 0;
  bool     digital_input_3_           = 0;
  bool     digital_input_4_           = 0;
  bool     user_miso_                 = 0;
  uint32_t     time_stamp_            = 0;


  uint32_t  color_mode           = 0; // write



private:
  ec_pdo_entry_info_t channels_[30] = {

    {CAN_OD_CONTROL_WORD, 0x00, 16},        /* Control word */
    {CAN_OD_MODES, 0x00, 8},                /* Mode of operation */
    {CAN_OD_TORQUE_TARGET, 0x00, 16},       /* Target torque */
    {CAN_OD_POSITION_TARGET, 0x00, 32},     /* Target position */
    {CAN_OD_VELOCITY_TARGET, 0x00, 32},     /* Target velocity */
    {0x60B2, 0x00, 16},                     /* Torque Offset */
    {0x2701, 0x00, 32},                     /* Tunning Command */
    {0x2601, 0x00, 1},                      /* Digital Output 1 */
    {0x2602, 0x00, 1},                      /* Digital Output 2 */
    {0x2603, 0x00, 1},                      /* Digital Output 3 */
    {0x2604, 0x00, 1},                      /* Digital Output 4 */
    {0x2703, 0x00, 32},                     /* User MOSI */

    {CAN_OD_STATUS_WORD, 0x00, 16},         /* Statusword */
    {CAN_OD_MODES_DISPLAY, 0x00, 8},        /* Mode of operation display */
    {CAN_OD_POSITION_VALUE, 0x00, 32},      /* Position actual value */
    {CAN_OD_VELOCITY_VALUE, 0x00, 32},      /* Velocity actual value */
    {CAN_OD_TORQUE_VALUE, 0x00, 16},        /* Torque actual value */
    {0x230A, 0x00, 32},                     /* Secondory Position actual value */
    {0x230B, 0x00, 32},                     /* Secondory Velocity actual value */
    {0x2401, 0x00, 16},                     /* Analog Input 1 value */
    {0x2402, 0x00, 16},                     /* Analog Input 2 value */
    {0x2403, 0x00, 16},                     /* Analog Input 3 value */
    {0x2404, 0x00, 16},                     /* Analog Input 4 value */
    {0x2702, 0x00, 32},                     /* Tuning Status value */
    {0x2501, 0x00, 1},                      /* Digital Input 1 value */
    {0x2502, 0x00, 1},                      /* Digital Input 2 value */
    {0x2503, 0x00, 1},                      /* Digital Input 3 value */
    {0x2504, 0x00, 1},                      /* Digital Input 4 value */
    {0x2704, 0x00, 32},                     /* User MISO */
    {0x20F0, 0x00, 32},                     /* Time Stamp value */
  };

  ec_pdo_info_t pdos_[2] = {
    {0x1600, 12, channels_ + 0},  /* RxPDO Mapping */
    {0x1a00, 18, channels_ + 12},  /* TxPD0 Mapping */
  };
/** Sync manager configuration information.
 *
 * This can be use to configure multiple sync managers including the PDO
 * assignment and PDO mapping. It is used as an input parameter type in
 * ecrt_slave_config_pdos().
 */
  ec_sync_info_t syncs_[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, pdos_ + 1, EC_WD_DISABLE},
    {0xff}
  };

  DomainMap domains_ = {
    {0, {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29}}
  };
};


}

#endif
