/************************************************************/



/**********************************************************/


#ifndef AIU_ATI_AXIA80_H_
#define AIU_ATI_AXIA80_H_


#include "knexo_ethercat_slave.h"
#include "global_shared_variables.h"

namespace knexo_ethercat {


class ft_sensor : public Slave
{

public:
  ft_sensor() : Slave()
  {
    vendor_id_  = 0x00000732;
    product_id_ = 0x26483053;
  }
  virtual ~ft_sensor() {}

  virtual int processData(size_t index, uint8_t* domain_address){
    // DATA READ WRITE
    // EC_WRITE_S16(domain_address, color_mode);

    switch(index)
    {
    case 0:

      if(clear_bias_op)
      {
//        std::cout<<"39\n";
        if((control_word1_ & 0b00000000000000000000000000000100) == 0b00000000000000000000000000000100)
        {
          control_word1_= control_word1_ & 0b11111111111111111111111111111011;
          clear_bias_op=false;
          //std::cout<<"done\n";
        }
        else
        {
          control_word1_=control_word1_ | 0b00000000000000000000000000000100;
          std::cout<<"Clearing Current Bias...";
        }
      }
      if(set_bias_op)
      {
//        std::cout<<"55\n";
        if(control_word1_ & 0b00000000000000000000000000000001 == 0b00000000000000000000000000000001)
        {
          control_word1_= control_word1_ & 0b11111111111111111111111111111110;
          set_bias_op=false;
          //std::cout<<"done\n";
        }
        else
        {
          control_word1_=control_word1_ | 0b00000000000000000000000000000001;
          //std::cout<<"Setting Bias against current load.....";
        }
      }
      else
      {
      control_word1_= control_word1_ & 0b11111111111111111111111111111110;
      }
      if(sample_rate > 0)
      {
        if(sample_rate == 1)
        {
          control_word1_ = (control_word1_ & 0b11111111111111110000111111111111) | 0b00000000000000000001000000000000;
        }
        else if(sample_rate == 2)
        {
//          std::cout<<"Control Word before "<<control_word1_<<"\n";
          control_word1_ = (control_word1_ & 0b11111111111111110000111111111111) | 0b00000000000000000010000000000000;
        }
        else if(sample_rate == 3)
        {
          control_word1_ = (control_word1_ & 0b11111111111111110000111111111111) | 0b00000000000000000011000000000000;
        }
      }
      if(low_pass_filter_fc > 0)
      {
        if(low_pass_filter_fc == 1)
        {
          control_word1_ = (control_word1_ & 0b11111111111111111111111100001111) | 0b00000000000000000000000000010000;
        }
        if(low_pass_filter_fc == 2)
        {
          control_word1_ = (control_word1_ & 0b11111111111111111111111100001111) | 0b00000000000000000000000000100000;
        }
        if(low_pass_filter_fc == 3)
        {
          control_word1_ = (control_word1_ & 0b11111111111111111111111100001111) | 0b00000000000000000000000000110000;
        }
        if(low_pass_filter_fc == 4)
        {
          control_word1_ = (control_word1_ & 0b11111111111111111111111100001111) | 0b00000000000000000000000001000000;
        }
        if(low_pass_filter_fc == 5)
        {
          control_word1_ = (control_word1_ & 0b11111111111111111111111100001111) | 0b00000000000000000000000001010000;
        }
        if(low_pass_filter_fc == 6)
        {
          control_word1_ = (control_word1_ & 0b11111111111111111111111100001111) | 0b00000000000000000000000001100000;
        }
        if(low_pass_filter_fc == 7)
        {
         control_word1_ = (control_word1_ & 0b11111111111111111111111100001111) | 0b00000000000000000000000001110000;
        }
        if(low_pass_filter_fc == 8)
        {
          control_word1_ = (control_word1_ & 0b11111111111111111111111100001111) | 0b00000000000000000000000010000000;
        }
      }

//      std::cout<<"Control word: "<<control_word1_<<"\n";
      EC_WRITE_S32(domain_address, control_word1_);
      break;
    case 1:
      EC_WRITE_S32(domain_address, control_word2_);
      break;
    case 2:
//      fx_ = (EC_READ_S32(domain_address))/1000000;
      fx_ = (EC_READ_S32(domain_address));

      break;
    case 3:
      fy_ = (EC_READ_S32(domain_address));
      break;
    case 4:
      fz_ = (EC_READ_S32(domain_address));
      break;
    case 5:
      tx_ = (EC_READ_S32(domain_address));
      break;
    case 6:
      ty_ = (EC_READ_S32(domain_address));
      break;
    case 7:
      tz_ = (EC_READ_S32(domain_address));
      break;
    case 8:
      status_word_ = EC_READ_S32(domain_address);
      break;
    case 9:
      sample_counter_ = EC_READ_S32(domain_address);
      break;
      // case 39:
      //     EC_WRITE_S32(domain_address, control_word1_);
      //     break;
      // case 40:
      //     EC_WRITE_S32(domain_address, control_word2_);
      //     break;
      // case 41:
      //     fx_ = EC_READ_S32(domain_address);
      //     break;
      // case 42:
      //     fy_ = EC_READ_S32(domain_address);
      //     break;
      // case 43:
      //     fz_ = EC_READ_S32(domain_address);
      //     break;
      // case 44:
      //     tx_ = EC_READ_S32(domain_address);
      //     break;
      // case 45:
      //     ty_ = EC_READ_S32(domain_address);
      //     break;
      // case 46:
      //     tz_ = EC_READ_S32(domain_address);
      //     break;
      // case 47:
      //     status_word_ = EC_READ_S32(domain_address);
      //     break;
      // case 48:
      //     sample_counter_ = EC_READ_S32(domain_address);
      //     break;
    default:
      std::cout << "WARNING. axia80 pdo index out of range." << std::endl;
    }
   return 1;
  }

  virtual const ec_sync_info_t* syncs() { return &syncs_[0]; }

  virtual size_t syncSize() {
    return sizeof(syncs_)/sizeof(ec_sync_info_t);
  }

  virtual const ec_pdo_entry_info_t* channels() {
    return channels_;
  }

  virtual void domains(DomainMap& domains) const {
    domains = domains_;
  }

  // if(status_word_ == )


  // bool initialized_ = false;


  int32_t fx_ ;
  int32_t fy_ ;
  int32_t fz_ ;
  int32_t tx_ ;
  int32_t ty_ ;
  int32_t tz_ ;
  int32_t status_word_ ;
  int32_t sample_counter_;


  int32_t control_word1_ ;
  int32_t control_word2_ ;

  bool clear_bias_op=false;
  bool set_bias_op=false;
  int sample_rate = 0;
  int low_pass_filter_fc = 0;
//  int



private:
  ec_pdo_entry_info_t channels_[10] =
  {

    /*Rx PDO*/
    {0x7010, 0x01, 32},        /* Control word1 */
    {0x7010, 0x02, 32},         /* Control word2*/

    /*Tx PDO*/

    {0x6000, 0x01, 32},         /* Force x */
    {0x6000, 0x02, 32},         /* Force y */
    {0x6000, 0x03, 32},         /* Force z */
    {0x6000, 0x04, 32},         /* Torque x */
    {0x6000, 0x05, 32},         /* Torque y */
    {0x6000, 0x06, 32},         /* Torque z */
    {0x6010, 0x00, 32},         /* Statusword */
    {0x6020, 0x00, 32},         /* Sample counter*/

  };

  ec_pdo_info_t pdos_[2] = {
    {0x1601, 2, channels_ + 0},  /* RxPDO Mapping */
    {0x1a00, 8, channels_ + 2}, /* TxPD0 Mapping */
  };

  ec_sync_info_t syncs_[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, pdos_ + 1, EC_WD_DISABLE},
    {0xff}
  };

  DomainMap domains_ = {
    {0, {0,1,2,3,4,5,6,7,8,9}}
  };
  // DomainMap domains_ = {
  //      {39, {39,40,41,42,43,44,45,46,47}}
  //  };
};


}

#endif
