#ifndef SAMD51LAN9252_H_
#define SAMD51LAN9252_H_
#include <iostream>
#include <knexo_ethercat_slave.h>
#include <global_shared_variables.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <error.h>
#include <fcntl.h>
#include <unistd.h>    
using namespace std;


/*!
* \brief knexo_ethercat : create a namespace.
*/
namespace knexo_ethercat {

/*!
* \brief SAMD51Lan9252 : create a class name SAMD51Lan9252 and it inherited with Slave class.
*/
class SAMD51Lan9252 : public Slave
{
  /*!
  * \brief SAMD51Lan9252() : create a constructor which store the vendor and product id.
  */ 
public:
     //SAMD51Lan9252() : Slave(0x00000cc4, 0x00009252) {
   SAMD51Lan9252() : Slave() 
      {
            vendor_id_  =0x00000cc4;
//             vendor_id_  =0x000004d8;
            product_id_ = 0x00009252;

         unsigned int size_ADC_CNT = sizeof(int[7]);
       int fd_ADC_CNT = shm_open("ForceTorqueADC_CNT", O_CREAT | O_RDWR, 0666);
       ftruncate(fd_ADC_CNT, size_ADC_CNT);
       //ADC_input_value = static_cast<int*>(mmap(nullptr, size_ADC_CNT, PROT_READ | PROT_WRITE, MAP_SHARED, fd_ADC_CNT, 0));
           unsigned int size_force = sizeof(float[3]);
       int fd_force = shm_open("force", O_CREAT | O_RDWR, 0666);
       ftruncate(fd_force, size_force);
       force = static_cast<float*>(mmap(nullptr, size_force, PROT_READ | PROT_WRITE, MAP_SHARED, fd_force, 0));
           unsigned int size_torque = sizeof(float[3]);
       int fd_torque = shm_open("torque", O_CREAT | O_RDWR, 0666);
       ftruncate(fd_torque, size_torque);
      torque= static_cast<float*>(mmap(nullptr, size_torque, PROT_READ | PROT_WRITE, MAP_SHARED, fd_torque, 0));
      unsigned int size_slope = sizeof(float[6]);
       int fd_slope = shm_open("slope", O_CREAT | O_RDWR, 0666);
       ftruncate(fd_slope, size_slope);
     slope= static_cast<float*>(mmap(nullptr, size_slope, PROT_READ | PROT_WRITE, MAP_SHARED, fd_slope, 0));
      unsigned int size_intercept = sizeof(float[6]);
       int fd_intercept = shm_open("intercept", O_CREAT | O_RDWR, 0666);
       ftruncate(fd_intercept, size_intercept);
    intercept= static_cast<float*>(mmap(nullptr, size_intercept, PROT_READ | PROT_WRITE, MAP_SHARED, fd_intercept, 0));
   
         unsigned int size_calibration_flag = sizeof(float[8]);
       int fd_calibration_flag = shm_open("calibration_flag", O_CREAT | O_RDWR, 0666);
       ftruncate(fd_calibration_flag, size_calibration_flag);
        calibrationFlag=static_cast<float*>(mmap(nullptr, size_calibration_flag, PROT_READ | PROT_WRITE, MAP_SHARED, fd_calibration_flag, 0));
       
}
    virtual ~SAMD51Lan9252() {

     //digital_output_[0]=0;
    //digital_output_[1]=0;
  
    //digital_output_[2]=0;
     //digital_output_[3]= 0;
      //if(slave.digital_input_[0])
       
     //led_colour=0x000000;
}

struct Adc_config
{
      int Configpdo,Regpdo,RegValpdo;
      uint16_t      ADC_Config[5];
      uint16_t   ADC_Reg[4];
      uint16_t      ADC_Reg_Val[4];
};

          
        Adc_config Adc_config_arr[3];
        
        int LanDigitalIn_cnt = 6;
        int LanDigitalOut_cnt = 4;
        int SamDigitalIn_cnt = 6;
        int imu3In_cnt = 6;
        int SamDigitalOut_cnt = 2;
        int numAnalogIn = 7;
        int numAnalogOut = 4;
        int read_count_imu = 0;

       int slave_offline_cnt=0;
       int   Adc_modepdo=0x8009;
       int   slope_constpdo=0x800a;
       int   cal_flagpdo=0x800b;
       int   adc_configpdo=0x800E;
      std::vector<int>Lan_digital_input_pin= {4,0,1,2,3};//Count along with pin numbers
      std::vector<uint16_t>Lan_digital_input_value= {4,0,0,0,0};//Count along with pin state 

      std::vector<int>Sam_digital_input_pin= {4,0,1,2,3};//Count along with pin numbers
      std::vector<int>Sam_digital_input_value= {4,0,0,0,0,0,0};//Count along with pin state 

std::vector<int>imu3_input_value= {4,0,0,0,0,0,0};//Count along with pin state 

      std::vector<int>Lan_digital_output_pin= {4,0,1,2,3};//Count along with pin numbers (0,1 are digital outputs)
      std::vector<int>Lan_digital_output_value= {4,1,0,0,0};//Count along with pin state 


      std::vector<int>Sam_digital_output_pin= {2,8,9};//Count along with pin numbers(second pin is ADG input and Third pin is ADG)
      std::vector<int>Sam_digital_output_value= {2,0,1};//Count along with pin state 

      //int     digital_output_pin[4]          = {0,0,0,0}; // 0,1 are digital outputs,second index is ADG input and Third index is ADG Enable

      std::vector<int>DAC_output_pin={4,0,1,2,3};//Count along with pin numbers
      std::vector<int>DAC_output_value={4,45,65,0,0};
      //int      *ADC_input_value;
std::vector<int>ADC_input_value= {6,0,0,0,0,0,0};//Count along with pin state 

      std::vector<int>ADC_input_pin= {7,0,7,8,9,4,5,6}; // along with count {0,4,5,6,7,8,9} As Torque on ADS-2(SPI-2) and Force on ADS-1(SPI-3), AD4112 on SPI-1
      std::vector<int>ADC_input_Mode= {7,1,1,1,1,1,1,1};
      //int      DAC_out[4];
      int      led_cnt=15;
      int      led_colour=0xff0000;// 0xRGB format (00-ff each)
      float   *slope/*[6]={1.6,2.0,3.7,4.8,5.9,6.2}*/,*intercept;
     float     *calibrationFlag ;
      float   *force;
      float   *torque;
      float SDO[6];
      int     AD4112_ID,ADS1235_ID_1,ADS1235_ID_2;
      
    virtual int processData(size_t index, uint8_t* domain_address)
     {
       int sub_id,id;
       id=slave_0_pdo_entries[index].index;
       switch(id)
        {
      //    case 0x7000://LAN INPUT PINS
      //     sub_id=1;
          
      //     for(int i=0;i<LanDigitalIn_cnt+1;i++)
      //       {
             
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           if(Lan_digital_input_pin[0]>=(sub_id-1))
      //             {
      //               EC_WRITE_U8(domain_address,Lan_digital_input_pin[i]);  
      //               domain_address++;
      //             }
      //           else
      //              {
      //                  EC_WRITE_U8(domain_address,0);
      //                  domain_address++;
      //              } 
      //         }
      //         else
      //         {
      //           printf("PDO subindex mismatches %x \n",id);
      //           exit(1);
      //         }
              
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;
      //    case 0x7001://SAM INPUT PINS
      //     sub_id=1;
      //      for(int i=0;i<SamDigitalIn_cnt+1;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //             if(Sam_digital_input_pin[0]>=(sub_id-1))
      //             {
      //               EC_WRITE_U8(domain_address,Sam_digital_input_pin[i]);  
      //               domain_address++;
      //             }
      //             else
      //              {
      //                  EC_WRITE_U8(domain_address,0);
      //                   domain_address++;
      //              }
      //         } 
      //         else
      //         {
      //           printf("PDO subindex mismatches %x \n",id);
      //           exit(1);
      //         }
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;
      //   case 0x7002://ADC INPUT PINS
      //    sub_id=1;
      //     for(int i=0;i<numAnalogIn+1;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           if(ADC_input_pin[0]>=(sub_id-1))
      //           {
      //             EC_WRITE_U8(domain_address,ADC_input_pin[i]);  
      //             domain_address++;
      //           }
      //           else
      //            {
      //                EC_WRITE_U8(domain_address,0);
      //                 domain_address++;
      //            } 
      //         }
      //         else
      //         {
      //           printf("PDO subindex mismatches %x \n",id);
      //           exit(1);
      //         }
              
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;
        
      //   case 0x7003://LAN OUTPUT PINS
      //    sub_id=1;
      //     for(int i=0;i<LanDigitalOut_cnt+1;i++)
      //       {

      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           if(Lan_digital_output_pin[0]>=(sub_id-1))
      //           {
      //             EC_WRITE_U8(domain_address,Lan_digital_output_pin[i]);  
      //             domain_address++;
      //           }
      //           else
      //            {
      //                EC_WRITE_U8(domain_address,0);
      //                 domain__U8(domain_address,0);
      //                 domain_

              
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;
      //   case 0x7004://LAN OUTPUT PIN Values
      //    sub_id=1;
      //    for(int i=0;i<LanDigitalOut_cnt+1;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
          
      //           if(Lan_digital_output_value[0]>=(sub_id-1))
      //           {
      //             EC_WRITE_U8(domain_address,Lan_digital_output_value[i]);  
      //             domain_address++;
      //           }
      //           else
      //            {
      //             EC_WRITE_U8(domain_address,0); 
      //              domain_address++;
      //            } 
      //          }
      //         else
      //         {
      //           printf("PDO subindex mismatches %x \n",id);
      //           exit(1);
      //         }
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;
      //   case 0x7005://SAM OUTPUT PINS
      //     sub_id=1;
      //     for(int i=0;i<SamDigitalOut_cnt+1;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           if(Sam_digital_output_pin[0]>=(sub_id-1))
      //           {
      //            EC_WRITE_U8(domain_address,Sam_digital_output_pin[i]);  
      //            domain_address++;
      //           }
      //           else
      //            {
      //             EC_WRITE_U8(domain_address,0); 
      //              domain_address++;
      //            } 

      //         }
      //         else
      //         {
      //           printf("PDO subindex mismatches %x \n",id);
      //           exit(1);
      //         }
              
      //           index++;
      //           sub_id++;
      //       } 
      //       return sub_id-1;                      
      //      break;
      //   case 0x7006://SAM OUTPUT PIN Values
      //      sub_id=1;
      //     for(int i=0;i<SamDigitalOut_cnt+1;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           if(Sam_digital_output_pin[0]>=(sub_id-1))
      //           {
      //             EC_WRITE_U8(domain_address,Sam_digital_output_value[i]);  
      //            domain_address++;
      //           }
      //           else
      //            {
      //             EC_WRITE_U8(domain_address,0);
      //              domain_address++;
      //            }
      //         } 
      //         else
      //         {
      //           printf("PDO subindex mismatches %x \n",id);
      //           exit(1);
      //         }
      //           index++;
      //           sub_id++;
      //       } 
      //       return sub_id-1;                      
      //      break;
      //   case 0x7008://ADC OUTPUT PINS
      //    sub_id=1;
      //     for(int i=0;i<numAnalogOut+1;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           if(DAC_output_pin[0]>=(sub_id-1))
      //           {
      //             EC_WRITE_U8(domain_address,DAC_output_pin[i]);  
      //             domain_address++;
      //           }
      //           else
      //            {
      //                EC_WRITE_U8(domain_address,0);
      //                 domain_address++;
      //            }
      //         }
      //         else
      //         {
      //           printf("PDO subindex mismatches %x \n",id);
      //           exit(1);
      //         }
               
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;
      //  case 0x7009://ADC OUTPUT PIN Value
      //    sub_id=1;
      //     for(int i=0;i<numAnalogOut+1;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           if(DAC_output_pin[0]>=(sub_id-1))
      //           {
      //             EC_WRITE_U32(domain_address,DAC_output_value[i]);  
      //             domain_address+=4;
      //           }
      //           else
      //            {
      //             EC_WRITE_U32(domain_address,0); 
      //             domain_address+=4;
      //            } 
      //          }
      //         else
      //         {
      //           printf("PDO subindex mismatches %x \n",id);
      //           exit(1);
      //         }
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;
      //  case 0x8000:    
      //    sub_id=1;
      //     for(int i=0;i<5;i++)
      //       {            
      //          EC_WRITE_U16(domain_address,Adc_config_arr[0].ADC_Config[i]);  
      //          // cout<<"\n Adc_config_arr[0].ADC_Config[i]" << std::hex<< Adc_config_arr[0].ADC_Config[i]<<"    \n   ";
      //          domain_address+=2;
      //          sub_id++;
      //       }  
      //       return sub_id-1;  
      // break;
      // case 0x8001:
      //    sub_id=1;
      //     for(int i=0;i<4;i++)
      //       {            
      //          EC_WRITE_U16(domain_address,Adc_config_arr[0].ADC_Reg[i]);  
      //          domain_address+=2;
      //          sub_id++;
      //       } 
      //       return sub_id-1;                      
      // break;
      // case 0x8002:
      //    sub_id=1;
      //     for(int i=0;i<4;i++)
      //       {            
      //          EC_WRITE_U16(domain_address,Adc_config_arr[0].ADC_Reg_Val[i]);  
      //          domain_address+=2;
      //          sub_id++;
      //       } 
      //       return sub_id-1;                      
      // break;       
      //  case 0x8003:  
      //      sub_id=1;
      //       for(int i=0;i<5;i++)
      //         {            
      //            EC_WRITE_U16(domain_address,Adc_config_arr[1].ADC_Config[i]); 
      //            // cout<<"\n Adc_config_arr[0].ADC_Config[i]" << std::hex<< Adc_config_arr[1].ADC_Config[i]<<"    \n   "; 
      //            domain_address+=2;
      //            sub_id++;
      //         } 
      //         return sub_id-1;
          
      // break;
      // case 0x8004:
      //    sub_id=1;
      //     for(int i=0;i<4;i++)
      //       {            
      //          EC_WRITE_U16(domain_address,Adc_config_arr[1].ADC_Reg[i]);  
      //          domain_address+=2;
      //          sub_id++;
      //       } 
      //       return sub_id-1;                      
      // break;   
      // case 0x8005:
      //    sub_id=1;
      //     for(int i=0;i<4;i++)
      //       {            
      //          EC_WRITE_U16(domain_address,Adc_config_arr[1].ADC_Reg_Val[i]);  
      //          domain_address+=2;
      //          sub_id++;
      //       } 
      //       return sub_id-1;                      
      // break;           
      // case 0x8006:  
      //    sub_id=1;
      //     for(int i=0;i<5;i++)
      //       {            
      //          EC_WRITE_U16(domain_address,Adc_config_arr[2].ADC_Config[i]); 
      //          // cout<<"\n Adc_config_arr[0].ADC_Config[i]" << std::hex<<Adc_config_arr[2].ADC_Config[i]<<"    \n   "; 
      //          domain_address+=2;
      //          sub_id++;
      //       }  
      //       return sub_id-1;
      // break;
      // case 0x8007:
      //    sub_id=1;
      //     for(int i=0;i<4;i++)
      //       {            
      //          EC_WRITE_U16(domain_address,Adc_config_arr[2].ADC_Reg[i]);  
      //          domain_address+=2;
      //          sub_id++;
      //       } 
      //       return sub_id-1;                      
      // break;   
      // case 0x8008:
      //    sub_id=1;
      //     for(int i=0;i<4;i++)
      //       {            
      //          EC_WRITE_U16(domain_address,Adc_config_arr[2].ADC_Reg_Val[i]);  
      //          domain_address+=2;
      //          sub_id++;
      //       } 
      //       return sub_id-1;                      
      // break;         
      // case 0x8009://ADC OUTPUT PIN Mode
      //    sub_id=1;
      //     for(int i=0;i<ADC_input_Mode[0]+1;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           EC_WRITE_U8(domain_address,ADC_input_Mode[i]);  
      //           domain_address++;
      //         }
      //         else
      //          {
      //              printf("PDO subindex mismatches %x \n",id);
      //              exit(1);
      //          } 
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;
      // case 0x800A:
      //    sub_id=1;
      //     for(int i=0;i<6;i++)
      //       {            
      //          EC_WRITE_REAL(domain_address,slope[i]);  
      //          domain_address+=4;
      //          sub_id++;
      //       } 
      //     for(int i=0;i<6;i++)
      //       {            
      //          EC_WRITE_REAL(domain_address,intercept[i]);  
      //          domain_address+=4;
      //          sub_id++;
      //       } 
      //       return sub_id-1;                      
      // break; 
      // case 0x800B:
      //    sub_id=1;
      //     for(int i=0;i<7;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           EC_WRITE_REAL(domain_address,calibrationFlag[i]);  
      //           domain_address+=4;
      //         }
      //         else
      //          {
      //             printf("PDO subindex mismatches %x \n",id);
      //             exit(1);
      //          } 
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //      break;      
      // case 0x800C:   
               
      //     id=slave_0_pdo_entries[index].subindex;
      //     if(id==1)
      //      {
      //        AD4112_ID= EC_READ_U32(domain_address);

      //         return 1;
      //       }
      //      if(id==2)
      //      {
      //         ADS1235_ID_1=EC_READ_U32(domain_address);
      //          printf(" Samd ADS1235_ID_1 %x\n",ADS1235_ID_1);
      //         return 1;
      //       }
      //      if(id==3)
      //      {
      //         ADS1235_ID_2=EC_READ_U32(domain_address);
      //          printf(" Samd ADS1235_ID_2 %x\n",ADS1235_ID_2);
      //         return 1;
      //      }
          
      // break; 
      // case 0x800D:
      //     id=slave_0_pdo_entries[index].subindex;
      //     if(id ==1)
      //         {
      //           EC_WRITE_U32(domain_address,led_colour);
      //           return 1;
      //         }
            
      //     if(id ==2)
      //         {
      //            EC_WRITE_U32(domain_address,led_cnt);
      //            return 1;
      //         }
           
          
      //  break;  
      //  case 0x800E:
      //       sub_id=1;
      //     for(int i=0;i<7;i++)
      //       {
      //         if(slave_0_pdo_entries[index].subindex==sub_id)
      //         {
      //           EC_WRITE_REAL(domain_address,SDO[i]);  
      //           domain_address+=4;
      //         }
      //         else
      //          {
      //             printf("PDO subindex mismatches %x \n",id);
      //             exit(1);
      //          } 
      //           index++;
      //           sub_id++;
              
      //       }
      //      return sub_id-1;                     
      //  break;      
                    
       case 0x6000://LAN INPUT PIN Values
          sub_id=1;
          //if(read_count_imu % 300 == 0)
          //{read_count_imu = 1;
         for(int i=0;i<LanDigitalIn_cnt+1;i++)
            {
              if(slave_0_pdo_entries[index].subindex==sub_id)
              {

                    Lan_digital_input_value[i]=EC_READ_U8(domain_address);  
                    domain_address++;
                  
              }
              else
               {
                  printf("PDO subindex mismatches %x \n",id);
                  exit(1);
               } 
                index++;
                sub_id++;
              
            }
          //}
            //read_count_imu++;
           return sub_id-1;                     
        break;
       case 0x6001://SAM INPUT PIN Values
          sub_id=1;
          for(int i=0;i<SamDigitalIn_cnt+1;i++)
            {
              if(slave_0_pdo_entries[index].subindex==sub_id)
              {
                Sam_digital_input_value[i]=EC_READ_U8(domain_address);  
                domain_address++;
              }
              else
               {
                  printf("PDO subindex mismatches %x \n",id);
                  exit(1);
               } 
                index++;
                sub_id++;
              
            }
           return sub_id-1;                     
        break;
       case 0x6002:
          sub_id=1;
          for(int i=0;i<numAnalogIn+1;i++)
            {
              if(slave_0_pdo_entries[index].subindex==sub_id)
              {
                ADC_input_value[i]=EC_READ_U32(domain_address);  
                domain_address+=4;
              }
              else
               {
                  printf("PDO subindex mismatches %x \n",id);
                  exit(1);
               } 
                index++;
                sub_id++;
              
            }
          
           return sub_id-1;                     
        break;
      case 0x6003:
          sub_id=1;
          for(int i=0;i<imu3In_cnt+1;i++)
            {
              if(slave_0_pdo_entries[index].subindex==sub_id)
              {
                imu3_input_value[i]=EC_READ_U8(domain_address);  
                domain_address++;
              }
              else
               {
                  printf("PDO subindex mismatches %x \n",id);
                  exit(1);
               } 
                index++;
                sub_id++;
              
            }
           return sub_id-1;                     
           break; 
       default:
          std::cout<< "WARNING. SAMD51LAN9252 pdo index out of range." << index <<"   "<<id<<std::endl;
          //exit(1);

         }
     }
    


    virtual const ec_sync_info_t* syncs() { return &slave_0_syncs[0]+1; }

    virtual size_t syncSize() {
        return sizeof(slave_0_syncs)/sizeof(ec_sync_info_t);
    }

    virtual const ec_pdo_entry_info_t* channels() {
        return slave_0_pdo_entries;
    }

    virtual void domains(DomainMap& domains) const {
        domains = domains_;
    }







private:
     
    
// 
  ec_pdo_entry_info_t slave_0_pdo_entries[29] = {

    // {0x7000, 0x01, 8},   //0  //LAN INPUT Count
    // {0x7000, 0x02, 8},   //1  //LAN INPUT Pin-0
    // {0x7000, 0x03, 8},   //2  //LAN INPUT Pin-1
    // {0x7000, 0x04, 8},   //3  //LAN INPUT Pin-2
    // {0x7000, 0x05, 8},   //4  //LAN INPUT Pin-3
    // {0x7000, 0x06, 8},   //5  //LAN INPUT Pin-4
    // {0x7000, 0x07, 8},   //6  //LAN INPUT Pin-5
    
    // {0x7001, 0x01, 8},   //7  //SAM INPUT Count
    // {0x7001, 0x02, 8},   //8  //SAM INPUT Pin-0
    // {0x7001, 0x03, 8},   //9  //SAM INPUT Pin-1
    // {0x7001, 0x04, 8},   //10  //SAM INPUT Pin-2
    // {0x7001, 0x05, 8},   //11 //SAM INPUT Pin-3
    // {0x7001, 0x06, 8},   //12 //SAM INPUT Pin-2
    // {0x7001, 0x07, 8},   //13 //SAM INPUT Pin-3
    
    // {0x7003, 0x01, 8},   //14 //LAN OUTPUT Count
    // {0x7003, 0x02, 8},   //15 //LAN OUTPUT Pin-0
    // {0x7003, 0x03, 8},   //16 //LAN OUTPUT Pin-1
    // {0x7003, 0x04, 8},   //17 //LAN OUTPUT Pin-2
    // {0x7003, 0x05, 8},   //18 //LAN OUTPUT Pin-3
    
    // {0x7004, 0x01, 8},   //19 //LAN OUTPUT Count
    // {0x7004, 0x02, 8},   //20 //LAN OUTPUT Value-0
    // {0x7004, 0x03, 8},   //21 //LAN OUTPUT Value-1
    // {0x7004, 0x04, 8},     //22 //LAN OUTPUT Value-2
    // {0x7004, 0x05, 8},     //23 //LAN OUTPUT Value-3
    
    // {0x7005, 0x01, 8},   //24 //SAM OUTPUTS Count
    // {0x7005, 0x02, 8},     //25 //SAM OUTPUTS Pin-0
    // {0x7005, 0x03, 8},   //26 //SAM OUTPUTS Pin-1
    
    // {0x7006, 0x01, 8},   //27 //SAM OUTPUTS Count
    // {0x7006, 0x02, 8},   //28 //SAM OUTPUT Value-0
    // {0x7006, 0x03, 8},   //29 //SAM OUTPUT Value-1
    
    // //=========================================================
    // {0x7002, 0x01, 8},   //0  //ADC INPUTS Count
    // {0x7002, 0x02, 8},   //1  //ADC INPUT Pin-0
    // {0x7002, 0x03, 8},   //2  //ADC INPUT Pin-1
    // {0x7002, 0x04, 8},   //3  //ADC INPUT Pin-2
    // {0x7002, 0x05, 8},   //4  //ADC INPUT Pin-3
    // {0x7002, 0x06, 8},     //5  //ADC INPUT Pin-4
    // {0x7002, 0x07, 8},   //6  //ADC INPUT Pin-5
    // {0x7002, 0x08, 8},     //7  //ADC INPUT Pin-6
    
    // {0x7008, 0x01, 8},   //8  //DAC OUTPUTS Count
    // {0x7008, 0x02, 8},     //9  //DAC OUTPUTS Pin-0
    // {0x7008, 0x03, 8},     //10 //DAC OUTPUTS Pin-1
    // {0x7008, 0x04, 8},     //11 //DAC OUTPUTS Pin-2
    // {0x7008, 0x05, 8},     //12 //DAC OUTPUTS Pin-3
    
    // {0x7009, 0x01, 32},    //13 //DAC OUTPUTS Count
    // {0x7009, 0x02, 32},    //14 //DAC OUTPUTS Value-0
    // {0x7009, 0x03, 32},    //15 //DAC OUTPUTS Value-1
    // {0x7009, 0x04, 32},    //16 //DAC OUTPUTS Value-2
    // {0x7009, 0x05, 32},    //17 //DAC OUTPUTS Value-3
    
    // {0x8009, 0x01, 8},   //18 //ADC INPUTS Count
    // {0x8009, 0x02, 8},   //19 //ADC INPUTS Mode Configuration Pin-0
    // {0x8009, 0x03, 8},   //20 //ADC INPUTS Mode Configuration Pin-1
    // {0x8009, 0x04, 8},   //21 //ADC INPUTS Mode Configuration Pin-2
    // {0x8009, 0x05, 8},   //22 //ADC INPUTS Mode Configuration Pin-3
    // {0x8009, 0x06, 8},   //23 //ADC INPUTS Mode Configuration Pin-4
    // {0x8009, 0x07, 8},   //24 //ADC INPUTS Mode Configuration Pin-5
    // {0x8009, 0x08, 8},   //25 //ADC INPUTS Mode Configuration Pin-6

    // //=========================================================


    // {0x8000, 0x01, 16},   //0 //AD4112 Configuration
    // {0x8000, 0x02, 16},   //1 
    // {0x8000, 0x03, 16},   //2 
    // {0x8000, 0x04, 16},   //3 
    // {0x8000, 0x05, 16},   //4 
    
    // {0x8001, 0x01, 16},   //5 //AD4112 Register  Configuration
    // {0x8001, 0x02, 16},   //6 
    // {0x8001, 0x03, 16},   //7 
    // {0x8001, 0x04, 16},   //8 
    
    // {0x8002, 0x01, 16}, //9 //AD4112 Register Value  Configuration
    // {0x8002, 0x02, 16},   //10  
    // {0x8002, 0x03, 16},   //11  
    // {0x8002, 0x04, 16},   //12  
    
    // {0x8003, 0x01, 16},   //13  //ADS1235-1 Configuration
    // {0x8003, 0x02, 16},   //14  
    // {0x8003, 0x03, 16},   //15  
    // {0x8003, 0x04, 16},   //16  
    // {0x8003, 0x05, 16},   //17  
    
    // {0x8004, 0x01, 16},   //18  //ADS1235-1 Register  Configuration
    // {0x8004, 0x02, 16},   //19  
    // {0x8004, 0x03, 16},   //20  
    // {0x8004, 0x04, 16}, //21  
    
    // {0x8005, 0x01, 16}, //22  //ADS1235-1 Register Value  Configuration
    // {0x8005, 0x02, 16},   //23  
    // {0x8005, 0x03, 16},   //24  
    // {0x8005, 0x04, 16},   //25  
    
    // {0x8006, 0x01, 16}, //26  //ADS1235-2 Configuration
    // {0x8006, 0x02, 16},   //27  
    // {0x8006, 0x03, 16},   //28  
    // {0x8006, 0x04, 16},   //29  
    // {0x8006, 0x05, 16},   //30  
    
    // {0x8007, 0x01, 16}, //31  //ADS1235-2 Register  Configuration
    // {0x8007, 0x02, 16},   //32  
    // {0x8007, 0x03, 16},   //33  
    // {0x8007, 0x04, 16},   //34  
    
    // {0x8008, 0x01, 16}, //35  //ADS1235-2 Register Value  Configuration
    // {0x8008, 0x02, 16},   //36  
    // {0x8008, 0x03, 16},   //37  
    // {0x8008, 0x04, 16},   //38  

    // //========================================================= 
    
    // {0x800a, 0x01, 32},         //0 //Slope-1
    // {0x800a, 0x02, 32},   //1 //Slope-2
    // {0x800a, 0x03, 32},   //2 //Slope-3
    // {0x800a, 0x04, 32},   //3 //Slope-4
    // {0x800a, 0x05, 32},   //4 //Slope-5
    // {0x800a, 0x06, 32},   //5 //Slope-6
    // {0x800a, 0x07, 32}, //6 //Constant-1
    // {0x800a, 0x08, 32}, //7 //Constant-2
    // {0x800a, 0x09, 32}, //8 //Constant-3
    // {0x800a, 0x0a, 32}, //9 //Constant-4
    // {0x800a, 0x0b, 32}, //10  //Constant-5
    // {0x800a, 0x0c, 32}, //11  //Constant-6
    
    // {0x800b, 0x01,  32},  //12  //Calibration Flag (Offset)
    // {0x800b, 0x02,  32},  //13  //Calibration Flag-Fx
    // {0x800b, 0x03,  32},  //14  //Calibration Flag-Fy
    // {0x800b, 0x04,  32},  //15  //Calibration Flag-Fz
    // {0x800b, 0x05,  32},  //16  //Calibration Flag-Tx
    // {0x800b, 0x06,  32},  //17  //Calibration Flag-Ty
    // {0x800b, 0x07,  32},  //18  //Calibration Flag-Tz
    
    
    
    // {0x800d, 0x01, 32},   //19  //LED Colour
    // {0x800d, 0x02, 32},   //20  //LED Count
//0x800E SDO    
    //=========================================================
    
    
    {0x6000, 0x01, 8},  //0 //LAN INPUT Count
    {0x6000, 0x02, 8},  //1 //LAN INPUT Value-1
    {0x6000, 0x03, 8},  //2 //LAN INPUT Value-2
    {0x6000, 0x04, 8},  //3 //LAN INPUT Value-3
    {0x6000, 0x05, 8},  //4 //LAN INPUT Value-4
    {0x6000, 0x06, 8},  //5 //LAN INPUT Value-5
    {0x6000, 0x07, 8},  //6 //LAN INPUT Value-6
    
    {0x6001, 0x01, 8},  //7 //SAM INPUT Count
    {0x6001, 0x02, 8},  //8 //SAM INPUT Value-1
    {0x6001, 0x03, 8},    //9 //SAM INPUT Value-2
    {0x6001, 0x04, 8},  //10 //SAM INPUT Value-3
    {0x6001, 0x05, 8},    //11 //SAM INPUT Value-4
    {0x6001, 0x06, 8},  //12 //SAM INPUT Value-3
    {0x6001, 0x07, 8},    //13 //SAM INPUT Value-4
    
    {0x6002, 0x01, 32},   //14  //ADC Output Count
    {0x6002, 0x02, 32},   //15  //ADC Output Channel-1
    {0x6002, 0x03, 32},   //16  //ADC Output Channel-2
    {0x6002, 0x04, 32},   //17  //ADC Output Channel-3
    {0x6002, 0x05, 32},   //18  //ADC Output Channel-4
    {0x6002, 0x06, 32},   //19  //ADC Output Channel-5
    {0x6002, 0x07, 32},   //20  //ADC Output Channel-6
    {0x6002, 0x08, 32},   //21  //ADC Output Channel-7
    
    {0x6003, 0x01, 8},   //22 //Force-X
    {0x6003, 0x02, 8},   //23  //Force-Y
    {0x6003, 0x03, 8}, //24  //Force-Z
    {0x6003, 0x04, 8},   //25  //Torque-X
    {0x6003, 0x05, 8},   //26  //Torque-Y
    {0x6003, 0x06, 8},   //27  //Torque-Z
    {0x6003, 0x07, 8},   //28  //Torque-Z

    // {0x800c, 0x01, 32},     //0 //Ad4112 ID
    // {0x800c, 0x02, 32},     //1 //ADS1235-1 ID
    // {0x800c, 0x03, 32},     //2 //ADS1235-2 ID
};

ec_pdo_info_t slave_0_pdos[6] = {
    {0x1600, 29, slave_0_pdo_entries + 0} // 0x1600 has total 66 subindex for gpios
    // {0x1601, 26, slave_0_pdo_entries + 30}, //0x1601 has total 33 subindex for ADC
    // {0x1602, 39, slave_0_pdo_entries + 56}, //0x1602 has total 40 subindex for ADC Configuration
    // {0x1603, 21, slave_0_pdo_entries + 95}, //0x1603 has total 40 subindex 
    // {0x1a00, 29, slave_0_pdo_entries + 116}, //0x1A00 has total 25 subindex
    // {0x1a01, 3, slave_0_pdo_entries + 145}                                       //0x1A01 has total 25 subindex
                                            //0x1A02 has total 25 subindex
                                            //0x1A03 has total 25 subindex
};

ec_sync_info_t slave_0_syncs[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 0, NULL, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
    {0xff}

};



    DomainMap domains_ = {
        {0, {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28}}
    };
};
}

#endif
