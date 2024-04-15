
#ifndef SIMPLECAT_SLAVE_H_
#define SIMPLECAT_SLAVE_H_

#include <ecrt.h>
#include <map>
#include <vector>
//#include "ros/ros.h"
#include <rclcpp/rclcpp.hpp>
//---------------------Shared Memory and Semaphores headers---------------------
#include <semaphore.h>
#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <sys/shm.h>
#include <sys/mman.h>
#include <error.h>
#include<iostream>
#include<fstream>
//------------------------------------------------------------------------------
/*!
* \brief knexo_ethercat : create a namespace.
*/
namespace knexo_ethercat {

/*!
  * \brief Slave : create a class name Slave.
  */
class Slave
{
public:

  
  /*!
  * \brief Slave() : create a constructor which store the vendor and product id of slave .
  */ 
  Slave() :
    vendor_id_(0x000022d2),
    product_id_(0x00000301) {


  }

/*!
* \brief ~Slave() : create a Destructor .
*/
  virtual ~Slave() {
    //shm_unlink(name);
  }
   

  /** read or write data to the domain */
/*!
* \brief processData() : it process the data and write in domain.
*/
  virtual int processData(size_t index, uint8_t* domain_address){
    //read/write macro needs to match data type. e.g.
    //read_data_[index] = EC_READ_U8(domain_address)
    //EC_WRITE_S16(domain_address, write_data_[index]);
  }
/*!
* \brief init() : it help to incialise the slave.
*/
  virtual int32_t init(uint8_t* domain_address){

  }

  /** a pointer to syncs. return &syncs[0] */
/*!
* \brief syncs() : a pointer to syncs. return &syncs[0]
*/
  virtual const ec_sync_info_t* syncs() {
    //return &syncs_[0];
    return NULL;
  }

  /** number of elements in the syncs array. */
  virtual size_t syncSize() {
    //return sizeof(syncs_)/sizeof(ec_sync_info_t);
    return 0;
  }

  /** a pointer to all PDO entries */
  virtual const ec_pdo_entry_info_t* channels() {
    //return channels_;
    return NULL;
  }

  /** a map from domain index to pdo indices in that domain.
     *  map<domain index, vector<channels_ indices> > */
  typedef std::map<unsigned int, std::vector<unsigned int> > DomainMap;

  virtual void domains(DomainMap& domains) const {
    //domains = domains;
  }

  uint32_t vendor_id_;
  uint32_t product_id_;
  ec_sdo_request_t *request_[3];
 
  // //array to store the data to be read or sent
  //uint8_t               read_data_[E]; //example
  //int16_t               write_data_[F]; //example

protected:
  // //see SETUP_ETHERLAB.md for explanation
  //ec_pdo_entry_info_t   channels_[A];
  //ec_pdo_info_t         pdos_[B];
  //ec_sync_info_t        syncs_[C];
  //ec_pdo_entry_info_t   domain_regs_[D];
  //DomainMap domains_;



};

}

#endif



//
// #ifndef SIMPLECAT_SLAVE_H_
// #define SIMPLECAT_SLAVE_H_
//
// #include <ecrt.h>
// #include <map>
// #include <vector>
//
// //---------------------Shared Memory and Semaphores headers---------------------
// #include <semaphore.h>
// #include <fcntl.h>           /* For O_* constants */
// #include <sys/stat.h>        /* For mode constants */
// #include <sys/shm.h>
// #include <sys/mman.h>
// #include <error.h>
// //------------------------------------------------------------------------------
//
// namespace knexo_ethercat {
//
//
// class Slave
// {
// public:
//
//     Slave() :
//         vendor_id_(0x000022d2),
//         product_id_(0x00000201) {
//
//
//         }
//
//     virtual ~Slave() {
//       //shm_unlink(name);
//     }
//
//     /** read or write data to the domain */
//     virtual void processData(size_t index, uint8_t* domain_address){
//         //read/write macro needs to match data type. e.g.
//         //read_data_[index] = EC_READ_U8(domain_address)
//         //EC_WRITE_S16(domain_address, write_data_[index]);
//     }
//
//     virtual int32_t init(uint8_t* domain_address){
//
//      }
//
//     /** a pointer to syncs. return &syncs[0] */
//     virtual const ec_sync_info_t* syncs() {
//         //return &syncs_[0];
//         return NULL;
//     }
//
//     /** number of elements in the syncs array. */
//     virtual size_t syncSize() {
//         //return sizeof(syncs_)/sizeof(ec_sync_info_t);
//         return 0;
//     }
//
//     /** a pointer to all PDO entries */
//     virtual const ec_pdo_entry_info_t* channels() {
//         //return channels_;
//         return NULL;
//     }
//
//     /** a map from domain index to pdo indices in that domain.
//      *  map<domain index, vector<channels_ indices> > */
//     typedef std::map<unsigned int, std::vector<unsigned int> > DomainMap;
//
//     virtual void domains(DomainMap& domains) const {
//         //domains = domains;
//     }
//
//     uint32_t vendor_id_;
//     uint32_t product_id_;
//     ec_sdo_request_t *request_[3];
//
//     // //array to store the data to be read or sent
//     //uint8_t               read_data_[E]; //example
//     //int16_t               write_data_[F]; //example
//
// protected:
//     // //see SETUP_ETHERLAB.md for explanation
//     //ec_pdo_entry_info_t   channels_[A];
//     //ec_pdo_info_t         pdos_[B];
//     //ec_sync_info_t        syncs_[C];
//     //ec_pdo_entry_info_t   domain_regs_[D];
//     //DomainMap domains_;
//
//
//
// };
// 
// }
//
// #endif
