#include <knexo_ethercat_master.h>
// #include <knexo_ethercat_slave.h>
// #include <knexo_synapticon_dc1k_d2.h>

#include <unistd.h>
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <global_shared_variables.h>
using namespace std;
namespace knexo_ethercat {


Master::DomainInfo::DomainInfo(ec_master_t* master)
{
  domain = ecrt_master_create_domain(master);
  if (domain==NULL){
    printWarning("Failed to create domain");
    return;
  }
  else
    printf("Domain Creation Success\n");
  const ec_pdo_entry_reg_t empty = {0};
  domain_regs.push_back(empty);
}


Master::DomainInfo::~DomainInfo()
{
  for (Entry& entry : entries){
    delete [] entry.offset;
    delete [] entry.bit_position;
  }
}

struct slave_bus_info
{
 ec_slave_info_t infoofSlave;
 ec_sync_info_t *infoofSync;
 std::vector<ec_pdo_entry_info_t> pdo_entry_info;
  ec_slave_config_t*      config              = NULL;
  ec_slave_config_state_t config_state        = {0};
  int pdo_cnt=0,entry_cnt=0,position;
   
};

slave_bus_info *bus_info;


// unsigned int domain_index = 0;
void  Master::slave_config( int position)
{
    // configure slave in master
   int flag= ecrt_master_get_slave(master_,position,&bus_info[position].infoofSlave);
   if(flag==0)
   {
    printf("Success of getting %d slave info\n",position );
   }

    ec_slave_info_t *slave_info=&bus_info[position].infoofSlave;
     bus_info[slave_info->position].position=position;
     bus_info[slave_info->position].config=ecrt_master_slave_config(master_, slave_info->alias,slave_info->position,slave_info->vendor_id,slave_info->product_code);
    if (bus_info[slave_info->position].config==NULL){
      printWarning("Add slave. Failed to get slave configuration.");
      
    }

     // create a new entry in the domain
      DomainInfo::Entry domain_entry;
      switch(slave_name_map[slave_info->name])
      {
         case 1:
    printf("SOMANET Circulo CiA402 Drive\n");

          domain_entry.slave = new knexo_ethercat::Synapticon_DC1K_D3();
           slave_obj_map[slave_name_map[slave_info->name]].push_back(domain_entry.slave);
           printf("Drive Index  %d\n",slave_obj_map[slave_name_map[slave_info->name]].size()-1);
          break;
        case 2:
           printf("SAMD51 EtherCAT Slave\n");
          domain_entry.slave        = new knexo_ethercat::SAMD51Lan9252();
           slave_obj_map[slave_name_map[slave_info->name]].push_back(domain_entry.slave);
           
          break;
       
       case 3:
       printf("AXIA FT Sensor\n");
           domain_entry.slave        = new knexo_ethercat::ft_sensor();
           slave_obj_map[slave_name_map[slave_info->name]].push_back(domain_entry.slave);
       break;
       // case 2:
          //domain_entry.slave        = new knexo_ethercat::Synapticon_IO();
          //break;
          default :
            printf("Unknown Slave \n");
      }
     
     // slave_obj_map.insert(pair<std::string,Slave*>(slave_info->name,domain_entry.slave ));
    
    std::string slavename="SOMANET Circulo CiA402 Drive";
    if(slave_info->name==slavename)
    {
      printf("  Creating SDO requests for temperature....\n");
     if (!(sdo_req[slave_info->position] = ecrt_slave_config_create_sdo_request(bus_info[slave_info->position].config, 0x2031, 1, sizeof(int32_t))))
      {
        fprintf(stderr, "Failed to create SDO request for slave: %d",slave_info->position);
        // ecrt_release_master(master_);
        return;
      }
      ecrt_sdo_request_timeout(sdo_req[slave_info->position], 1000); // ms. Timeout in slave side???
    }
     // check if slave has pdos
    // size_t num_syncs =slave_info->sync_count ;
       size_t num_syncs = domain_entry.slave->syncSize() ;
    // const ec_sync_info_t* syncs = &(bus_info[position].infoofSync[0]);
       printf("syncSize %d\n",num_syncs);
     const ec_sync_info_t* syncs = domain_entry.slave->syncs();
    if (num_syncs>0)
    {
      // configure pdos in slave
      int pdos_status = ecrt_slave_config_pdos(bus_info[slave_info->position].config, EC_END, syncs);
      if (pdos_status){
        printWarning("Add slave. Failed to configure PDOs");
        //return;
      }
    } 
    else 
    {
      // TODO : uncomment
      printWarning("Add slave. Sync size is zero for " + std::to_string(slave_info->alias) + " : " + std::to_string(slave_info->position));
//     printWarning("Add slave. Sync size is zero for " + static_cast<std::ostringstream*>( &(std::ostringstream() << slave_info->alias) )->str() + ":" + static_cast<std::ostringstream*>( &(std::ostringstream() << slave_info->position) )->str());
      
    }


    // check if slave registered any pdos for the domain
         Slave::DomainMap domain_map;
         domain_entry.slave->domains(domain_map);
         
         for (auto& iter : domain_map)
         {

            // get the domain info, create if necessary
              //Domain Creation
             unsigned int domain_index = iter.first;//position;
              DomainInfo* domain_info = domain_info_[domain_index];
              if (domain_info==NULL)
              {
                printf("Domain Creation\n" );
                 domain_info = new DomainInfo(master_);
                 domain_info_[domain_index] = domain_info;
              }
             // expand the size of the domain

               // unsigned int num_pdo_regs = bus_info[position].entry_cnt;
                std::vector<unsigned int>&channel_indices=iter.second;

                unsigned int num_pdo_regs = channel_indices.size();
               size_t start_index = domain_info->domain_regs.size()-1; //empty element at end
               domain_info->domain_regs.resize(domain_info->domain_regs.size()+num_pdo_regs);

               domain_entry.num_pdos     = num_pdo_regs;
               domain_entry.offset       = new unsigned int[num_pdo_regs];
               domain_entry.bit_position = new unsigned int[num_pdo_regs];
               domain_info->entries.push_back(domain_entry);

               // // add to array of pdos registrations domain_entry.slave
               // const ec_pdo_entry_info_t* pdo_regs =&(bus_info[position].pdo_entry_info[0]); 
               const ec_pdo_entry_info_t* pdo_regs =domain_entry.slave->channels(); 
               for (size_t i=0; i<num_pdo_regs; ++i)
               {
                 // create pdo entry in the domain
                 ec_pdo_entry_reg_t& pdo_reg = domain_info->domain_regs[start_index+i];
                 pdo_reg.alias       = slave_info->alias;
                 pdo_reg.position    = slave_info->position;
                 pdo_reg.vendor_id   = slave_info->vendor_id;
                 pdo_reg.product_code= slave_info->product_code;
                 // pdo_reg.index       = pdo_regs[i].index;
                 // pdo_reg.subindex    = pdo_regs[i].subindex;
                 pdo_reg.index       = pdo_regs[channel_indices[i]].index;

                 pdo_reg.subindex    = pdo_regs[channel_indices[i]].subindex;
                 // printf(" pdo index%x    subindex  %d index %d\n",  pdo_reg.index ,pdo_reg.subindex,i);
                 pdo_reg.offset      = &(domain_entry.offset[i]);
                 pdo_reg.bit_position= &(domain_entry.bit_position[i]);
             }

              // set the last element to null
              ec_pdo_entry_reg_t empty = {0};
              domain_info->domain_regs.back() = empty;
          }
}

void Master::Ethercat_bus_reading()
{

//  std::cout<<"Checking Master state: ";
//  RCLCPP_INFO(rclcpp::get_logger("ethercat"),"Check master state");
  checkMasterState();
    std::cout<<"checked Master state: ";

      bus_info=new slave_bus_info[ master_state_.slaves_responding];
//  RCLCPP_INFO(rclcpp::get_logger("ethercat"),"bus info");
      printf("slave_count %d\n", master_state_.slaves_responding);
      for(int i=0;i< master_state_.slaves_responding;i++)
        {
       printf("Slave index %d Info\n",i );

       slave_config(i);
//       RCLCPP_INFO_STREAM(rclcpp::get_logger("ethercat"),"slave config, "<<i);
       usleep(1000);      
     }
//  RCLCPP_INFO(rclcpp::get_logger("ethercat"),"210");
      sync_flag=sync_check();
      if(!sync_flag)
      printf("Slave Configuration Failed\n");

}
Master::Master(const int master)
{
 
 master_index = master;
 slave_name_map["SOMANET Circulo CiA402 Drive"]=1;
 slave_name_map["SAMD51 EtherCAT Slave"]=2;
 slave_name_map["ATI Axia F/T Sensor"] = 3;
  master_ = ecrt_request_master(master);
  if (master_==NULL) {
    printWarning("Failed to obtain master.");
    return;
   }
 
}


Master::~Master()
{
  for (SlaveInfo& slave : slave_info_){
    //
  }
  for (auto& domain : domain_info_){
    delete domain.second;
  }
}
void Master::master_reset()
 {
  std::cout<<"Resetting\n";
  // ecrt_master_reset(master_);
   ecrt_release_master(master_);
   sleep(1);
   master_ = ecrt_request_master(master_index);
   if (master_==NULL)
   {
     printWarning("Failed to obtain master.");
     return;
    }
}
 void Master::deactivate()
 {
  printf("Deactivating Ethercat Master\n");
  ecrt_master_deactivate(master_);
  
 }
 
 void Master::checkMasterBusScaning()
 {
    int flag= ecrt_master(master_,&master_info);
    if(flag!=0)
    {
     printf("Getting master Information Failed\n");
    }

 }
void Master::activate()
{
  // register domain
 
  for (auto& iter : domain_info_){

    DomainInfo* domain_info = iter.second;
   std::cout<<"domain_info->domain_regs size"<<domain_info->domain_regs.size()<<"\n";
    int domain_status = ecrt_domain_reg_pdo_entry_list(
          domain_info->domain,
          &(domain_info->domain_regs[0]));
   
    if (domain_status){
      printWarning("Activate. Failed to register domain PDO entries.");
      return;
    }   

  }

  int flag= ecrt_master(master_,&master_info);
      if(flag!=0)
      {
       printf("Getting master Information Failed\n");
      }

      
      if(!master_info.scan_busy)
      std::cout<<"master_scanning\n";


  // activate master
  bool activate_status = ecrt_master_activate(master_);
 
  if (activate_status){
    printWarning("Activate. Failed to activate master.");
    return;
  }


printf("Domain Info Size %d \n",domain_info_.size());
  // retrieve domain data
  for (auto& iter : domain_info_){
    
    DomainInfo* domain_info = iter.second;
    domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
   
    if (domain_info->domain_pd==NULL){
      printWarning("Activate. Failed to retrieve domain process data.");
      return;
    }
  }
  
  std::cout<<std::dec<<"done\n";
  //----------------------------------------------------------------------

}
void Master::slave_reconfig()
{
  int i=0;
  for ( auto itr = slave_obj_map.begin(); itr != slave_obj_map.end(); ++itr)
      {
        for( auto obj =itr->second.begin(); obj!=itr->second.end()&& i< master_state_.slaves_responding;obj++)
        {
           // check if slave has pdos
           const ec_sync_info_t* actual_sync = (*obj)->syncs();
           ec_slave_info_t *slave_info=&bus_info[i].infoofSlave;
           //  bus_info[slave_info->position].config=ecrt_master_slave_config(master_, slave_info->alias,slave_info->position,slave_info->vendor_id,slave_info->product_code);
           // if (bus_info[slave_info->position].config==NULL){
           //   printWarning("Add slave. Failed to get slave configuration.");
             
           // }
           // size_t num_syncs =slave_info->sync_count ;
              size_t num_syncs = (*obj)->syncSize() ;
          std::cout<<"Slave name"<<slave_info->name<< "  "<<num_syncs<<"\n";
           if (num_syncs>0)
           {
             // configure pdos in slave
             int pdos_status = ecrt_slave_config_pdos(bus_info[i].config, EC_END, actual_sync);
             if (pdos_status){
               printWarning("Add slave. Failed to configure PDOs");
               //return;
             }
           } 
           else 
           {
             // TODO : uncomment
             printWarning("Add slave. Sync size is zero for " + std::to_string(slave_info->alias) + " : " + std::to_string(slave_info->position));
//            printWarning("Add slave. Sync size is zero for " + static_cast<std::ostringstream*>( &(std::ostringstream() << slave_info->alias) )->str() + ":" + static_cast<std::ostringstream*>( &(std::ostringstream() << slave_info->position) )->str());
             
           }
           i++;
           std::string slavename="SOMANET Circulo CiA402 Drive";
           if(slave_info->name==slavename)
           {
             printf("  Creating SDO requests for temperature....\n");
            if (!(sdo_req[slave_info->position] = ecrt_slave_config_create_sdo_request(bus_info[slave_info->position].config, 0x2031, 1, sizeof(int32_t))))
             {
               fprintf(stderr, "Failed to create SDO request for slave: %d",slave_info->position);
               // ecrt_release_master(master_);
               return;
             }
             ecrt_sdo_request_timeout(sdo_req[slave_info->position], 1000); // ms. Timeout in slave side???
           }
           // check if slave registered any pdos for the domain
               // create a new entry in the domain
                DomainInfo::Entry domain_entry;
                // domain_entry=(*obj)
                Slave::DomainMap domain_map;
                (*obj)->domains(domain_map);
                
                for (auto& iter : domain_map)
                {

                   // get the domain info, create if necessary
                     //Domain Creation
                    unsigned int domain_index = iter.first;//position;
                     DomainInfo* domain_info = domain_info_[domain_index];
                     if (domain_info==NULL)
                     {
                       printf("Domain Creation\n" );
                        domain_info = new DomainInfo(master_);
                        domain_info_[domain_index] = domain_info;
                     }
                    // expand the size of the domain

                      // unsigned int num_pdo_regs = bus_info[position].entry_cnt;
                       std::vector<unsigned int>&channel_indices=iter.second;

                       unsigned int num_pdo_regs = channel_indices.size();
                      size_t start_index = domain_info->domain_regs.size()-1; //empty element at end
                      domain_info->domain_regs.resize(domain_info->domain_regs.size()+num_pdo_regs);

                      domain_entry.num_pdos     = num_pdo_regs;
                      domain_entry.offset       = new unsigned int[num_pdo_regs];
                      domain_entry.bit_position = new unsigned int[num_pdo_regs];
                      domain_info->entries.push_back(domain_entry);

                      // // add to array of pdos registrations domain_entry.slave
                      // const ec_pdo_entry_info_t* pdo_regs =&(bus_info[position].pdo_entry_info[0]); 
                      const ec_pdo_entry_info_t* pdo_regs =domain_entry.slave->channels(); 
                      for (size_t i=0; i<num_pdo_regs; ++i)
                      {
                        // create pdo entry in the domain
                        ec_pdo_entry_reg_t& pdo_reg = domain_info->domain_regs[start_index+i];
                        pdo_reg.alias       = slave_info->alias;
                        pdo_reg.position    = slave_info->position;
                        pdo_reg.vendor_id   = slave_info->vendor_id;
                        pdo_reg.product_code= slave_info->product_code;
                        // pdo_reg.index       = pdo_regs[i].index;
                        // pdo_reg.subindex    = pdo_regs[i].subindex;
                        pdo_reg.index       = pdo_regs[channel_indices[i]].index;

                        pdo_reg.subindex    = pdo_regs[channel_indices[i]].subindex;
                        // printf(" pdo index%x    subindex  %d index %d\n",  pdo_reg.index ,pdo_reg.subindex,i);
                        pdo_reg.offset      = &(domain_entry.offset[i]);
                        pdo_reg.bit_position= &(domain_entry.bit_position[i]);
                    }

                     // set the last element to null
                     ec_pdo_entry_reg_t empty = {0};
                     domain_info->domain_regs.back() = empty;
                 }
        }
      }

}
bool Master::sync_check()
 {
    checkMasterState();
    if( master_state_.slaves_responding<=NUM_JOINTS )
    {
      // std::cout<<"all slaves are not responding "<<master_state_.slaves_responding<<"\n";
       return false;
    }
    bool slave_status =checkSlaveStates();
       if(!slave_status) //if Slaves are not configured then getting slave's sync manager info fails so reading data after slaves are in Operational state 
        {
          // std::cout<<"All slaves are not in operational\n";
          return slave_status;
        }
      // bus_info=new slave_bus_info[master_info.slave_count];
    printf("slave_count %d\n", master_state_.slaves_responding);
    int i=0;
    int cnt=0;
     map<int,std::vector<knexo_ethercat::Slave*>>::iterator itr;
     itr = slave_obj_map.begin();
     // for ( itr = slave_obj_map.begin(); itr != slave_obj_map.end(); ++itr)
     //  {
      // for(auto obj =slave_obj_map[2].begin(); obj!=slave_obj_map[2].end() && i< master_state_.slaves_responding;obj++)
      printf("slave_count %d\n", itr->second.size());
      for( auto obj =itr->second.begin(); obj!=itr->second.end()&& i< master_state_.slaves_responding;obj++)
      {
           cnt=0;
            printf("Slave index %d Info\n",i);
           const ec_sync_info_t* actual_sync = (*obj)->syncs();
           const ec_pdo_entry_info_t* actual_channels = (*obj)->channels();
           bus_info[i].pdo_cnt=0;bus_info[i].entry_cnt=0;
           ecrt_master_get_slave(master_,i,&bus_info[i].infoofSlave);

        // printf("sync_count %d\n",bus_info[i].infoofSlave.sync_count);
        
        bus_info[i].infoofSync= new  ec_sync_info_t[bus_info[i].infoofSlave.sync_count];
        
         
          for(int j=0;j<bus_info[i].infoofSlave.sync_count;j++)
          {

            
            ecrt_master_get_sync_manager(master_,i,j,&bus_info[i].infoofSync[j]);
            // printf("SM INDEX%x\n",bus_info[i].infoofSync[j].index);
            // printf("no of pdos %d\n",bus_info[i].infoofSync[j].n_pdos);
           
           // printf(" actual_sync->n_pdos %d\n",actual_sync[j].n_pdos);
            if(bus_info[i].infoofSync[j].n_pdos!= (actual_sync[j].n_pdos))
            {
              printf("Syncs not matched %d %d \n",bus_info[i].infoofSync[j].n_pdos,actual_sync[j].n_pdos);
              // int pdos_status = ecrt_slave_config_pdos(bus_info[bus_info[i].position].config, EC_END, actual_sync);
              //     if (pdos_status){
              //       printWarning("Add slave. Failed to configure PDOs");
                   
              //     }
                  return  false;
              // 
            }
            
            
                bus_info[i].infoofSync[j].pdos=new ec_pdo_info_t[ bus_info[i].infoofSync[j].n_pdos];
            for(int k=0;k<bus_info[i].infoofSync[j].n_pdos;k++)
              {

                ecrt_master_get_pdo(master_,i,bus_info[i].infoofSync[j].index,k,&bus_info[i].infoofSync[j].pdos[k]);
                // printf("PDO INDEX %x\n",bus_info[i].infoofSync[j].pdos[k].index);
                // printf("no of entries %d\n",bus_info[i].infoofSync[j].pdos[k].n_entries);
                bus_info[i].infoofSync[j].pdos[k].entries=new ec_pdo_entry_info_t[bus_info[i].infoofSync[j].pdos[k].n_entries]; 
               
                for(int z=0;z<bus_info[i].infoofSync[j].pdos[k].n_entries;z++)
                  {
                    ecrt_master_get_pdo_entry(master_,i,bus_info[i].infoofSync[j].index,k,z,&bus_info[i].infoofSync[j].pdos[k].entries[z]);  
                    bus_info[i].pdo_entry_info.push_back(bus_info[i].infoofSync[j].pdos[k].entries[z]);
                    // printf("PDO entries INDEX %x\n",bus_info[i].infoofSync[j].pdos[k].entries[z].index);
                    if(bus_info[i].infoofSync[j].pdos[k].entries[z].index!= actual_channels[cnt].index)
                    {
                      printf("Pdo  index not matched  for drive %d %x %x \n",i,bus_info[i].infoofSync[j].pdos[k].entries[z].index,actual_channels[cnt].index);
                      // int pdos_status = ecrt_slave_config_pdos(bus_info[bus_info[i].position].config, EC_END, actual_sync);
                      //     if (pdos_status){
                      //       printWarning("Add slave. Failed to configure PDOs");
                           
                      //     }
                           return  false;
                    }
                    
                     cnt++;
                    
                  }
                bus_info[i].entry_cnt+=bus_info[i].infoofSync[j].pdos[k].n_entries;
                
               }
         

          }
            i++;
        }
        return true;     
 }

uint8_t* Master::read_sdo(ec_sdo_request_t *sdo,int i)
{
  uint8_t* res=NULL;
  int state=ecrt_sdo_request_state(sdo);
  switch (state)
  {
  case EC_REQUEST_UNUSED: // request was not used yet
    ecrt_sdo_request_read(sdo); // trigger first read
    break;
  case EC_REQUEST_BUSY:
    // fprintf(stderr, "Still busy...\n");
    break;
  case EC_REQUEST_SUCCESS:
  {
    success[i]=true;
    res=ecrt_sdo_request_data(sdo);//Return pointer along with the bool *ok pointer
    // result=EC_READ_U4(addr);//This will change depending on the number of biys to read
    //            std::string s(res,res+50);//Read string from the pointer with the number of bytes to read
    //            received=true;
    ecrt_sdo_request_read(sdo); // trigger next read
    // do whatever processing you want.
  }
    break;
  case EC_REQUEST_ERROR:
    //            fprintf(stderr, "Failed to read SDO!\n");
    ecrt_sdo_request_read(sdo); // retry reading
    break;
  }

  return res;
}

void Master::checkDomainState(unsigned int domain)
{
  DomainInfo* domain_info = domain_info_[domain];
  if (domain_info!=NULL)
     {
          ec_domain_state_t ds;
          ecrt_domain_state(domain_info->domain, &ds);

          if (ds.working_counter != domain_info->domain_state.working_counter){
            //  printf("Domain: WC %u.\n", ds.working_counter);
          }
          if (ds.wc_state != domain_info->domain_state.wc_state){
            //  printf("Domain: State %u.\n", ds.wc_state);
          }
          domain_info->domain_state = ds;
     }
}

void Master::checkMasterState()
{
  ec_master_state_t ms;
  ecrt_master_state(master_, &ms);

  if (ms.slaves_responding != master_state_.slaves_responding){
    printf("%u slave(s).\n", ms.slaves_responding);
  }
  if (ms.al_states != master_state_.al_states){
    printf("Master AL states: 0x%02X.\n", ms.al_states);
  }
  if (ms.link_up != master_state_.link_up){
    printf("Link is %s.\n", ms.link_up ? "up" : "down");
  }
  master_state_ = ms;
}

bool Master::checkSlaveStates()
{
    int cnt=0;
    for(int i=0;i< master_state_.slaves_responding  && !Initialisation_Flag /*&& sync_flag*/;i++)
      //for (SlaveInfo& slave : slave_info_)
      {
        
        ec_slave_config_state_t s;
         ecrt_slave_config_state(bus_info[i].config, &s);
         if(s.al_state==0x8)
           slave_pre_op_state=true;
          if(s.al_state!=8)
            return false;
        // if (s.al_state != slave.config_state.al_state){
           if (s.al_state != bus_info[i].config_state.al_state)
           {
          //this spams the terminal at initialization.
          printf("Slave: State 0x%02X.\n", s.al_state);
        }
          if (s.online != bus_info[i].config_state.online)
          {
        // if (s.online != slave.config_state.online){
          printf("Slave: %s.\n", s.online ? "online" : "offline");
          if(i<NUM_JOINTS)
          {
            drive_conn_state.set(i,s.online);
          }
        }
        // if (s.operational != slave.config_state.operational){
          if (s.operational != bus_info[i].config_state.operational){
          printf("Slave: %soperational.\n", s.operational ? "" : "Not ");
        }
        // slave.config_state = s;
         bus_info[i].config_state = s;

      }
    
    return true;
}
void Master::update(bool *temp,bool *ready, unsigned int domain)
{

    clock_gettime(CLOCK_MONOTONIC,&receive_t);
      ecrt_master_receive(master_);
//std::cout<<"\nMaster:644\n";

      // check for master and slave state change
     bool slave_status = checkSlaveStates();
     if(!slave_status  /*&& slave_pre_op_state !=true*/)
        usleep(1000);      
     checkMasterState();
     // printf("Master AL states: 0x%02X.\n", master_state_.al_states);
     checkMasterBusScaning();
//     std::cout<<"\nMaster:653: "<<master_state_.link_up<<" "<<master_state_.slaves_responding<<"\n";

    if(/*!master_info.scan_busy &&*/ /*slave_status==true &&*/ master_state_.link_up == 1 && master_state_.slaves_responding>=NUM_JOINTS)
    {
    if(!Initialisation_Flag )

    {
//      std::cout<<"\nMaster:660\n";

          DomainInfo* domain_info = domain_info_[domain];
          if (domain_info!=NULL  )
          {
//            std::cout<<"\nMaster:665\n";

                ecrt_domain_process(domain_info->domain);
//                std::cout<<"\nMaster:668\n";

                // check process data state (optional)
//                std::cout<<"\n\n\nHere:\n\n\n\n ";
                checkDomainState(domain);
//                std::cout<<"\nMaster:673\n";

                // if (update_counter_ % check_state_frequency_ == 0){
                //   checkMasterState();
                //   slave_status=checkSlaveStates();
                // }
//                std::cout<<"\nMaster:679\n";

               
                int id;
                if(!master_info.scan_busy && slave_status==true)
                {

//                  std::cout<<"\nMaster:684\n";

                        if(temp_count==1000)            
                        {
                          *ready=true;
                          temp_wait=false;
                          //      std::cout<<"---------------------Temp read timeout!!--------------------\n";
//                          for(int i=0;i<7;i++)
//                          {
//                            drive_temperature_[i]=-1;
//                          }
                          temp_count=0;
                        }
//                        std::cout<<"\nMaster:697\n";

                        if(*temp && temp_wait==false && *ready==false)
                        {
                          for(int i=0;i<NUM_JOINTS;i++)
                          {
                            success[i]=false;
                          }
                          temp_wait=true;
                          temp_count=0;
                        }
//                        std::cout<<"\nMaster:708\n";

                        if(temp_wait)
                        {
                          temp_count++;
                          for(int i=0;i<NUM_JOINTS;i++)
                          {
                            if(success[i]==false)
                            {

                              if(slave_obj_map.find(slave_name_map["SOMANET Circulo CiA402 Drive"])!=slave_obj_map.end() && !Initialisation_Flag)
                              {
                                if(sdo_req[i])
                                result[i]=read_sdo(sdo_req[i],i);
                                // sdo_errorcode[i]=read_sdo(sdo_err_req[i],i);
                              }
                            }
                          }
//                          std::cout<<"\nMaster:727\n";

                        }
                        if(temp_wait)
                        {

                          bool succ=true;
                          for(int j=0;j<NUM_JOINTS;j++)
                         {
                            if(success[j]==false)
                            {
                              succ=false;
                            }
                          }

                          if(succ)
                          {
                            temp_wait=false;
                            *ready=true;
//                            for(int i=0;i<NUM_JOINTS && !Initialisation_Flag;i++)
//                            {

//                              drive_temperature_[i]=((double)(int)EC_READ_S32(result[i]))/1000;
//                              // drive_erro_code_[i]=  EC_READ_S32(sdo_errorcode[i]);
//                            }
                          }
                        }
                }
               

                
                  for (DomainInfo::Entry& entry : domain_info->entries){
                      
                      for (int i=0; i<entry.num_pdos; )
                      {
                           id= (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
                           // printf("offset%d\n", entry.offset[i] );
                           i=i+id;
                          
                      }
                  }
                  // send process data
//                  std::cout<<"\nMaster:759\n";

                ecrt_domain_queue(domain_info->domain);
          }
      }

      }
    ecrt_master_send(master_);
//    std::cout<<"\nMaster:765\n";

      // send_t = std::chrono::steady_clock::now();
       clock_gettime(CLOCK_MONOTONIC,&send_t);
      ++update_counter_;
   
  }



void Master::setCtrlCHandler(SIMPLECAT_EXIT_CALLBACK user_callback)
{
  // ctrl c handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = user_callback;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

void Master::run(SIMPLECAT_CONTRL_CALLBACK user_callback, double frequency)
{
  printf("Running loop at [%.1f] Hz\n", frequency);

  unsigned int interval = 1000000000.0/frequency;

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC ,&t);
  t.tv_sec++;
  unsigned int domain = 0;
  running_ = true;
  start_t_ = std::chrono::system_clock::now();
  while(running_)
  {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    // update EtherCAT bus
    //this->update();
    // receive process data
    ecrt_master_receive(master_);

    DomainInfo* domain_info = domain_info_[domain];

    ecrt_domain_process(domain_info->domain);

    // check process data state (optional)
    checkDomainState(domain);

    // check for master and slave state change
    if (update_counter_ % check_state_frequency_ == 0){
      checkMasterState();
      checkSlaveStates();
    }
    // read and write process data
    for (DomainInfo::Entry& entry : domain_info->entries){
      for (int i=0; i<entry.num_pdos; ++i){
        (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
      }
    }
    // user callback
    user_callback();

    // send process data
    ecrt_domain_queue(domain_info->domain);
    ecrt_master_send(master_);
    ++update_counter_;

    // get actual time
    curr_t_ = std::chrono::system_clock::now();

    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += interval;
    while (t.tv_nsec >= 1000000000){
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }
}

double Master::elapsedTime()
{
  std::chrono::duration<double> elapsed_seconds = curr_t_ - start_t_;
  return elapsed_seconds.count()-1.0; // started after 1 second
}

unsigned long long Master::elapsedCycles()
{
  return update_counter_;
}



void Master::printWarning(const std::string& message)
{
  std::cout << "WARNING. Master. " << message << std::endl;
}

}
