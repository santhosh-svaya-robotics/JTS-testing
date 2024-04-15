#include <svaya_joint_state_reader/joint_state_reader.h>

namespace svaya_joint_state_reader {

JointStateReader::JointStateReader()
{
//  std::cout<<"..................... JS_reader constructor....................................\n";

  node = rclcpp::Node::make_shared("svaya_joint_state_reader"+std::to_string(rclcpp::Clock().now().nanoseconds()));
//  RCLCPP_INFO_STREAM(node->get_logger(),"...............Constructor...............");
  for(int i=0;i<numJoints;i++)
    {
      joint_pos[i]=0;
      joint_vel[i]=0;
    }
  joint_pos_vec.resize(NUM_JOINTS,0);
  joint_vel_vec.resize(NUM_JOINTS,0);
  joint_pos_vec_ptr = new std::vector<double>(NUM_JOINTS);
  joint_vel_vec_ptr = new std::vector<double>(NUM_JOINTS);
  subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/joint_state", 1000, std::bind(&JointStateReader::joint_values_callback, this, std::placeholders::_1));
  sleep(2);
  thread_running = true;
  th = std::thread(&JointStateReader::run,this);
}

JointStateReader::~JointStateReader()
{
  if(thread_running)
  {
    thread_running = false;
    rclcpp::shutdown();
    th.join();
  }
}

void JointStateReader::joint_values_callback(const std_msgs::msg::Float64MultiArray &msg)
{
  for(unsigned int cnt=0; cnt<NUM_JOINTS; cnt++)
  {
    joint_pos[cnt] = msg.data[cnt];
//    std::cout<<msg.data[cnt]<<std::endl;
    joint_pos_vec[cnt] = msg.data[cnt];
//    joint_pos_vec_ptr[cnt] = msg.data[cnt];
    joint_pos_vec_ptr->at(cnt) = msg.data[cnt];
    joint_vel[cnt] = msg.data[cnt+NUM_JOINTS];
    joint_vel_vec_ptr->at(cnt) = msg.data[cnt+NUM_JOINTS];

//    std::cout<<cnt+1<<" pos "<<msg.data[cnt]<<" vel "<<msg.data[cnt+NUM_JOINTS]<<"\n";
  }
}


void JointStateReader::run()
{

  rclcpp::spin(node);

  return;
}

double* JointStateReader::getJointStates()
{
  return  joint_pos;
}

void JointStateReader::getJointStates(std::vector<double> &jp)
{
joint_pos_vec_ptr = &jp;
}

double* JointStateReader::getJointVelStates()
{
  return  joint_vel;
}

void JointStateReader::getJointVelStates(std::vector<double> &jv)
{
  joint_vel_vec_ptr = &jv;
}

std::vector<double> JointStateReader::getJointVelStates_as_vec()
{
  for(unsigned int i=0; i<numJoints ; i++)
  {
    joint_vel_vec[i] = joint_vel[i];
  }
  return  joint_vel_vec;
}
}
