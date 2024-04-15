#ifndef JOINT_STATE_READER_H
#define JOINT_STATE_READER_H

#include <rclcpp/rclcpp.hpp>
#include<std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <semaphore.h>
#include <math.h>
#include <iostream>

//TODO : define as global variable
#define NUM_JOINTS 1
namespace svaya_joint_state_reader {

class JointStateReader
{

public:
  JointStateReader();
  virtual ~JointStateReader();


    double* getJointStates();

    void getJointStates(std::vector<double> &jp);

    /*!
     * \fn    getJointStates()
     * \brief This method returns the latest joint state values of the robot.
     * \return  double* : It returns an array of double values of size 7 corresponding
     *          to the 7 joints of the robot.
     */
    double* getJointVelStates();
    void getJointVelStates(std::vector<double> &jv);

    /*!
     * \brief run : Contains the code for monitoring/sampling the robot's joint values @ 2000Hz.
     * Releases the semaphores so that other modules waiting on it can be notified to read
     * the latest vales at regular intervals.
     */
    void run();

    void joint_values_callback(const std_msgs::msg::Float64MultiArray &msg);

    std::vector<double> getJointVelStates_as_vec();


private:


  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber;

  bool ready=false;
//  double *jointState= new double[6];

  //TODO : define as global variable
  double *joint_pos= new double[NUM_JOINTS];
  std::vector<double> joint_pos_vec;
  std::vector<double> *joint_pos_vec_ptr;
  std::vector<double> *joint_vel_vec_ptr;

  double *joint_vel= new double[NUM_JOINTS];
  std::vector<double> joint_vel_vec;

  std::thread reader_thread;

  int call_back_flag = 0;
  bool stop = false;
  int numJoints = NUM_JOINTS;
  std::thread th;
  bool thread_running = true;
};

}

#endif // JOINT_STATE_READER_H
