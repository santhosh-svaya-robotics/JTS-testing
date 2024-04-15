#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <svaya_joint_state_reader/joint_state_reader.h>

#include<controller_manager_msgs/srv/switch_controller.hpp>
#include <semaphore.h>


namespace trajectory_publisher
{

class TrajectoryPublisher
{
public:
  TrajectoryPublisher(sem_t *lock, int *exe_flag);
  ~TrajectoryPublisher();
  /**
     * Executes trajectory by publishing it to the /joint_trajectory_controller/joint_trajectory topic
     * @param[in] traj vector of double vectors each containing the time stamp and the joint positions
     * @return 1 if successful, 0 otherwise.
     */
  int PublishTrajectory(double joint_vel_, std::vector<double> p1, std::vector<double> p2);
  /**
     * Set the maximum joint velocity of joints
     * @param[in] max_vel maximum_velocity vector(radian/s)
     */
  void SetMaxJointVel(std::vector<double> max_vel /*max_vel*/);
  /**
     * Set the maximum joint acceleration of joints
     * @param[in] max_acc maximum_velocity vector(radian/s)
     */
  void SetMaxJointAcc(std::vector<double> max_acc /*max_acc*/);
  /**
     * Get the current joint position
     * @param[in] max_acc maximum_velocity vector(radian/s)
     */
  void GetJointPos(std::vector<double> &pos);
  /**
     * Get the current joint velocity
     * @param[in] max_acc maximum_velocity vector(radian/s)
     */
  void GetJointVel(std::vector<double> &vel);
  /**
     * Execute trajectory from a csv-file. first element of each line of csv
     * should be time in seconds from the start of execution and rest should be
     * joint positions in the right order
     * @param[in] path: path of the file as string.
     */
  int ExecuteFromCsv(std::string path /*path*/);

  int goToPos(std::vector<double> pos, double vel);

  int ExecuteFromTrajMsg(trajectory_msgs::msg::JointTrajectory &jointTraj);
  int startController();
  int stopController();
private:
sem_t *lock_;
int *exe_flag;

int targetWatchDog();
rclcpp::Node::SharedPtr node;
int ValidateTrajectory();
int ValidateTrajectory(trajectory_msgs::msg::JointTrajectory &jointTraj);

rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_publisher;
double jv_ja_to_time(std::vector<double> desired_joint_vel, std::vector<double> desired_joint_acc, std::vector<double> init_joint_val, std::vector<double> final_joint_val,
                     std::vector<double> prev_joint_val={},std::vector<double> prev_joint_vel={}, std::vector<double> prev_joint_acc={});
double point_to_point_feat_time (std::vector<double> init_joint_val, std::vector<double> final_joint_val, double segment_time,
                                 std::vector<double>& last_joint_pos,std::vector<double> desired_joint_acc);

int point_to_point_stop(std::vector<double> curr_joint_pos, std::vector<double> curr_joint_vel);



std::vector<double> target_position;
std::vector<double> current_position;
std::vector<double> current_velocity;
double current_traj_time;
svaya_joint_state_reader::JointStateReader js;
std::vector<double> MAX_JOINT_MOTION_VEL = {M_PI/2,M_PI/2};
std::vector<double> MAX_JOINT_MOTION_ACC = {M_PI/2,M_PI/2};
int numJoints = 1;
std::vector<double> joint_limits_upper = {0.9*M_PI/2,0.9*M_PI/2};
std::vector<double> joint_limits_lower = {-0.9*M_PI/2,-0.9*M_PI/2};

rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_control;

};

}

#endif
