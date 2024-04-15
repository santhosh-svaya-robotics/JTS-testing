/*!
 * \author Rohan Bampal
 *
 * \file include/global_user_info.h
 *
 * \brief This header will store the single instance of variables defined
 * 		  in this file. There will be a single instance of the variables
 *        where ever this header file is included.
 *        The variables points to the shared memory segment to be used by
 *        other modules.
 *
 * \date 26th September, 2018
 **/

#ifndef GLOBAL_SHARED_VARIABLES_H
#define GLOBAL_SHARED_VARIABLES_H
#include<string>
#include<semaphore.h>
#include<vector>
#include<bitset>


#define RUN_GAZEBO 0

#define RUN_UI 1

// 0 For old 7 axis alpha robot
// 1 For new 7 axis robot
// 2 For new 6 axis 6kg v1 robot
// 3 For 6 axis 6kg v2 robot
// 4 For 6 axis 3kg robot
// 5 For 6 axis 12kg robot
// 6 For 6 axis robot with Aksim encoders
#define ROBOT_VERSION 3

#if ROBOT_VERSION>=2
#define NUM_JOINTS 1
#else
#define NUM_JOINTS 7
#endif

#define DRIVES_PRESENT 1

extern int *auto_out;
extern int *auto_in;
extern bool Initialisation_Flag;
extern bool rv_flag;
extern bool *safe_stop_done;
extern uint8_t safe_stop_type;

extern double *drive_temperature;
extern int *drive_error_codes;

extern int *shm_safety_in;
extern int *shm_safety_out;
extern int *shm_dedicated_digital_in;
extern int *shm_dedicated_digital_out;
extern double *shm_force_mode_data;
extern bool *shm_force_mode_enable;
extern bool *shm_mode_of_operation;
extern bool *shm_recovery_mode_flag;
extern uint32_t *shm_safety_recovery_params;
//extern bool *real_virtual_flag;

extern double *shm_safety_plane_data;
extern double *shm_safety_tool_position;
extern double *shm_safety_tool_direction;
extern double *shm_safety_joint_limits;
extern double *shm_safety_robot_limits;
extern double *shm_safety_error_list;

extern bool reduced_mode;
extern bool *new_traj;
extern double time_red_mode;
extern double last_time_val;
extern double red_factor;
extern double time_data_uptime;
extern double last_time_normal_mode;
extern double prev_start_time;
extern std::vector<double> traj_start_time;
extern int traj_start_time_ctr;
extern bool slider_traj;
extern bool ft_sensor_present;
extern double ft_sensor_values[6];
extern bool underSingularity;
extern bool too_close_point;
extern std::vector<double> joint_position_encoders;
extern std::vector<double> joint_velocity_encoders;
extern std::vector<double> joint_position_values_effort;
extern std::vector<double> joint_velocity_values_effort;
extern std::vector<double> joint_position_real_command_effort;
extern std::vector<double> joint_secondary_position_;
extern std::vector<double> joint_secondary_velocity_;
extern std::vector<double> desired_joint_vel_data;

//
extern std::vector<double> joint_pos_test;
extern int test;
//

extern bool reduced_mode_transistion;

extern std::vector<double> joint_last_command_sent;

extern bool pos_eff_controller_running;
extern int controller_transistion_ctr;

extern std::vector<double> joint_secondary_position;
extern std::vector<double> joint_secondary_velocity;
extern std::vector<double> estimated_torque;

extern std::vector<std::vector<double>> fric_coeff_pos;
extern std::vector<std::vector<double>> fric_coeff_neg;
extern std::vector<std::vector<double>> stiffness_coeff;
extern std::vector<double> rotor_inertia;
extern std::vector<double> joint_rated_torque;
extern std::vector<double> fric_upper_cutoff;
extern std::vector<double> fric_lower_cutoff;
extern std::vector<double> torque_constant;
extern std::vector<double> gear_ratio;
extern std::vector<double> gravity_fric_coeff;
extern std::vector<double> touch_stop_coeff_less;
extern std::vector<double> touch_stop_coeff_more;
extern std::vector<double> touch_stop_coeff_most;
extern int touch_stop_mode;
extern double gravity_torque_coeff;
extern bool force_mode_transistion;

extern std::vector<double> Kp_joint;
extern std::vector<double> Kv_joint;
extern std::vector<double> Kp_cart;
extern std::vector<double> Kv_cart;

extern std::vector<double> joint_position_brake;

extern bool transition;

extern bool drive_init;

extern bool safety_limits_flag;
extern bool safety_flag;

extern bool *force_stop_enable;
extern bool force_test_controller_enable;

extern std::vector<double> pos_sign;
extern std::vector<double> vel_sign;
extern std::vector<double> torq_sign;

extern std::bitset<NUM_JOINTS> drive_fault_state;
extern std::bitset<NUM_JOINTS> drive_warn_state;
extern std::bitset<NUM_JOINTS> drive_conn_state;

extern bool drive_fault_triggered;

extern int *estop_triggered,prev_estop_state;

extern std::vector<double> payload_torque_offset;

#endif // GLOBAL_SHARED_VARIABLES_H
