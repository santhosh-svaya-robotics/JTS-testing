#ifndef EMBR_H
#define EMBR_H

#include <chrono>
#include <memory>
#include <string>
#include <vector>




#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>


#include <hardware_interface/visibility_control.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
//#include "SvayaDriver.h"
#include <global_shared_variables.h>

#include <knexo_ethercat_master.h>
#include <knexo_synapticon_dc1k_d2.h>
#include <knexo_synapticon_io.h>

bool Initialisation_Flag;

std::bitset<NUM_JOINTS> drive_fault_state;
std::bitset<NUM_JOINTS> drive_warn_state;
std::bitset<NUM_JOINTS> drive_conn_state;


sem_t *temperature_lock;

double low_pass_position_cut_off_hz = 100;
double low_pass_velocity_cut_off_hz = 300;
double low_pass_torque_cut_off_hz = 100;

bool drive_fault_triggered;

int *estop_triggered;

knexo_ethercat::Master master(0);
std::vector<knexo_ethercat::Synapticon_DC1K_D3 *> drive;

namespace embr
{
    class embrHW : public hardware_interface::SystemInterface
    {
      public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type Slave_obj_assignment(knexo_ethercat::Master &master, std::vector<knexo_ethercat::Synapticon_DC1K_D3 *> &drive);
        void controller_status();
        void read_parameters();
      private:
        double mean(std::vector<double> &diff_vec);
        void add_data( std::vector<double> &diff_vec,double data);
      std::vector<int> pos_sign;
      std::vector<int> vel_sign;

      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_publisher;


      std_msgs::msg::Float64MultiArray fts_sensor_data;
//            svaya_driver::SvayaDriver *svaya_driver;

        std::vector<std::string> active_controllers_;

        std::shared_ptr<rclcpp::Node> node;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torq_sens;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr analog_signal;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr analog_scaled_signal;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_torq_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dc_link_vol_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_torq_pub;


        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr enc_pos_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torq_est_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr model_torq_pub;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr calibration_sub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gc_flag_sub;



        std_msgs::msg::Float64MultiArray state_msg;

        std_msgs::msg::Float64MultiArray Torq;

        std_msgs::msg::Float64MultiArray enc_pos_data;
        std_msgs::msg::Float64 torq_sensor_data;
        std_msgs::msg::Float64 analog_data;
        std_msgs::msg::Float64 analog_scaled_data;


        std_msgs::msg::Float64 torq_est;

        std::vector<double> cmd_pos;
        std::vector<double> cmd_vel;
        std::vector<double> cmd_acc;
        std::vector<double> cmd_eff;

        std::vector<int64_t> gear_rato;
        std::vector<int64_t> enc_count;
        std::vector<double> rated_torq;

        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> acc;
        std::vector<double> eff;

        std::vector<double> dummy_pos;
        std::vector<double> dummy_vel;
        std::vector<double> dummy_eff;

        double hw_start_sec_;
        double hw_stop_sec_;
        double hw_slowdown_;

        bool virtual_flag;
        bool initialized;
        bool joint_trajectory_controller_status;
        bool engaged;
        // std::vector<integration_level_t> control_level_;

        static rclcpp::Logger getLogger();
        bool parameter_reading_done;

        int analog_in;
        std::vector<double> analog_vec;

        void controller_callback(const rclcpp::Parameter &P);
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> controller_handle;


        std::shared_ptr<rclcpp::SyncParametersClient> parameter_client;
    };
};

#endif // EMBR_H
