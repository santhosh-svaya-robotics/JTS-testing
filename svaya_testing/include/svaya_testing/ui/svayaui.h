#ifndef SVAYAUI_H
#define SVAYAUI_H

#include <QMainWindow>
#include <svaya_trajectory_publisher/trajectory_publisher.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <fstream>


namespace Ui {
class SvayaUi;
}

class SvayaUi : public QMainWindow
{
    Q_OBJECT

public:
    explicit SvayaUi(QWidget *parent = nullptr);
    ~SvayaUi();


private slots:
  void on_stop_record_clicked();

private slots:
  void on_startRecord_clicked();

//private slots:
//  void on_spinBox_valueChanged(int arg1);

private slots:
    void on_plan_clicked();

    void on_stop_clicked();

    void on_speed_valueChanged(int value);

    void on_time_duration_valueChanged(int arg1);

    void on_error_spin_valueChanged(int arg1);


    void on_touchStop_toggled(bool checked);

    void on_goToHome_clicked();

    void on_calibrate_clicked();

//    void on_static_clicked();



    void on_static_2_clicked();

//    void on_brakeEngage_stateChanged(int arg1);

    void on_gchold_toggled(bool checked);

    void on_doubleSpinBox_valueChanged(double arg1);

    void on_doubleSpinBox_2_valueChanged(double arg1);

    void on_doubleSpinBox_3_valueChanged(double arg1);

    void on_doubleSpinBox_4_valueChanged(double arg1);

    void on_doubleSpinBox_5_valueChanged(double arg1);

    void on_doubleSpinBox_6_valueChanged(double arg1);

    void on_doubleSpinBox_11_valueChanged(double arg1);

    void on_doubleSpinBox_7_valueChanged(double arg1);

    void on_doubleSpinBox_15_valueChanged(double arg1);

    void on_doubleSpinBox_16_valueChanged(double arg1);

    void on_doubleSpinBox_13_valueChanged(double arg1);

    void on_doubleSpinBox_14_valueChanged(double arg1);

    void on_doubleSpinBox_19_valueChanged(double arg1);



    void on_doubleSpinBox_37_valueChanged(double arg1);

    void on_doubleSpinBox_34_valueChanged(double arg1);

    void on_doubleSpinBox_38_valueChanged(double arg1);

    void on_doubleSpinBox_32_valueChanged(double arg1);

    void on_doubleSpinBox_29_valueChanged(double arg1);

    void on_doubleSpinBox_41_valueChanged(double arg1);

    void on_doubleSpinBox_25_valueChanged(double arg1);

    void on_doubleSpinBox_23_valueChanged(double arg1);

    void on_doubleSpinBox_30_valueChanged(double arg1);

    void on_horizontalSlider_sliderMoved(int position);

private:
    void grav_torq_callback(const std_msgs::msg::Float64MultiArray & msg);
    void torq_sensor_callback(const std_msgs::msg::Float64 & msg);
    void analog_in_callback(const std_msgs::msg::Float64 & msg);
    void enc_data_callback(const std_msgs::msg::Float64MultiArray & msg);
    void error_callback(const std_msgs::msg::Float64 & msg);


    void error_check();

    rclcpp::Node::SharedPtr node;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr grav_subs;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr enc_data_subs;
//    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr calibrate_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gc_flag_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gc_sens;



    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sensor_subs;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr analog_subs;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_sub;


    double p_pos;
    double s_pos;
    float error;

    Ui::SvayaUi *ui;
    int numJoints = 1;
    sem_t lock;
    int exe_flag = 1;
    double joint_vel = M_PI/6;
    trajectory_publisher::TrajectoryPublisher *publisher;
    std::thread th;
    std::thread th2;
    bool isMotionActive;

    double grav_torq;
    double actual_torq;
    double analog_in;

    double duration_stop = 4.0;
    bool controller_started = false;

    double err_ = 5;

    bool touch_stop = false;

    bool enable_touch_stop = false;
    bool gchold = false;

    std::vector<double> goTo = {0};

    std::string file_name = "recorded.csv";

    std::fstream fout;
    std::fstream fout2;
    std::fstream fout3;

    double intercept = 32106;
    double slope = 52.760;
    double stiffness = 2310.0;
    double COG_offset = 1.85;
    bool start_record = false;
    std::vector<double> range_lower_limit = {0};
    std::vector<double> range_upper_limit={0};
};

#endif // SVAYAUI_H

