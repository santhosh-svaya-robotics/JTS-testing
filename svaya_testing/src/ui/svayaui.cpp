#include "svaya_testing/ui/svayaui.h"
#include "ui_svayaui.h"
//#include "svaya_hw/include/svaya_hw.h"
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>


SvayaUi::SvayaUi(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SvayaUi)
{
    node = rclcpp::Node::make_shared("ui_node");

    sem_init(&lock,0,1);
    publisher = new trajectory_publisher::TrajectoryPublisher(&lock, &exe_flag);
    ui->setupUi(this);
    publisher->startController();
    isMotionActive = false;
    grav_torq = 0;
    actual_torq = 0;



    grav_subs = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/model_torq",100, std::bind(&SvayaUi::grav_torq_callback, this, std::placeholders::_1));
    sensor_subs = node->create_subscription<std_msgs::msg::Float64>("/svaya/analog_scaled_signal",100, std::bind(&SvayaUi::torq_sensor_callback, this, std::placeholders::_1));
    analog_subs = node->create_subscription<std_msgs::msg::Float64>("/svaya/analog_signal",100, std::bind(&SvayaUi::analog_in_callback, this, std::placeholders::_1));

    enc_data_subs   = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/enc_pos",100, std::bind(&SvayaUi::enc_data_callback, this, std::placeholders::_1));
    error_sub   = node->create_subscription<std_msgs::msg::Float64>("/svaya/torq_error",100, std::bind(&SvayaUi::error_callback, this, std::placeholders::_1));

    calibrate_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/calibrate",100);
    gc_flag_pub = node->create_publisher<std_msgs::msg::Bool>("/svaya/gc_flag",100);
    gc_sens = node->create_publisher<std_msgs::msg::Float64>("/svaya/gc_sens",100);


    auto err_callback = [this](){this->error_check();};
    th2 = std::thread(err_callback);
}

SvayaUi::~SvayaUi()
{
    delete ui;
    rclcpp::shutdown();
}

void SvayaUi::on_plan_clicked()
{
    if(isMotionActive)
    {
      return;
    }
    auto thread = [this]()
    {
        this->exe_flag = 1;
        isMotionActive = true;
        this->publisher->PublishTrajectory(joint_vel, range_lower_limit, range_upper_limit);
    };

    if(!isMotionActive)
    {
      th = std::thread(thread);
    }
    //    ui->plan->hide();
    ui->plan->setDisabled(true);
//    ui->error_spin->hide();
//    ui->stop->setDisabled(true);
    ui->error_spin->setDisabled(true);
    ui->time_duration->setDisabled(true);
    ui->speed->setDisabled(true);
    ui->goToHome->setDisabled(true);
    ui->gchold->setDisabled(true);
}

void SvayaUi::on_stop_clicked()
{
    if(touch_stop)
    {
      isMotionActive = false;
    }
    exe_flag = -2;
    if(th.joinable())
        th.join();

    isMotionActive = false;
    ui->plan->setEnabled(true);
    ui->error_spin->setEnabled(true);
    ui->touchStop->setEnabled(true);
//    ui->error_spin->show();
    ui->speed->setEnabled(true);
    ui->time_duration->setEnabled(true);
    ui->goToHome->setEnabled(true);
    ui->gchold->setEnabled(true);
}

void SvayaUi::on_speed_valueChanged(int value)
{
    joint_vel = 2*M_PI*(double)value/360;
    ui->speed_val->setText(std::to_string(value).c_str());
}

void SvayaUi::grav_torq_callback(const std_msgs::msg::Float64MultiArray &msg)
{
    grav_torq = msg.data[0];
}

void SvayaUi::torq_sensor_callback(const std_msgs::msg::Float64 &msg)
{
  actual_torq = msg.data;
}

void SvayaUi::analog_in_callback(const std_msgs::msg::Float64 &msg)
{
  analog_in = msg.data;
}

void SvayaUi::enc_data_callback(const std_msgs::msg::Float64MultiArray &msg)
{
  p_pos = msg.data[0];
  s_pos = msg.data[1];
}

void SvayaUi::error_callback(const std_msgs::msg::Float64 &msg)
{
  error = msg.data;
}



void SvayaUi::error_check()
{
    bool stopped = false;
    bool trigger = false;

    auto start_time = rclcpp::Clock().now();
    int k = 0;

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
//        std_msgs::msg::Float64 error;
//        error.data = (grav_torq - actual_torq);
//        error_pub->publish(error);

        if(isMotionActive && enable_touch_stop)
        {
            k++;

            if(fabs(error) > err_)
            {
                trigger = true;
                k = 0;
            }
            std::cout<<trigger<<std::endl;
            if (trigger)
            {
              std::cout<<"stopping\n\n"<<std::endl;
                RCLCPP_INFO_STREAM(node->get_logger(),"triggered");
                if(exe_flag ==1)
                {
                    on_stop_clicked();
//                    ui->stop->setDisabled(true);
                }
                stopped = true;
                start_time = rclcpp::Clock().now();
                trigger = false;
            }

//            if(stopped)
//            {
//                touch_stop = true;
//                auto current_time = rclcpp::Clock().now();
//                auto duration = current_time - start_time;
//                double duration_sec = (double)duration.seconds() + (double)duration.nanoseconds()*1e-9;
//                RCLCPP_INFO_STREAM(node->get_logger(),"stopped for : " <<duration_sec);
//                if(duration_sec > duration_stop)
//                {
//                    if(exe_flag == -1 && fabs(error) < err_)
//                    {
//                        exe_flag = 1;
//                        sem_post(&lock);
//                        ui->stop->setEnabled(true);
//                        touch_stop = false;
//                        RCLCPP_INFO_STREAM(node->get_logger(),"starting ");
//                    }
//                    stopped = false;
//                }
//            }
        }

        usleep(1000);
    }
}

void SvayaUi::on_time_duration_valueChanged(int arg1)
{
    duration_stop = (double)arg1;
}


void SvayaUi::on_error_spin_valueChanged(int arg1)
{
    err_ = arg1;
}



void SvayaUi::on_touchStop_toggled(bool checked)
{
//  RCLCPP_INFO(node->get_logger(),"touch stop clicked");
    enable_touch_stop = checked;
}
void SvayaUi::on_gchold_toggled(bool checked)
{
  std::cout<<checked<<std::endl;
    gchold = checked;
    std_msgs::msg::Bool gchold_flag;
    gchold_flag.data = gchold;
    if(checked){
      ui->speed->setDisabled(true);
      ui->goToHome->setDisabled(true);
      ui->touchStop->setDisabled(true);
      ui->plan->setDisabled(true);
      ui->error_spin->setDisabled(true);
      ui->time_duration->setDisabled(true);
    }
    if(!checked){
      ui->speed->setEnabled(true);
      ui->goToHome->setEnabled(true);
      ui->touchStop->setEnabled(true);
      ui->plan->setEnabled(true);
      ui->error_spin->setEnabled(true);
      ui->time_duration->setEnabled(true);
    }
    gc_flag_pub->publish(gchold_flag);
}
void SvayaUi::on_goToHome_clicked()
{
    exe_flag = 1;

    ui->touchStop->setDown(true);
    touch_stop = false;
    this->publisher->goToPos(goTo, joint_vel);
}

//void SvayaUi::on_spinBox_valueChanged(int arg1)
//{
//    goTo = (double)arg1*M_PI/180;
//}


void SvayaUi::on_startRecord_clicked()
{
    file_name = ui->fileName->text().toStdString() + ".csv";
    if(!start_record)
    {
      start_record = true;
      fout.open(file_name, std::ios::out);
      fout2.open("enc_data.csv",std::ios::out | std::ios::trunc);
      fout<<"torq sensor, th torq, error, analog in, voltage, encoder\n";
      fout2<<"grav_torq, p_pos, s_pos";

    }
}

void SvayaUi::on_stop_record_clicked()
{
    if(start_record)
    {
      fout.close();
      fout.clear();
      fout2.clear();
      start_record = false;
    }
}


void SvayaUi::on_calibrate_clicked()
{
    this->publisher->goToPos({90*M_PI/180}, joint_vel);
    sleep(8);
    this->publisher->goToPos({90*M_PI/180}, joint_vel);
    sleep(2);
    this->publisher->goToPos({90*M_PI/180}, joint_vel);
    sleep(4);
    this->publisher->goToPos({90*M_PI/180}, joint_vel);

    double analog_value_1 = 0.00;
    for(int i=1;i<1000;i++){
        if(i==1){
            analog_value_1 = analog_in;
        }
        analog_value_1 = (analog_value_1 + analog_in)/2;
        usleep(5000);
        ;
    }

    this->publisher->goToPos({90*M_PI/180}, joint_vel);
    sleep(8);
    this->publisher->goToPos({90*M_PI/180}, joint_vel);
    sleep(2);
    this->publisher->goToPos({90*M_PI/180}, joint_vel);
    sleep(4);
    this->publisher->goToPos({90*M_PI/180}, joint_vel);

    double analog_value_2 = 0.00;
    for(int i=1;i<1000;i++){
        if(i==1){
            analog_value_2 = analog_in;
        }
        analog_value_2 = (analog_value_2 + analog_in)/2;
        usleep(5000);
        ;
    }
    double analog_value_0 = (analog_value_1 + analog_value_2)/2;
    double analog_value_90 = fabs(analog_value_1 - analog_value_2)/2;
    std_msgs::msg::Float64MultiArray cal_data;
    cal_data.data.resize(2);
    cal_data.data[0] = analog_value_0*5.61132/analog_value_90;
    cal_data.data[1] = 5.61132/analog_value_90;

    calibrate_pub->publish(cal_data);
    
    

}

//void SvayaUi::on_static_clicked()


//  {
//    fout3.open("static_error.csv", std::ios::out);

//    for (int motion_loop=0;motion_loop<=2;motion_loop++)
//    {
//      int start_pos;
//      int stop_pos;
//      int increment;
//      if (motion_loop==0)
//      {
//        start_pos = 0;
//        stop_pos = 90;
//        increment = 10;
//      }
//      else if (motion_loop==1)
//      {
//        start_pos = 90;
//        stop_pos = -90;
//        increment = -10;
//      }
//      else
//      {
//        start_pos = -90;
//        stop_pos = 0;
//        increment = 10;
//      }
//      for(int i=start_pos; i!=stop_pos; i=i+increment){

//        double target =i*M_PI/180;
//        this->publisher->goToPos(target, 90*M_PI/180);
//        sleep(3);
//        this->publisher->goToPos(target, 90*M_PI/180);
//        sleep(1);
//        this->publisher->goToPos(target, 90*M_PI/180);
//        sleep(1);
//        this->publisher->goToPos(target, 90*M_PI/180);
//        sleep(1);
//        this->publisher->goToPos(target, 90*M_PI/180);
//        double e = 0;
//        double an = 0.0;
//        for(int j=1;j<800;j++){
//                if(j==1){
//                    e = grav_torq - actual_torq;
//                    an = analog_in;
//                }
//                e = (e + grav_torq - actual_torq)/2;
//                an = (an + analog_in)/2;
//                usleep(5000);
//                ;
//            }
//        fout3<<target<<","<<p_pos<<","<<e<<","<<actual_torq<<","<<an<<"\n";

//      }
//    }
//    fout3.close();
//}



void SvayaUi::on_static_2_clicked()
{
  fout3.open("static_error.csv", std::ios::out);
  for (int iteration= 0; iteration<10; iteration ++){


  for (int motion_loop=0;motion_loop<=4;motion_loop++)
  {
    int start_pos;
    int stop_pos;
    int increment;
    if (motion_loop==0)
    {
      start_pos = 0;
      stop_pos = 90;
      increment = 5;
    }
    else if (motion_loop==1)
    {
      start_pos = 90;
      stop_pos = -90;
      increment = -5;
    }
    else if (motion_loop==2)
    {
      start_pos = -90;
      stop_pos = 90;
      increment = 5;
    }
    else if (motion_loop==3)
    {
      start_pos = 90;
      stop_pos = -90;
      increment = -5;
    }

    else
    {
      start_pos = -90;
      stop_pos = 0;
      increment = 5;
    }
    for(int i=start_pos; i!=stop_pos; i=i+increment){

      double target =i*M_PI/180;
      this->publisher->goToPos({target}, 20*M_PI/180);
      sleep(3);
//      this->publisher->goToPos(target, 20*M_PI/180);
//      sleep(1);
//      this->publisher->goToPos(target, 20*M_PI/180);
//      sleep(1);
//      this->publisher->goToPos(target, 20*M_PI/180);
//      sleep(1);
//      this->publisher->goToPos(target, 20*M_PI/180);
      double e = 0;
      double an = 0.0;
      for(int j=1;j<50;j++){
              if(j==1){
                  e = grav_torq - actual_torq;
                  an = analog_in;
              }
              e = (e + grav_torq - actual_torq)/2;
              an = (an + actual_torq)/2;
              usleep(100);
              ;
          }
      fout3<<target<<","<<p_pos<<","<<e<<","<<actual_torq<<","<<an<<"\n";

    }
  }
  }
  fout3.close();
}

//void SvayaUi::on_brakeEngage_stateChanged(int arg1)
//{
//  return;
//}





void SvayaUi::on_doubleSpinBox_valueChanged(double arg1)
{
    // slope
  slope = arg1;
  std_msgs::msg::Float64MultiArray cal_data;
  cal_data.data.resize(4);
  cal_data.data[0] = slope;
  cal_data.data[1] = intercept;
  cal_data.data[2] = stiffness;
  cal_data.data[3] = COG_offset;


  calibrate_pub->publish(cal_data);

}

void SvayaUi::on_doubleSpinBox_2_valueChanged(double arg1)
{
    // intercept
  intercept = arg1;
  std_msgs::msg::Float64MultiArray cal_data;
  cal_data.data.resize(4);
  cal_data.data[0] = slope;
  cal_data.data[1] = intercept;
  cal_data.data[2] = stiffness;
  cal_data.data[3] = COG_offset;


  calibrate_pub->publish(cal_data);
}

void SvayaUi::on_doubleSpinBox_3_valueChanged(double arg1)
{
  stiffness = arg1;
  std_msgs::msg::Float64MultiArray cal_data;
  cal_data.data.resize(4);
  cal_data.data[0] = slope;
  cal_data.data[1] = intercept;
  cal_data.data[2] = stiffness;
  cal_data.data[3] = COG_offset;


  calibrate_pub->publish(cal_data);
}

void SvayaUi::on_doubleSpinBox_4_valueChanged(double arg1)
{
  COG_offset = arg1;
  std_msgs::msg::Float64MultiArray cal_data;
  cal_data.data.resize(4);
  cal_data.data[0] = slope;
  cal_data.data[1] = intercept;
  cal_data.data[2] = stiffness;
  cal_data.data[3] = COG_offset;


  calibrate_pub->publish(cal_data);
}


void SvayaUi::on_doubleSpinBox_13_valueChanged(double arg1)
{
  goTo[0] = arg1*M_PI/180;
}


void SvayaUi::on_doubleSpinBox_14_valueChanged(double arg1)
{
//  goTo[1] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_19_valueChanged(double arg1)
{
//  goTo[2] = arg1*M_PI/180;
}


void SvayaUi::on_doubleSpinBox_37_valueChanged(double arg1)
{
//  goTo[3] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_34_valueChanged(double arg1)
{
//  goTo[4] = arg1*M_PI/180;
}

void SvayaUi::on_doubleSpinBox_38_valueChanged(double arg1)
{
//  goTo[5] = arg1*M_PI/180;
}
void SvayaUi::on_doubleSpinBox_5_valueChanged(double arg1)
{
    range_lower_limit[0] = arg1*M_PI/180;
}

void SvayaUi::on_doubleSpinBox_6_valueChanged(double arg1)
{
    range_upper_limit[0] = arg1*M_PI/180;
}

void SvayaUi::on_doubleSpinBox_11_valueChanged(double arg1)
{
//  range_lower_limit[1] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_7_valueChanged(double arg1)
{
//  range_upper_limit[1] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_15_valueChanged(double arg1)
{
//  range_lower_limit[2] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_16_valueChanged(double arg1)
{
//  range_upper_limit[2] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_32_valueChanged(double arg1)
{
//  range_lower_limit[3] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_29_valueChanged(double arg1)
{
//  range_upper_limit[3] = arg1*M_PI/180;
}


void SvayaUi::on_doubleSpinBox_41_valueChanged(double arg1)
{
//  range_lower_limit[4] = arg1*M_PI/180;
}

void SvayaUi::on_doubleSpinBox_25_valueChanged(double arg1)
{
//  range_upper_limit[4] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_23_valueChanged(double arg1)
{
//  range_lower_limit[5] = arg1*M_PI/180;

}

void SvayaUi::on_doubleSpinBox_30_valueChanged(double arg1)
{
//  range_upper_limit[5] = arg1*M_PI/180;

}

void SvayaUi::on_horizontalSlider_sliderMoved(int position)
{
  std_msgs::msg::Float64 sens;
  sens.data = position;
  gc_sens->publish(sens);
  ;
}
