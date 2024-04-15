#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include<sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>
#include <fstream>
#include <cmath>

using namespace std::chrono_literals;

double th;
double th_dot;
double th_ddot;
double th_dot_fil;
double th_dot_prev = 0;
double th_fil = 0;
double th_prev = 0;


double th2;
double th2_dot;
double th2_ddot;

double stiffness;
double strain_gauge_factor;
double damping_coeff;
double link_length;
double mass;
double inertia;
double g = 9.81;
double f;
double th1_prev = 0.0;
double T_set = 0.02;
double th1 = 0.0;
double th1_dot;
double torq = 0.0;
double torq_prev = 0.0;
double torq_fil = 0.0;
const int len = 20;
double v_buffer[len];
double th1_buffer[len];
double sensor_torq;
rclcpp::Time curr_time;
rclcpp::Time prev_time;


double delta_t = 0;

rclcpp::Node::SharedPtr node;

bool initialised = false;

void callback_func(const std_msgs::msg::Float64MultiArray & msg)
{
    
    curr_time = rclcpp::Clock().now();

    auto duration = curr_time - prev_time;
    delta_t = duration.seconds();
    prev_time = curr_time;
    
    th = msg.data[0];
    th_dot = msg.data[1];
    th_fil = delta_t*th/T_set + (T_set - delta_t)*th_prev/T_set;
    th_dot_fil = delta_t*th_dot/T_set + (T_set - delta_t)*th_dot_prev/T_set;
    th_dot_prev = th_dot_fil;
    th_prev = th_fil;
    th_ddot = 0;
//    for (int i = 0; i<len-1; i++){
//        v_buffer[i] = v_buffer[i+1];
//        th1_buffer[i] = th1_buffer[i+1];
//    }

    v_buffer[len-1] = th_dot;
    th1_buffer[len-1] = th;
    if(initialised == false)
    {
      th2 = th;
      th1 = th;
      th1_dot = th_dot;
      th2_dot = th_dot;
      th2_ddot = th_ddot;
//      for (int i = 0; i<len; i++){
//          v_buffer[i] = th_dot;
//          th1_buffer[i] = th;
//      }
      initialised = true;
    }

    damping_coeff = sqrt(4.0*stiffness*f*inertia)*1.4753;
//    std::cout<<"damping" <<damping_coeff<<"\n";
    th2_ddot = (mass*g*link_length*sin(th2) + stiffness *f* (th_fil - th2) + damping_coeff * (th_dot_fil - th2_dot) )/inertia;

    th2_dot = th2_dot +  th2_ddot * delta_t;
    th2 = th2 + th2_dot * delta_t + 0.5*th2_ddot*delta_t*delta_t;//+ 0.5 * th2_ddot * delta_t * delta_t;
//    th1_prev = th1;
 //   std::cout<<th2<<" \n";

//    RCLCPP_INFO_STREAM(node->get_logger(),"Delta_t: "<<delta_t<<"\n");
//    std::cout<<"theta 1: "<<sin(th1)<<"\n";

}

void read_parameters()
{
std::cout<<"Reading Parameters\n";

        stiffness = node->get_parameter("stiffness").as_double();
//        damping_coeff = node->get_parameter("damping_coeff").as_double();

        strain_gauge_factor = node->get_parameter("strain_gauge_factor").as_double();

        link_length = node->get_parameter("link_length").as_double();
        mass = node->get_parameter("mass").as_double();
        f = node->get_parameter("f").as_double();

inertia = mass*link_length*link_length + 2.6*0.22*0.16/3;

std::cout<<"Reading Parameters Done\n";
}
void sensor_callback(const std_msgs::msg::Float64 & msg)
{
    sensor_torq = msg.data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);


   curr_time = rclcpp::Clock().now();
   prev_time = rclcpp::Clock().now();
  //std::cout<<"creating node ... \n";

   node = rclcpp::Node::make_shared("model_torq_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//     std::cout<<"125\n";
   read_parameters();

//     std::cout<<"128\n";
  auto model_torq_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/model_torq",100);
  auto subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/joint_state",100,callback_func);
  auto sensor_subs = node->create_subscription<std_msgs::msg::Float64>("/svaya/torq_sensor",100, sensor_callback);
  auto sensor_pub = node->create_publisher<std_msgs::msg::Float64>("/svaya/sensor_rep",100);
  auto model_torq_pub1 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/th2",10);

//    std::cout<<"134\n";

  std::fstream fout;
  fout.open("output.csv", std::ios::out);

  std::vector<double> R0;
  R0.resize(8,350);
  std::vector<double> R;
  R.resize(8,350);
  double R1,R2,R3,R4;
  std_msgs::msg::Float64MultiArray Torq;

  std_msgs::msg::Float64MultiArray th2_;
  std_msgs::msg::Float64 sensor_rep;
  th2_.data.resize(4);
  Torq.data.resize(2);

//  std::cout<<"147\n";
  while(rclcpp::ok())
  {
//        std::cout<<"150\n";
    double delta_th = th_fil - th2;
    R[0] = R0[0] + R0[0]*strain_gauge_factor*( delta_th  );
    R[1] = R0[0] + R0[0]*strain_gauge_factor*(-delta_th);
    R[2] = R0[0] + R0[0]*strain_gauge_factor*(delta_th);
    R[3] = R0[0] + R0[0]*strain_gauge_factor*( -delta_th );
    R[4] = R0[0] + R0[0]*strain_gauge_factor*( delta_th);
    R[5] = R0[0] + R0[0]*strain_gauge_factor*( -delta_th );
    R[6] = R0[0] + R0[0]*strain_gauge_factor*( delta_th );
    R[7] = R0[7] + R0[0]*strain_gauge_factor*( -delta_th );
    R1 = R[0]*R[6]/(R[0] + R[6]);
    R2 = R[1]*R[7]/(R[1] + R[7]);
    R3 = R[3]*R[5]/(R[3] + R[5]);
    R4 = R[4]*R[2]/(R[4] + R[2]);

    torq = (-978.8766*(R1*R4-R2*R3)/((R1+R2)*(R3+R4))*f) ;
    torq_fil = delta_t*torq/T_set + (T_set-delta_t)*torq_prev/T_set;
    Torq.data[1] = torq;
    Torq.data[0] = torq_fil;
    torq_prev = torq_fil;

//    std::cout<<"Delta theta: "<<delta_th<<"\n";
//    std::cout<<"Eq resistance: "<<(R1*R4-R2*R3)/((R1+R2)*(R3+R4))<<"\n";

//    std::cout<<"f "<<f<<"\n";

    if(fabs(Torq.data[0]) > 100)
    {
        if(Torq.data[0] > 0)
        {
            Torq.data[0] = 100;
        }
        else
        {
            Torq.data[0] = -100;

        }
    }

    th2_.data[0] = th2;
    th2_.data[1] = th2_dot;
    th2_.data[2] = th2_ddot;
    th2_.data[3] = delta_th;
    sensor_rep.data = sensor_torq;


    model_torq_pub->publish(Torq);
    model_torq_pub1->publish(th2_);
    sensor_pub->publish(sensor_rep);

fout<<th << "," << th_dot << "," <<torq<< "," << sensor_torq<<  ","<< delta_t<< "\n";

rclcpp::spin_some(node);

    rclcpp::sleep_for(1ms);
//      std::cout<<"194\n";
  }

  fout.close();



return 0;


}
