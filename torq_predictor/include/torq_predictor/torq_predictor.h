#pragma once
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <fstream>
#include <std_msgs/msg/float64.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#ifndef TORQ_PREDICTOR
#define TORQ_PREDICTOR
int check = 10;
const int numOfJoints = 6;
using Matrix = std::array<std::array<double, 3>, 3>;
using Point = std::array<double, 3>;
rclcpp::Node::SharedPtr node;

std::vector<Eigen::Matrix3d> get_interia()
{
    Eigen::Matrix3d MOIj1, MOIj2, MOIj3, MOIj4, MOIj5, MOIj6;
    MOIj1 <<  3.1374214e+04,  1.3005871e+01,  1.0455330e+02,
            1.3005871e+01,  3.2600136e+04,  1.7757511e+03,
            1.0455330e+02,  1.7757511e+03,  2.6991760e+04;
    MOIj1 = MOIj1/1000000;

  MOIj2 <<  5.7233482e+04, -1.2356263e+02,  2.5976367e+04,
          -1.2356263e+02,  1.6456321e+06,  1.9864331e+02,
           2.5976367e+04,  1.9864331e+02,  1.6344083e+06;
  MOIj2 = MOIj2/1000000;


  MOIj3 << 7.0427173e+03,  3.8965521e+00, -2.4267305e+01,
            3.8965521e+00,  4.8971697e+03, -8.0600743e+02,
            -2.4267305e+01, -8.0600743e+02,  5.8659630e+03;
  MOIj3 = MOIj3/1000000;


  MOIj4 << 1.1946489e+05,  7.0783910e+00,  3.4733165e+01,
            7.0783910e+00,  9.9822804e+03, -2.0685176e+04,
            3.4733165e+01, -2.0685176e+04,  1.1374279e+05;
  MOIj4 = MOIj4/1000000;

  MOIj5 <<  3.4625575e+03, -2.6772353e+00, -2.5952455e+01,
            -2.6772353e+00,  3.5486298e+03, -1.2281511e+02,
            -2.5952455e+01, -1.2281511e+02,  2.4609147e+03;
  MOIj5 = MOIj5/1000000;

  MOIj6 <<  6.0860720e+02,  1.8916742e-01, -1.3739869e-01,
            1.8916742e-01,  6.0373237e+02,  1.8848745e-02,
            -1.3739869e-01,  1.8848745e-02,  7.3161655e+02;
  MOIj6 = MOIj6/1000000;

  std::vector<Eigen::Matrix3d> interia_mat;

  interia_mat.push_back(MOIj1);
  interia_mat.push_back(MOIj2);
  interia_mat.push_back(MOIj3);
  interia_mat.push_back(MOIj4);
  interia_mat.push_back(MOIj5);
  interia_mat.push_back(MOIj6);

  return interia_mat;
}

using ArrayOfMatrices = std::array<Matrix,numOfJoints>;
std::vector<Eigen::Matrix3d> MOI_list;

//// Print the retrieved matrix

std::array<Eigen::Matrix3d, numOfJoints> Ijte; // moment of inertia from joint to end effector
std::array<Point, numOfJoints+1> COG_list = {{
  {3.8768e-24,	-0.033862,	0.025345},
  {0.0001288,	0.0051491,	0.1336},
  {-0.00013142,	0.16434,	0.44419},
  {0.00015504,	0.017475,	0.86664},
  {-0.00016591,	0.08478,	1.3635},
  {0.00019096,	0.0029037,	1.4933},
  {-3.6864e-05,	0.0001093,	1.5782}
  }};

static const Eigen::Matrix<double, 3,6> cs_to_com = [] {
    Eigen::Matrix<double, 3,6> mat;
  mat << 1.2880e-04,  -1.3142e-04,  -1.6125e-02,  -1.6591e-04,   1.9097e-04,  -3.6864e-05,
         5.1491e-03,  -2.7060e-01,  -1.5504e-04,  -3.4200e-03,  -1.9860e-02,  -8.8091e-02,
         -3.9991e-02,   1.6434e-01,   1.7475e-02,   5.1295e-01,  -8.5296e-02,   3.0475e-01;
  return mat;
  }();

std::array<double, numOfJoints+1> mass = {{
1.086,	8.6664,	16.752,	2.9073,	3.7003,	2.3603,	0.6
}};

double tool_offset = 0.2;
double payload = 10.0;
double dt = 0.001;
static const Eigen::Vector<double, 6> damping_factor = []{
    Eigen::Vector<double, 6> vec;
    vec<<1,1,1,0.2,0.05,0.02;
    return vec;
  }();static const Eigen::Vector<double, 6> stiffness = []{
    Eigen::Vector<double, 6> vec;
    vec<<1600, 1600, 1000, 200, 100, 100;
    return vec;
  }();
static const double R0 = 350.0;
static const double strain_gauge_constant = 2;

Eigen::VectorXd theta(6);
Eigen::VectorXd ajs(6);
Eigen::VectorXd theta_prev(6);
Eigen::VectorXd theta_dot = Eigen::VectorXd::Zero(6);
Eigen::VectorXd theta_dot_prev = Eigen::VectorXd::Zero(6);
Eigen::VectorXd theta_ddot = Eigen::VectorXd::Zero(6);
Eigen::VectorXd delta_theta = Eigen::VectorXd::Zero(6);

Eigen::VectorXd bending_moment = Eigen::VectorXd::Zero(6);
Eigen::VectorXd R = Eigen::VectorXd::Constant(9, R0);
Eigen::VectorXd dor;
Eigen::Matrix<double, 3, 7> w = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> w_dot = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> v = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> v_dot = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> v_dot_c = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> v_dot_grav = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> F = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> F_grav = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> N = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> f = Eigen::Matrix<double, 3, 7>::Zero();
Eigen::Matrix<double, 3, 7> n = Eigen::Matrix<double, 3, 7>::Zero();
    Eigen::Matrix<double, 3, 7> data1 = Eigen::Matrix<double, 3, 7>::Zero();

Eigen::VectorXd torq = Eigen::VectorXd::Zero(6);
Eigen::VectorXd bm = Eigen::VectorXd::Zero(6);
Eigen::VectorXd jts_torq = Eigen::VectorXd::Zero(6);
Eigen::Matrix<double, 3, 6> th2 = Eigen::Matrix<double, 3, 6>::Zero();

Eigen::Matrix<double, 3, 6> th2_dot = Eigen::Matrix<double, 3, 6>::Zero();
Eigen::Matrix<double, 3, 6> th2_ddot = Eigen::Matrix<double, 3, 6>::Zero();

Eigen::Matrix<double, 1,6> TORQ = Eigen::Matrix<double, 1,6>::Zero();
Eigen::Matrix<double, 1,6> BM = Eigen::Matrix<double, 1,6>::Zero();
Eigen::Vector<double, 6> c;



rclcpp::Time curr_time;
rclcpp::Time prev_time;
double delta_t = 0.0;
const double pi = M_PI;

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr model_torq_pub;


//theta << 0, 1.57, 0, 0, 0, 0 ;
const double L[5] = {0.17359, 0.67692, 0.62294, 0.13915, 0.0882};   // Link length

#endif // FORWARDKINEMATICS_H
