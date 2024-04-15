#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <array>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>

#include <Eigen/Dense>
#include "torq_predictor/torq_predictor.h"
using namespace std::chrono_literals;




void joint_state_callback_function(const std_msgs::msg::Float64MultiArray & msg){

  for(size_t j=0; j<3; j++){
    theta[j] = msg.data[j];

    theta_dot[j] = msg.data[j+1];
  }
  theta_prev = theta;
  theta_ddot = ( theta_dot - theta_dot_prev )/dt;    // calculate angular acceleration
  theta_dot_prev = theta_dot;


}

std::vector<Eigen::Matrix4d> forwardKinematics(const Eigen::VectorXd & theta,const double L[5]) {
std::vector<std::vector<double>> DH_params= {
  {theta[0], L[0], 0, -pi/2},
  {theta[1]+pi/2, 0, -L[1], 0},
  {theta[2]-pi/2, L[4], 0, pi/2},
  {theta[3], L[2], 0, -pi/2},
  {theta[4], 0, 0, pi/2},
  {theta[5], L[3], 0, 0}
  };
//  cout<<a[3];
  std::vector<Eigen::Matrix4d> Tr;
  for (int i = 0; i<6; i++){
    double theta1 = DH_params[i][0];
    double d = DH_params[i][1];
    double a = DH_params[i][2];
    double alpha = DH_params[i][3];
    Eigen::Matrix4d Mat;
    Mat << cos(theta1), -sin(theta1)*cos(alpha), sin(theta1)*sin(alpha), a*cos(theta1),
           sin(theta1), cos(theta1)*cos(alpha), -cos(theta1)*sin(alpha), a*sin(theta1),
           0, sin(alpha), cos(alpha), d,
           0,0,0,1;
    Tr.push_back(Mat);
   };
//         cout<<Tr[1][1][1];
   return Tr;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  curr_time = rclcpp::Clock().now();
  prev_time = rclcpp::Clock().now();
    std::cout<<"checkpoint 1"<<std::endl;

    node = rclcpp::Node::make_shared("torque_predictor_node");

    model_torq_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/svaya/model_torq",500);
    std_msgs::msg::Float64MultiArray model_torq;
    model_torq.data.resize(6);
    RCLCPP_INFO_STREAM(node->get_logger(),"Successfully started torque predictor");
    auto subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>("/svaya/joint_state",100,joint_state_callback_function);
    MOI_list = get_interia();
    while(rclcpp::ok()){
      curr_time = rclcpp::Clock().now();

      auto duration = curr_time - prev_time;
      delta_t = duration.seconds();
      if(delta_t > 0.01){
          delta_t = 0.01;
      }
      //RCLCPP_INFO_STREAM(node->get_logger(),"dt "<< delta_t);

      prev_time = curr_time;

      std::vector<Eigen::Matrix4d> fktr = forwardKinematics(theta, L);

      v_dot.col(0) << 0,0,9.81;

      for(size_t j=0; j<numOfJoints; ++j){
        // forward iteration

        Eigen::Matrix3d Rot = fktr[j].block(0,0,3,3).transpose();  // get rotation matrix
        w.col(j+1) = Rot*w.col(j) + Eigen::Vector3d(0, 0, theta_dot[j]); // get andular velocity of link j wrt base
        w_dot.col(j+1) = Rot*w_dot.col(j) + ( Rot*(w.col(j)) ).cross( Eigen::Vector3d(0, 0, theta_dot[j]) ) + Eigen::Vector3d(0, 0, theta_ddot[j]); // angular acceleration of each link

        Eigen::Vector3d s1=  w_dot.col(j).cross( fktr[j].block<3, 1>(0, 3) );

        v_dot.col(j+1) = Rot*(
          s1 +
          w.col(j).cross( (fktr[j].block<3, 1>(0, 3)) ) +
          v_dot.col(j)
        );                                              // linear acceleration

        Eigen::Vector3d s2 = w.col(j+1).cross(cs_to_com.col(j));
        v_dot_c.col(j+1) = w_dot.col(j+1).cross( cs_to_com.col(j) ) + w.col(j+1).cross(s2) + v_dot.col(j); // linear acceleration wrt com

        F.col(j+1) = -mass[j+1]*v_dot_c.col(j+1);
        v_dot_grav.col(j+1) = Rot*v_dot_grav.col(j);
        F_grav.col(j+1) = -mass[j+1]*v_dot_grav.col(j+1); // gravity force on a joint due to the corresponding link

        Eigen::VectorXd s3= MOI_list[j]*w_dot.col(j+1);
        N.col(j+1) = -( s3 + w.col(j+1).cross( MOI_list[j]*w.col(j+1) ) ); // torque on a joint due to acceleration


      }

      // backward iteration
      f.col(5) = F.col(6);
      n.col(5) = N.col(6) + cs_to_com.col(5).cross(F.col(6));

      Ijte[5] = MOI_list[5].array() + mass[6]*(Eigen::DiagonalMatrix<double, 3>(cs_to_com.col(5)).toDenseMatrix()).array().square(); // moment of inertia from j6 to end effector
      c[5] = damping_factor[5]*360;              // joint damping
//      th2_ddot.col(5) = Ijte[5].inverse()*(
//        n.col(5) +                                                     // external torque
//        stiffness[5]*(Eigen::Vector3d(0, 0, theta(5) - ajs[5]) ) + // restoring force
//        c[5]*( Eigen::Vector3d(0, 0, theta_dot[5]) - th2_dot.col(5))   // damping
//      );
      torq[5] = n(2,5);
      bm[5] = sqrt(n(0,5)*n(0,5) + n(1,5)*n(1,5));
      model_torq.data[5] = torq[5];

      Eigen::VectorXd s5= cs_to_com.col(5);   		// no idea what this is for
      for(int j = 4; j>=0; --j){

        Eigen::Matrix3d Rot = fktr[j].block(0,0,3,3);
        Eigen::Matrix<double, 3, 3> I_upper = Eigen::Matrix<double, 3, 3>::Zero();
        for(int l=5; l>j; --l){
          Eigen::MatrixXd transform = Eigen::MatrixXd::Identity(4, 4);
          for(int m=l; m>j; --m){
            transform = fktr[m-1]*transform;
          }
          s5 = (transform*(static_cast<void>(cs_to_com.col(l)),1)).block<3, 1>(0, 3);
          I_upper = transform.block(0,0,3,3)*MOI_list[l]   +
          mass[l+1]*(Eigen::DiagonalMatrix<double, 3>(s5[1]*s5[1] + s5[2]*s5[2], s5[0]*s5[0] + s5[2]*s5[2] , s5[0]*s5[0] + s5[1]*s5[1]).toDenseMatrix());
        }
        s5 = cs_to_com.col(j);
        Ijte[j] = MOI_list[j] + I_upper + mass[j+1]*(Eigen::DiagonalMatrix<double, 3>(s5[1]*s5[1] + s5[2]*s5[2], s5[0]*s5[0] + s5[2]*s5[2] , s5[0]*s5[0] + s5[1]*s5[1]).toDenseMatrix());
        f.col(j) = Rot*f.col(j+1) + F.col(j+1);
        Eigen::Vector3d T = fktr[j].block<3, 1>(0, 3);
        n.col(j) = N.col(j+1) + Rot*n.col(j+1) + cs_to_com.col(j).cross(F.col(j+1)) + T.cross(Rot*f.col(j+1));
        data1.col(j) =  cs_to_com.col(j).cross(F.col(j+1));
        c[j] = damping_factor[j]*360;
//        th2_ddot.col(j) = Ijte[j].inverse()*(
//          Rot*n.col(j) +
//          stiffness[j]*( Eigen::Vector3d( 0,0,theta[j]-ajs[j] ) ) +
//          c[j]*( Eigen::Vector3d(0,0,theta_dot[j] )- th2_dot.col(j) ) +
//          T.cross( Rot*f.col(j+1) ) +
//          cs_to_com.col(j).cross(F.col(j+1))
//        );

        torq[j] = n(2,j);
        bm[j] = sqrt(n(0,j)*n(0,j) + n(1,j)*n(1,j));
        model_torq.data[j] = torq[j];
      }
//      th2_dot = th2_dot + th2_ddot*dt;
//      ajs = ajs + th2_dot.row(2).transpose()*dt + 0.5*th2_ddot.row(2).transpose()*dt*dt;

//      delta_theta = theta - ajs;
//      for(int j=0; j<6; ++j){
//        R[1] = R0 + R0*strain_gauge_constant*(delta_theta[j]);
//        R[2] = R0 + R0*strain_gauge_constant*(-delta_theta[j]);
//        R[3] = R0 + R0*strain_gauge_constant*(delta_theta[j]);
//        R[4] = R0 + R0*strain_gauge_constant*(-delta_theta[j]);
//        R[5] = R0 + R0*strain_gauge_constant*(delta_theta[j]);
//        R[6] = R0 + R0*strain_gauge_constant*(-delta_theta[j]);
//        R[7] = R0 + R0*strain_gauge_constant*(delta_theta[j]);
//        R[8] = R0 + R0*strain_gauge_constant*(-delta_theta[j]);

//        double R1 = R[1]*R[7]/(R[1] + R[7]);
//        double R2 = R[2]*R[8]/(R[2] + R[8]);
//        double R3 = R[4]*R[6]/(R[4] + R[6]);
//        double R4 = R[5]*R[3]/(R[5] + R[3]);

//        jts_torq[j] = -(R1*R4-R2*R3)/((R1+R2)*(R3+R4))*97.84*8.1619;
//      }

      model_torq_pub->publish(model_torq);
      rclcpp::spin_some(node);
      rclcpp::sleep_for(1ms);
    }

return 0;
}


  /*




  for(size_t i=3; i<joint_space.rows(); ++i){
    theta = joint_space.row(i);
    theta_prev = joint_space.row(i-1);
    fktr = forwardKinematics(theta, L);                //calculate fk
    dor = ( theta - theta_prev ).array().sign();       // get direction of rotation
    theta_dot = ( theta - theta_prev )/dt;             // calculate angular speed
    theta_ddot = ( theta_dot - theta_dot_prev )/dt;    // calculate angular acceleration
    theta_dot_prev = theta_dot;
    for(size_t j=0; j<numOfJoints; j++){

//		std::cout<<"joint "<< j+1<<"\n\n"<<fktr[j]<<std::endl;
    }
    for(size_t j=0; j<numOfJoints; ++j){
      // forward iteration
      Eigen::Matrix3d Rot = fktr[j].block(0,0,3,3).transpose();  // get rotation matrix
//			std::cout<<"ROTATION MATRIX " <<j+1<<std::endl;
//			std::cout<<Rot<<"\n\n";
      w.col(j+1) = Rot*w.col(j) + Eigen::Vector3d(0, 0, theta_dot[j]); // get andular velocity of link j wrt base
      w_dot.col(j+1) = Rot*w_dot.col(j) + ( Rot*(w.col(j)) ).cross( Eigen::Vector3d(0, 0, theta_dot[j]) ) + Eigen::Vector3d(0, 0, theta_ddot[j]); // angular acceleration of each link

      Eigen::Vector3d s1=  w_dot.col(j).cross( fktr[j].block<3, 1>(0, 3) );

      v_dot.col(j+1) = Rot*(
        s1 +
        w.col(j).cross( (fktr[j].block<3, 1>(0, 3)) ) +
        v_dot.col(j)
      );                                              // linear acceleration

      Eigen::Vector3d s2 = w.col(j+1).cross(cs_to_com.col(j));
      v_dot_c.col(j+1) = w_dot.col(j+1).cross( cs_to_com.col(j) ) + w.col(j+1).cross(s2) + v_dot.col(j); // linear acceleration wrt com

      F.col(j+1) = -mass[j+1]*v_dot_c.col(j+1);
      v_dot_grav.col(j+1) = Rot*v_dot_grav.col(j);
      F_grav.col(j+1) = -mass[j+1]*v_dot_grav.col(j+1); // gravity force on a joint due to the corresponding link

      Eigen::VectorXd s3= MOI_list[j]*w_dot.col(j+1);
      N.col(j+1) = -( s3 + w.col(j+1).cross( MOI_list[j]*w.col(j+1) ) ); // torque on a joint due to acceleration
//			std::cout<<"joint "<<j+1<<" "<<N(0,j+1)<<" "<<N(1,j+1)<<" "<<N(2,j+1)<<std::endl;
//			std::cout<<"joint "<<j+1<<" "<<MOI_list[j];
    }
//					std::cout<<"F : \n"<<F<<std::endl;


    // backward iteration
    f.col(5) = F.col(6);
    n.col(5) = N.col(6) + cs_to_com.col(5).cross(F.col(6));

    Ijte[5] = MOI_list[5].array() + mass[6]*(Eigen::DiagonalMatrix<double, 3>(cs_to_com.col(5)).toDenseMatrix()).array().square(); // moment of inertia from j6 to end effector
    c[5] = damping_factor[5]*360;              // joint damping
    th2_ddot.col(5) = Ijte[5].inverse()*(
      n.col(5) +                                                     // external torque
      stiffness[5]*(Eigen::Vector3d(0, 0, joint_space(i,5) - ajs[5]) ) + // restoring force
      c[5]*( Eigen::Vector3d(0, 0, theta_dot[5]) - th2_dot.col(5))   // damping
    );
    torq[5] = n(2,5);
    bm[5] = sqrt(n(0,5)*n(0,5) + n(1,5)*n(1,5));
    Eigen::VectorXd s5= cs_to_com.col(5);   		// no idea what this is for
    for(int j = 4; j>=0; --j){
      Eigen::Matrix3d Rot = fktr[j].block(0,0,3,3);
      Eigen::Matrix<double, 3, 3> I_upper = Eigen::Matrix<double, 3, 3>::Zero();
      for(int l=5; l>j; --l){
        Eigen::MatrixXd transform = Eigen::MatrixXd::Identity(4, 4);
        for(int m=l; m>j; --m){
          transform = fktr[m-1]*transform;
        }
        s5 = (transform*(cs_to_com.col(l),1)).block<3, 1>(0, 3);
        I_upper = transform.block(0,0,3,3)*MOI_list[l]   +
        mass[l+1]*(Eigen::DiagonalMatrix<double, 3>(s5[1]*s5[1] + s5[2]*s5[2], s5[0]*s5[0] + s5[2]*s5[2] , s5[0]*s5[0] + s5[1]*s5[1]).toDenseMatrix());
      }
      s5 = cs_to_com.col(j);
      Ijte[j] = MOI_list[j] + I_upper + mass[j+1]*(Eigen::DiagonalMatrix<double, 3>(s5[1]*s5[1] + s5[2]*s5[2], s5[0]*s5[0] + s5[2]*s5[2] , s5[0]*s5[0] + s5[1]*s5[1]).toDenseMatrix());
      f.col(j) = Rot*f.col(j+1) + F.col(j+1);
      Eigen::Vector3d T = fktr[j].block<3, 1>(0, 3);
      n.col(j) = N.col(j+1) + Rot*n.col(j+1) + cs_to_com.col(j).cross(F.col(j+1)) + T.cross(Rot*f.col(j+1));
      data1.col(j) =  cs_to_com.col(j).cross(F.col(j+1));
      c[j] = damping_factor[j]*360;
      th2_ddot.col(j) = Ijte[j].inverse()*(
        Rot*n.col(j) +
        stiffness[j]*( Eigen::Vector3d( 0,0,joint_space(i,j)-ajs[j] ) ) +
        c[j]*( Eigen::Vector3d(0,0,theta_dot[j] )- th2_dot.col(j) ) +
        T.cross( Rot*f.col(j+1) ) +
        cs_to_com.col(j).cross(F.col(j+1))
      );

      torq[j] = n(2,j);
      bm[j] = sqrt(n(0,j)*n(0,j) + n(1,j)*n(1,j));
//					std::cout<<"checkpoint    " <<j<< std::endl;

    }
//		std::cout<<"f : \n"<<f<<std::endl;

    th2_dot = th2_dot + th2_ddot*dt;
    ajs = ajs + th2_dot.row(2).transpose()*dt + 0.5*th2_ddot.row(2).transpose()*dt*dt;
    TORQfile << torq.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",")) << std::endl;
    debug_data<< data1.col(0).transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ","))<<std::endl;
    delta_theta = theta - ajs;
    for(int j=0; j<6; ++j){
      R[1] = R0 + R0*strain_gauge_constant*(delta_theta[j]);
      R[2] = R0 + R0*strain_gauge_constant*(-delta_theta[j]);
      R[3] = R0 + R0*strain_gauge_constant*(delta_theta[j]);
      R[4] = R0 + R0*strain_gauge_constant*(-delta_theta[j]);
      R[5] = R0 + R0*strain_gauge_constant*(delta_theta[j]);
      R[6] = R0 + R0*strain_gauge_constant*(-delta_theta[j]);
      R[7] = R0 + R0*strain_gauge_constant*(delta_theta[j]);
      R[8] = R0 + R0*strain_gauge_constant*(-delta_theta[j]);

      double R1 = R[1]*R[7]/(R[1] + R[7]);
      double R2 = R[2]*R[8]/(R[2] + R[8]);
      double R3 = R[4]*R[6]/(R[4] + R[6]);
      double R4 = R[5]*R[3]/(R[5] + R[3]);

      jts_torq[j] = -(R1*R4-R2*R3)/((R1+R2)*(R3+R4))*97.84*8.1619;
    }
    JTS_TORQfile << jts_torq.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",")) << std::endl;
//		std::cout<<i<<std::endl;
  }
  TORQfile.close();
  JTS_TORQfile.close();
  debug_data.close();
  return 0;*/
//}
