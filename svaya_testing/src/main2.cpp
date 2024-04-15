#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <strstream>
#include <svaya_trajectory_publisher/trajectory_publisher.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

double numJoints = 6;

std::vector<double> joint_limits_upper = {0.9*M_PI,0.9*M_PI, 9*M_PI, 0.9*M_PI,0.9*M_PI,0.9*M_PI};
std::vector<double> joint_limits_lower = {-0.9*M_PI, -0.9*M_PI, -9*M_PI, -0.9*M_PI, -0.9*M_PI, -0.9*M_PI};


std::vector<double> MAX_JOINT_MOTION_VEL = {M_PI/3,M_PI/3, M_PI/3, M_PI/3,M_PI/3,M_PI/3};
std::vector<double> MAX_JOINT_MOTION_ACC = {M_PI/3,M_PI/3, M_PI/3, M_PI/3,M_PI/3,M_PI/3};

std::vector<std::vector<double>> traj;

int ValidateTrajectory()
{
  std::vector<double> jpos;

  //Checking time-stamps
  for (int i=0;i<traj.size();i++)
  {
    if(traj[i].size()-1 == numJoints)
    {
      jpos.resize(traj[i].size()-1);
    }
    else
    {
      std::cout<<"\n\n------ERROR: Invalid number of joints at time stamp "<<traj[i][0]<<" ------\n\n";
      return 0;
    }

    for(int jnt_cnt=0;jnt_cnt<jpos.size();jnt_cnt++)
    {
      jpos[jnt_cnt] = traj[i][jnt_cnt+1];
    }

    if(i==0)
    {
      if(traj[i][0] == 0)
      {
        std::cout<<"\n\n------ERROR: First time stamp should not be 0------\n\n";
        return 0;
      }
    }
    else
    {
      if(traj[i][0] - traj[i-1][0] <= 0)
      {
        std::cout<<"\n\n------ERROR: Time stamps should be strictly increasing: "<<i-1<<" "<<i<<" ------\n\n";
        return 0;

      }
    }

    for (int j=0; j<jpos.size();j++)
    {
      if(jpos[j] > joint_limits_upper[j] || jpos[j] < joint_limits_lower[j])
      {
        std::cout<<"\n\n------ERROR: Joint Limits Exceeding, Check your Trajectory at time stamp "<<traj[i][0]<<" , joint "<<j + 1<<" "<<jpos[j]<<" "<<joint_limits_upper[j]<<" "<<joint_limits_lower[j] <<" ------\n\n";
        return 0;
      }
    }


  }
  return 1;
}

}

