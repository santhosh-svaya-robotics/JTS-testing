#include <svaya_trajectory_publisher/trajectory_publisher.hpp>
//#include <svaya_hw/svaya_hw.h>


namespace trajectory_publisher
{
TrajectoryPublisher::TrajectoryPublisher(sem_t *lock, int *exe_flag) : lock_(lock), exe_flag(exe_flag)
{
  node =  rclcpp::Node::make_shared("Trajectory_Publisher_node");
  joint_traj_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory",1000);
  controller_control = node->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");


  sleep(2);
  target_position.resize(numJoints,0);
  current_position.resize(numJoints,0);
  current_velocity.resize(numJoints,0);
  MAX_JOINT_MOTION_ACC.resize(numJoints,M_PI/2);
  MAX_JOINT_MOTION_VEL.resize(numJoints,M_PI/2);
  js.getJointStates(current_position);
  js.getJointVelStates(current_velocity);

}

TrajectoryPublisher::~TrajectoryPublisher()
{


}

void TrajectoryPublisher::SetMaxJointVel(std::vector<double> max_vel)
{
  MAX_JOINT_MOTION_VEL =max_vel;
}

void TrajectoryPublisher::SetMaxJointAcc(std::vector<double> max_acc)
{
  MAX_JOINT_MOTION_ACC = max_acc;
}

void TrajectoryPublisher::GetJointPos(std::vector<double> &pos)
{
  pos = current_position;
}

void TrajectoryPublisher::GetJointVel(std::vector<double> &vel)
{
    vel = current_velocity;
}

int TrajectoryPublisher::goToPos(std::vector<double> pos, double vel)
{
    std::vector<double> initial_position;
    std::vector<double> zero_vec;
    std::vector<double> curr_pos = current_position;
    std::cout<<curr_pos[0]<<" "<<curr_pos[1]<<" "<<curr_pos[2]<<std::endl;

    curr_pos.resize(numJoints,0);
    initial_position.resize(numJoints,0);
    zero_vec.resize(numJoints,0);

    std::vector<std::vector<double>> traj = {pos};
    std::vector<double> joint_vel(numJoints, vel); // Take from parameter


    initial_position = traj[0];
    double initial_time = jv_ja_to_time(joint_vel,MAX_JOINT_MOTION_ACC,curr_pos,initial_position);
    current_traj_time = initial_time;
//    point_to_point_feat_time(curr_pos, initial_position,initial_time,curr_pos,MAX_JOINT_MOTION_ACC);

   //safs

    trajectory_msgs::msg::JointTrajectory jointTraj;
    for (int i =0; i<numJoints;i++)
    {
      jointTraj.joint_names.push_back("joint"+std::to_string(i+1));
    }
    std::vector<double> zeroVec(numJoints, 0.0);
    jointTraj.points.resize(2);
    jointTraj.points[0].positions = curr_pos;
    jointTraj.points[0].velocities = zeroVec;
    jointTraj.points[0].accelerations = zeroVec;
    jointTraj.points[0].time_from_start = rclcpp::Duration::from_seconds(0);
    jointTraj.points[1].positions = initial_position;
    jointTraj.points[1].velocities = zeroVec;
    jointTraj.points[1].accelerations = zeroVec;
    jointTraj.points[1].time_from_start = rclcpp::Duration::from_seconds(initial_time);

    jointTraj.header.stamp = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(0.01);
//    std::cout<<"before pub \n";
    point_to_point_feat_time(curr_pos, initial_position,initial_time,curr_pos,MAX_JOINT_MOTION_ACC);

//    joint_traj_publisher->publish(jointTraj);
//    std::cout<<"after pub \n";


    if(*exe_flag == -1)
    {
        sem_wait(lock_);
        sem_post(lock_);

        curr_pos = current_position;
        initial_position = traj[0];
        std::vector<double> dec_max = {50*M_PI};

        double initial_time = jv_ja_to_time(joint_vel,dec_max,curr_pos,initial_position);
        current_traj_time = initial_time;
        point_to_point_feat_time(curr_pos, initial_position,initial_time,curr_pos,dec_max);

        if(*exe_flag == -2)
        {
            return 1;
        }

    }
    if(*exe_flag == -2)
    {
        return 1;
    }

}



int trajectory_publisher::TrajectoryPublisher::PublishTrajectory(double joint_vel_, std::vector<double> p1, std::vector<double> p2)
{

  std::vector<double> initial_position;
  std::vector<double> zero_vec;
  std::vector<double> curr_pos = current_position;


  curr_pos.resize(numJoints,0);
  initial_position.resize(numJoints,0);
  zero_vec.resize(numJoints,0);
  trajectory_msgs::msg::JointTrajectory jointTraj;

  std::vector<std::vector<double>> traj = {p1,p2};
  std::vector<double> joint_vel(numJoints, joint_vel_);

  initial_position = traj[0];
  double initial_time = jv_ja_to_time(joint_vel,MAX_JOINT_MOTION_ACC,curr_pos,initial_position);
  current_traj_time = initial_time;
  point_to_point_feat_time(curr_pos, initial_position,initial_time,curr_pos,MAX_JOINT_MOTION_ACC);


  for (int i =0; i<numJoints;i++)
  {
    jointTraj.joint_names.push_back("joint"+std::to_string(i+1));
  }

  int waypoint_num = traj.size();
  std::vector<double> segment_time;
  segment_time.resize(waypoint_num,5);
  for(int i = 0; i < waypoint_num; i++)
  {
      if(i == waypoint_num - 1)
      {
//          RCLCPP_INFO(node->get_logger(), "line 88");
          segment_time[i] = jv_ja_to_time(joint_vel, MAX_JOINT_MOTION_ACC, traj[i], traj[0]);
      }
      else
        segment_time[i] = jv_ja_to_time(joint_vel, MAX_JOINT_MOTION_ACC, traj[i], traj[i+1]);
  }


  int jmax = 100;
  bool not_completed = true;
  int nxt, curr;
  std::vector<double> next_point;
  double seg_time;
  std::vector<double> prev_pos;
  curr_pos = current_position;
  prev_pos = curr_pos;
  while(not_completed)
  {
    for(int i = 0; i < waypoint_num; i++)
    {
        curr = (i)%waypoint_num;
        nxt = (i+1)%waypoint_num;
        next_point = traj[nxt];
        seg_time = segment_time[curr];
        curr_pos = current_position;
        seg_time = jv_ja_to_time(joint_vel, MAX_JOINT_MOTION_ACC, curr_pos, next_point);
        point_to_point_feat_time(curr_pos, next_point,seg_time,prev_pos,MAX_JOINT_MOTION_ACC);

        if(*exe_flag == -1)
        {

            sem_wait(lock_);
            sem_post(lock_);

            curr_pos = current_position;
            initial_position = traj[nxt];
            double initial_time = jv_ja_to_time(joint_vel,MAX_JOINT_MOTION_ACC,curr_pos,initial_position);
            current_traj_time = initial_time;
//            point_to_point_feat_time(curr_pos, initial_position,initial_time,curr_pos,MAX_JOINT_MOTION_ACC);
             i = i-1;

            if(*exe_flag == -2)
            {
                return 1;
            }

        }
        if(*exe_flag == -2)
        {
            return 1;
        }
        curr_pos = prev_pos;
    }
  }
}

int TrajectoryPublisher::targetWatchDog()
{
  rclcpp::Time watchdog_start_time = rclcpp::Clock().now();
  while(*exe_flag == 1)
  {
    bool target_reached = true;
    for(int i=0; i<numJoints;i++)
    {
      if(fabs(target_position[i] - current_position[i]) > 0.05)
      {
        target_reached = false;
      }
    }
    if(target_reached == true &&  rclcpp::Clock().now() - watchdog_start_time > rclcpp::Duration::from_seconds(current_traj_time))
    {
//      svaya_hw::change_op_mode(*exe_flag, target_reached);
      return 1;
      break;
    }

    if(rclcpp::Clock().now() - watchdog_start_time > rclcpp::Duration::from_seconds(2*current_traj_time))
    {
      std::cout<<"\n\nxxxxxx Timeout: Trajectory Execution failed xxxxxx\n\n";
      return 0;
      break;
    }
  }
  if(*exe_flag == -1 || *exe_flag == -2)
  {
      RCLCPP_INFO(node->get_logger(),"Stopping");
      point_to_point_stop(current_position, current_velocity);

//      trajectory_msgs::msg::JointTrajectory jointTraj;
//      for (int i =0; i<numJoints;i++)
//      {
//        jointTraj.joint_names.push_back("joint"+std::to_string(i+2));
//      }

//      jointTraj.points.resize(2);
//      jointTraj.points[0].positions = curr_pos;
//      jointTraj.points[0].velocities = {0};
//      jointTraj.points[0].accelerations = {0};
//      jointTraj.points[0].time_from_start = rclcpp::Duration::from_seconds(0);
//      jointTraj.points[1].positions = initial_position;
//      jointTraj.points[1].velocities = {0};
//      jointTraj.points[1].accelerations = {0};
//      jointTraj.points[1].time_from_start = rclcpp::Duration::from_seconds(initial_time);

//      jointTraj.header.stamp = rclcpp::Clock().now();

//      joint_traj_publisher->publish(jointTraj);
    // Stop the motion
  }
  return 1;
}

double TrajectoryPublisher::jv_ja_to_time(std::vector<double> desired_joint_vel, std::vector<double> desired_joint_acc, std::vector<double> init_joint_val, std::vector<double> final_joint_val, std::vector<double> prev_joint_val, std::vector<double> prev_joint_vel, std::vector<double> prev_joint_acc)
{
  for(unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
  {
    if (final_joint_val[joint_ctr] < init_joint_val[joint_ctr])
    {
      desired_joint_vel[joint_ctr] = -fabs(desired_joint_vel[joint_ctr]);
      desired_joint_acc[joint_ctr] = -fabs(desired_joint_acc[joint_ctr]);
    }
    else
    {
      desired_joint_vel[joint_ctr] = fabs(desired_joint_vel[joint_ctr]);
      desired_joint_acc[joint_ctr] = fabs(desired_joint_acc[joint_ctr]);
    }

    if(!prev_joint_vel.empty())
    {
      if (init_joint_val[joint_ctr] < prev_joint_val[joint_ctr])
      {
        prev_joint_vel[joint_ctr] = -fabs(prev_joint_vel[joint_ctr]);
        prev_joint_acc[joint_ctr] = -fabs(prev_joint_acc[joint_ctr]);
      }
      else
      {
        prev_joint_vel[joint_ctr] = fabs(prev_joint_vel[joint_ctr]);
        prev_joint_acc[joint_ctr] = fabs(prev_joint_acc[joint_ctr]);
      }
    }

  }


  double slowest_joint_time = 0.0;//intitalize the segment time

  //iterating over all joints for computing slowest segment time
  for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
  {

    // if desired vel is greater than maximum joint vel limit
    if (fabs(desired_joint_vel[joint_ctr]) > MAX_JOINT_MOTION_VEL[joint_ctr])
    {
      desired_joint_vel[joint_ctr] = desired_joint_vel[joint_ctr]/fabs(desired_joint_vel[joint_ctr])*MAX_JOINT_MOTION_VEL[joint_ctr];
    }

    // if desired acc is greater than maximum joint acc limit
    if (fabs(desired_joint_acc[joint_ctr]) > MAX_JOINT_MOTION_ACC[joint_ctr])
    {
      desired_joint_acc[joint_ctr] = desired_joint_acc[joint_ctr]/fabs(desired_joint_acc[joint_ctr])*MAX_JOINT_MOTION_ACC[joint_ctr];
    }

    if(!prev_joint_acc.empty())
    {
      // if prev vel is greater than maximum joint vel limit
      if (fabs(prev_joint_vel[joint_ctr]) > MAX_JOINT_MOTION_VEL[joint_ctr])
      {
        prev_joint_vel[joint_ctr] = prev_joint_vel[joint_ctr]/fabs(prev_joint_vel[joint_ctr])*MAX_JOINT_MOTION_VEL[joint_ctr];
      }

      // if prev acc is greater than maximum joint acc limit
      if (fabs(prev_joint_acc[joint_ctr]) > MAX_JOINT_MOTION_ACC[joint_ctr])
      {
        prev_joint_acc[joint_ctr] = prev_joint_acc[joint_ctr]/fabs(prev_joint_acc[joint_ctr])*MAX_JOINT_MOTION_ACC[joint_ctr];
      }
    }

    // if initial joint val and final joint val are different
    if (init_joint_val[joint_ctr] != final_joint_val[joint_ctr])
    {

      double theta_diff = final_joint_val[joint_ctr] - init_joint_val[joint_ctr];
      double max_vel_possible = sqrt(2*desired_joint_acc[joint_ctr]*theta_diff/2);//derived from newton's equation of motion based on const. acc
      double prev_max_vel_possible;
      if(!prev_joint_acc.empty())
      {
        prev_max_vel_possible = sqrt(2*prev_joint_acc[joint_ctr]*theta_diff/2);//derived from newton's equation of motion based on const. acc
        max_vel_possible=std::min(max_vel_possible,prev_max_vel_possible);
        prev_max_vel_possible=max_vel_possible;
      }

      // if max_joint vel is greater than max joint vel possible in between 2 points
      if (fabs(desired_joint_vel[joint_ctr]) > max_vel_possible)
      {
        desired_joint_vel[joint_ctr] = desired_joint_vel[joint_ctr]/fabs(desired_joint_vel[joint_ctr])*max_vel_possible;
      }

      if(!prev_joint_acc.empty())
      {
        // if prev_max_joint vel is greater than max joint vel possible in between 2 points
        if (fabs(prev_joint_vel[joint_ctr]) > prev_max_vel_possible)
        {
          prev_joint_vel[joint_ctr] = prev_joint_vel[joint_ctr]/fabs(prev_joint_vel[joint_ctr])*prev_max_vel_possible;
        }
      }

      double time_acc, time_cruise, time_deacc, dist_acc, dist_deacc, total_time_;

      //computing acc, decc, cruise times assuming trapezoidal velocity profile
      if(prev_joint_acc.empty())
        time_acc = desired_joint_vel[joint_ctr]/desired_joint_acc[joint_ctr];
      else
        time_acc = prev_joint_vel[joint_ctr]/prev_joint_acc[joint_ctr];

      time_deacc = desired_joint_vel[joint_ctr]/desired_joint_acc[joint_ctr];

      if(prev_joint_acc.empty())
        dist_acc =  desired_joint_vel[joint_ctr]*time_acc/2;
      else
        dist_acc =  prev_joint_vel[joint_ctr]*time_acc/2;

      dist_deacc = desired_joint_vel[joint_ctr]*time_deacc/2;

      if(prev_joint_acc.empty())
        time_cruise = (theta_diff - dist_acc - dist_deacc)/desired_joint_vel[joint_ctr];
      else
        time_cruise = (theta_diff/2 - dist_deacc)/desired_joint_vel[joint_ctr] + (theta_diff/2 - dist_acc)/prev_joint_vel[joint_ctr];

      //      std::cout<<"**JVJA** TA : "<<time_acc<<" TD : "<<time_deacc<<" | TC : "<<time_cruise<<"\n";
      //      std::cout<<"**JVJA** DJV : "<<desired_joint_vel[joint_ctr]<<" JDA : "<<desired_joint_acc[joint_ctr]<<" | TD : "<<theta_diff<<"\n";

      if(fabs(time_cruise) < 1e-5){
        time_cruise = 0;
      }



      total_time_ = time_acc + time_cruise + time_deacc;

      //updating global slowest time
      if (total_time_ >= slowest_joint_time)
      {
        slowest_joint_time = total_time_;
      }

    }
    else //if both joint positions are same
    {
      desired_joint_vel[joint_ctr] = 0.0;
      desired_joint_acc[joint_ctr] = 0.0;
    }

  }



  if(std::isnan(slowest_joint_time))//NAN check
  {
    std::cout<<"-----------------NAN jvja\n";
    return slowest_joint_time;
  }
  //    std::cout<<"**JVJA** ST : "<<slowest_joint_time<<"\n";

  for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
  {
    //checking validity of slowest time based on joint acc and correcting it. See "Introduction to Robotics. Mechanics and Control by John J Craig"
    if ( 4*fabs(final_joint_val[joint_ctr] - init_joint_val[joint_ctr])/(slowest_joint_time*slowest_joint_time) > fabs(desired_joint_acc[joint_ctr]))
    {
      slowest_joint_time = sqrt(4*fabs(final_joint_val[joint_ctr] - init_joint_val[joint_ctr])/fabs(0.9*desired_joint_acc[joint_ctr]));
    }
  }

//  RCLCPP_INFO_STREAM(node->get_logger(),"Segment Time: "<<slowest_joint_time);

  return slowest_joint_time;//return the segment time
}

double TrajectoryPublisher::point_to_point_feat_time(std::vector<double> init_joint_val, std::vector<double> final_joint_val, double segment_time, std::vector<double> &last_joint_pos, std::vector<double> desired_joint_acc)
{

//    std::cout<<"init pos : "<<init_joint_val[0]<<"\n";
//    std::cout<<"final pos : "<<final_joint_val[0]<<"\n";
  trajectory_msgs::msg::JointTrajectory jointTraj;
  for (int i =0; i<numJoints;i++)
  {
    jointTraj.joint_names.push_back("joint"+std::to_string(i+1));
  }

  jointTraj.points.resize(2);
  std::vector<double> zeroVec(numJoints, 0.0);

  jointTraj.points[0].positions = init_joint_val;
  jointTraj.points[0].velocities = zeroVec;
  jointTraj.points[0].accelerations = zeroVec;
  jointTraj.points[0].time_from_start = rclcpp::Duration::from_seconds(0.001);
  jointTraj.points[1].positions = final_joint_val;
  jointTraj.points[1].velocities = zeroVec;
  jointTraj.points[1].accelerations = zeroVec;
  jointTraj.points[1].time_from_start = rclcpp::Duration::from_seconds(segment_time*1.2);

  current_traj_time = segment_time*1.2;
  target_position = final_joint_val;
  last_joint_pos = final_joint_val;
  jointTraj.header.stamp = rclcpp::Time(0);
  joint_traj_publisher->publish(jointTraj);
  targetWatchDog();
  jointTraj.points.clear();

  return 1;



  //determining sign of acc.
  for(unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
  {
    if (final_joint_val[joint_ctr] < init_joint_val[joint_ctr])
    {
      desired_joint_acc[joint_ctr] = -fabs(desired_joint_acc[joint_ctr]);
    }
    else
    {
      desired_joint_acc[joint_ctr] = fabs(desired_joint_acc[joint_ctr]);
    }
  }



  double slowest_acc_period = 0;//variable to store slowest joint time

  std::vector<double> acc_period;
  acc_period.resize(numJoints, 0.0);


  bool planning_done = false;
  bool increase_curr_seg_time = false;


  while(planning_done == false)
  {
    planning_done = true;
    if(increase_curr_seg_time == true)
    {
      segment_time = segment_time * 1.1;
    }
    increase_curr_seg_time = false;
    //computing slowest time for all joints. See "Introduction to Robotics. Mechanics and Control by John J Craig"
    for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
    {
      double square = desired_joint_acc[joint_ctr]*desired_joint_acc[joint_ctr]*segment_time*segment_time- 4*desired_joint_acc[joint_ctr]*(final_joint_val[joint_ctr] - init_joint_val[joint_ctr]);

      if(square < 0 && fabs(square)>1e-4)
      {
        segment_time = 1.1 * sqrt(4*fabs((final_joint_val[joint_ctr] - init_joint_val[joint_ctr])/desired_joint_acc[joint_ctr]));
        planning_done = false;
        break;
      }

      if(fabs(square)<1e-4)
      {
        square=0;
      }

      acc_period[joint_ctr] = segment_time/2 - sqrt(square)/fabs(2*desired_joint_acc[joint_ctr]);

      //storing global slowest time
      if (acc_period[joint_ctr] > slowest_acc_period)
      {
        slowest_acc_period = acc_period[joint_ctr];
      }

    }

    if(planning_done == false)
    {
      continue;
    }
    //Re-computing desired joint acc based on the global slowest time
    for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
    {
      if(fabs(final_joint_val[joint_ctr] - init_joint_val[joint_ctr]) < 1e-4)
      {
        desired_joint_acc[joint_ctr] = 0;
      }
      else
      {
        desired_joint_acc[joint_ctr] = (final_joint_val[joint_ctr] - init_joint_val[joint_ctr])/(segment_time*slowest_acc_period -slowest_acc_period*slowest_acc_period);
      }
      acc_period[joint_ctr] = slowest_acc_period;
    }

    for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
    {
      if(fabs(desired_joint_acc[joint_ctr]*slowest_acc_period) > MAX_JOINT_MOTION_VEL[joint_ctr])
      {
        planning_done = false;
        increase_curr_seg_time = true;
        break;
      }
    }
  }

  bool not_completed = true;
  bool first_cycle = true;

  double percentage_completed=1;
  bool head_traj_done=false;

  double time = 0;
  double traj_time=0;
  double rem_traj_time;
  //Loop for trajextory generation and execution

  //  std::cout<<"Segment time: "<<segment_time<<" Acceleration time: "<<acc_period[5]<<" Acceleration: "<<desired_joint_acc[5]<<"\n";
  while(not_completed)
  {
    time = time + 0.0025;//total trajectory time
    traj_time=traj_time+0.0025;//Trajectory time, for time from start filed in Trajectory container

    std::vector<double> joint_position_curr, joint_velocity_curr, joint_acceleration_curr;
    joint_position_curr.resize(numJoints);
    joint_velocity_curr.resize(numJoints);
    joint_acceleration_curr.resize(numJoints);

    //Determining the trajecory phase over all joints
    for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
    {

      if (time < acc_period[joint_ctr])//Trajectory generation for acc phase
      {
        //Derived from Newton's equations of motion
        joint_position_curr[joint_ctr] = init_joint_val[joint_ctr] + 0.5*desired_joint_acc[joint_ctr]*time*time;
        joint_velocity_curr[joint_ctr] = desired_joint_acc[joint_ctr]*time;
        joint_acceleration_curr[joint_ctr] = desired_joint_acc[joint_ctr];
      }
      else if(time < (segment_time-acc_period[joint_ctr]))//Trajectory generation for cruise phase
      {
        //Derived from Newton's equations of motion
        joint_position_curr[joint_ctr] = init_joint_val[joint_ctr] + 0.5*desired_joint_acc[joint_ctr]*acc_period[joint_ctr]*acc_period[joint_ctr] + desired_joint_acc[joint_ctr]*acc_period[joint_ctr]*(time - acc_period[joint_ctr]);
        joint_velocity_curr[joint_ctr] = desired_joint_acc[joint_ctr]*acc_period[joint_ctr];
        joint_acceleration_curr[joint_ctr] = 0;
      }
      else if (time < (segment_time))//Trajectory generation for decc phase
      {
        //Derived from Newton's equations of motion
        joint_position_curr[joint_ctr] = init_joint_val[joint_ctr] + 0.5*desired_joint_acc[joint_ctr]*acc_period[joint_ctr]*acc_period[joint_ctr] + desired_joint_acc[joint_ctr]*acc_period[joint_ctr]*(segment_time-2*acc_period[joint_ctr]) + desired_joint_acc[joint_ctr]*acc_period[joint_ctr]*(time - (segment_time-acc_period[joint_ctr])) - 0.5*desired_joint_acc[joint_ctr]*(time - (segment_time-acc_period[joint_ctr]))*(time - (segment_time-acc_period[joint_ctr]));
        joint_velocity_curr[joint_ctr] = desired_joint_acc[joint_ctr]*acc_period[joint_ctr] - desired_joint_acc[joint_ctr]*(time - (segment_time-acc_period[joint_ctr]));
        joint_acceleration_curr[joint_ctr] = -desired_joint_acc[joint_ctr];
      }
      else //Trajectory generation for trajectory completion
      {
        //Setting directly to target
        joint_position_curr[joint_ctr] = final_joint_val[joint_ctr];
        joint_velocity_curr[joint_ctr] = 0.0;
        joint_acceleration_curr[joint_ctr] = -desired_joint_acc[joint_ctr];
        not_completed = false;
      }
    }
    for(int x=0;x<numJoints;x++)
    {
      if(std::isnan(init_joint_val[x]))//nan test
      {
        std::cout<<"-------------------------NAN 1\n";
      }
    }
    //std::cout << "before setting prev point "<< std::endl;
    if (first_cycle == true)//add prev point data to the Trajectory container for continuity
    {
      first_cycle = false;
      trajectory_msgs::msg::JointTrajectoryPoint new_point;
      new_point.positions.resize(numJoints);
      new_point.positions = init_joint_val;
      new_point.velocities.resize(numJoints,0.0);
      new_point.accelerations.resize(numJoints);
      new_point.accelerations = desired_joint_acc;
      new_point.time_from_start = rclcpp::Duration::from_seconds(0);
      jointTraj.points.push_back(new_point);
    }
    //    std::cout << "after setting prev point "<< std::endl;
    if (fabs(time - segment_time) < 1e-5 || time > segment_time)//Traj time has ended. Set final values directly
    {
      for(unsigned int joint_ctr = 0; joint_ctr< numJoints; joint_ctr++)
      {
        joint_position_curr[joint_ctr] = final_joint_val[joint_ctr];
        joint_velocity_curr[joint_ctr] = 0.0;
        joint_acceleration_curr[joint_ctr] = joint_acceleration_curr[joint_ctr];
      }
      traj_time = segment_time;
      current_traj_time = traj_time;
      not_completed = false;
    }

    if (not_completed == false)//completed. Set target valuues and times directly
    {
      target_position = joint_position_curr;
      last_joint_pos = joint_position_curr;
    }

    //    std::cout<<"Pos: "<<joint_position_curr[5]<<" Vel: "<<joint_velocity_curr[5]<<"\n";

    //FIlling generated trajecotry point in to the trajectory container/
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    new_point.positions.resize(numJoints);
    new_point.positions = joint_position_curr;//point position
    new_point.velocities.resize(numJoints);
    new_point.velocities = joint_velocity_curr;//point vel
    new_point.accelerations.resize(numJoints);
    new_point.accelerations = joint_acceleration_curr;//point acc
    new_point.time_from_start = rclcpp::Duration::from_seconds(traj_time);//point time w.r.t start of trajectory container
    jointTraj.points.push_back(new_point);//Add point into trajectory container
  }
  jointTraj.header.stamp = rclcpp::Time(0);
  joint_traj_publisher->publish(jointTraj);
  targetWatchDog();
  jointTraj.points.clear();

  return 1;
}



int TrajectoryPublisher::point_to_point_stop(std::vector<double> curr_joint_pos, std::vector<double> curr_joint_vel)
{

  trajectory_msgs::msg::JointTrajectory jointTraj;
  rclcpp::Time time1 = rclcpp::Clock().now();
//  RCLCPP_INFO_STREAM(node->get_logger(),"recieved stop request : "<<rclcpp::Clock().now().seconds());


  for (int i =0; i<numJoints;i++)
  {
    jointTraj.joint_names.push_back("joint"+std::to_string(i+1));
  }


  double segment_time = 0;
  std::vector<double> desired_acc;// =  MAX_JOINT_MOTION_ACC;
  desired_acc.resize(numJoints, 5*M_PI);
  for (int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
  {
    if(fabs(curr_joint_vel[joint_ctr]) < 1e-4)
    {
       desired_acc[joint_ctr] = 0;
    }
    else if(curr_joint_vel[joint_ctr] > 0)
    {
      desired_acc[joint_ctr] = -1 * desired_acc[joint_ctr];
    }

    if(desired_acc[joint_ctr] != 0 && fabs(curr_joint_vel[joint_ctr]/desired_acc[joint_ctr]) > segment_time)
    {
      segment_time = fabs(curr_joint_vel[joint_ctr]/desired_acc[joint_ctr]);
    }
  }

//  for (int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
//  {
//    if(desired_acc[joint_ctr] != 0)
//    {
//      desired_acc[joint_ctr] = desired_acc[joint_ctr]/fabs(desired_acc[joint_ctr]) * fabs(desired_acc[joint_ctr]);
//    }
//  }

  bool not_completed = true;
  double time = 0;
  double dt = 0.025;

  std::vector<double> joint_position_curr, joint_velocity_curr, joint_acceleration_curr;
  joint_position_curr.resize(numJoints);
  joint_velocity_curr.resize(numJoints);
  joint_acceleration_curr.resize(numJoints);

  trajectory_msgs::msg::JointTrajectoryPoint new_point;
  new_point.positions.resize(numJoints);
  new_point.velocities.resize(numJoints);
  new_point.accelerations.resize(numJoints);

  while(not_completed)
  {
    for(int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
    {
      joint_position_curr[joint_ctr] = curr_joint_pos[joint_ctr] + curr_joint_vel[joint_ctr] * time + 0.5 * desired_acc[joint_ctr] * time * time;
      joint_velocity_curr[joint_ctr] = curr_joint_vel[joint_ctr] + desired_acc[joint_ctr] * time;
      joint_acceleration_curr[joint_ctr] = desired_acc[joint_ctr];
    }
//    RCLCPP_INFO_STREAM(node->get_logger(),"J1 pos: "<<joint_position_curr[0]<<"J1 vel: "<<joint_velocity_curr[0]);
    if(time  >= segment_time)
    {
      for(int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
      {
        joint_position_curr[joint_ctr] = curr_joint_pos[joint_ctr] + curr_joint_vel[joint_ctr] * segment_time + 0.5 * desired_acc[joint_ctr] * segment_time * segment_time;
        joint_velocity_curr[joint_ctr] = 0;
        joint_acceleration_curr[joint_ctr] = desired_acc[joint_ctr];
      }
    not_completed = false;
    }
    new_point.positions = joint_position_curr;//point position
    new_point.velocities = joint_velocity_curr;//point vel
    new_point.accelerations = joint_acceleration_curr;//point acc
    new_point.time_from_start = rclcpp::Duration::from_seconds(time );//point time w.r.t start of trajectory container
    jointTraj.points.push_back(new_point);

    time = time + dt;
  }
  std::vector<double> zeroVec(numJoints, 0.0);

  new_point.positions = joint_position_curr;//point position
  new_point.velocities = zeroVec;//point vel
  new_point.accelerations = zeroVec;//point acc
  new_point.time_from_start = rclcpp::Duration::from_seconds(time );//point time w.r.t start of trajectory container
  jointTraj.points.push_back(new_point);

  jointTraj.header.stamp = rclcpp::Time(0);//set time stamp for the trajectory container
  rclcpp::Time time2 = rclcpp::Clock().now();
  auto duration = time2 - time1;
  RCLCPP_INFO_STREAM(node->get_logger(),"publiching trajectory : "<<duration.seconds());

  joint_traj_publisher->publish(jointTraj);//Send trajectory

  return 1;
}



int TrajectoryPublisher::startController()
{
  auto switch_msg = std::make_shared<controller_manager_msgs::srv::SwitchController_Request>();
  switch_msg->strictness = 2;
  switch_msg->activate_controllers.push_back("joint_trajectory_controller");
  auto result_ = controller_control->async_send_request(switch_msg);
  if (rclcpp::spin_until_future_complete(node, result_) == rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cout<<"\n\n---------Started Joint Trajectory Controller---------\n\n";
    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/svaya_parameters");
    while (!parameter_client->wait_for_service())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    std::vector<rclcpp::Parameter> param;
    param.push_back(rclcpp::Parameter("controller_status", true));
    parameter_client->set_parameters(param);
    return true;

  }
  else
  {
    return false;
  }
}

int TrajectoryPublisher::stopController()
{
  auto switch_msg = std::make_shared<controller_manager_msgs::srv::SwitchController_Request>();
  switch_msg->strictness = 2;
  switch_msg->deactivate_controllers.push_back("joint_trajectory_controller");
  auto result_ = controller_control->async_send_request(switch_msg);
  if (rclcpp::spin_until_future_complete(node, result_) == rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cout<<"\n\n---------Stopped Joint Trajectory Controller---------\n\n";
    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/svaya_parameters");
    while (!parameter_client->wait_for_service())
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    std::vector<rclcpp::Parameter> param;
    param.push_back(rclcpp::Parameter("controller_status", false));
    parameter_client->set_parameters(param);
    return true;

  }
  else
  {
    return false;
  }
}

}
