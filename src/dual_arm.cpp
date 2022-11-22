// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
//#include "tests/test_common.h"

#include <thread>
#include "common.h"
#include "logger.h"

#include <fstream>
#include <franka/rate_limiting.h>

namespace dualarm{
  std::vector<std::array<double, 7>> left_arm_traj; 
  std::vector<std::array<double, 7>> right_arm_traj; 
  int left_time_step = 0; 
  int right_time_step = 0; 
}
const double print_rate = 10.0;
std::atomic_bool left_running{true};
std::atomic_bool right_running{true};

// franka::JointPositions left_joint_pos_motion_generator(const franka::RobotState&robot_state, franka::Duration period, int index){
  
//   franka::JointPositions output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
//   if(dualarm::left_time_step + int(1000*period.toSec())>= dualarm::left_arm_traj.size()){
//     return franka::MotionFinished(output);
//     //output = dualarm::left_arm_traj[dualarm::left_time_step];
//   }
//   else{
//     dualarm::left_time_step += int(1000*period.toSec());
//     output = dualarm::left_arm_traj[dualarm::left_time_step];
//     dualarm::left_time_step++;
//   }
//   return output;   
// }


// franka::JointPositions right_joint_pos_motion_generator(const franka::RobotState&robot_state, franka::Duration period){                                   
//   franka::JointPositions output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
//   if(dualarm::right_time_step + int(1000*period.toSec())>= dualarm::right_arm_traj.size()){
//     return franka::MotionFinished(output);
//     //output = dualarm::right_arm_traj[dualarm::right_time_step];
//   }
//   else{
//     dualarm::right_time_step += int(1000*period.toSec());
//     output = dualarm::right_arm_traj[dualarm::right_time_step];
//     //dualarm::left_time_step++;
//   }
//   return output;   
// }




std::vector<std::array<double, 7>> parse_file(std::string file_name){

  std::vector<std::array<double, 7>> output; 
  std::ifstream file(file_name);
  std::string str;

  bool first = true; 
  while(std::getline(file,str)){
    if(first){
      first = false; 
      continue;
    }
    if(!first){
      std::array<double, 7> j_position = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; 
      int i = 0; 
      std::stringstream ss(str);
      while (ss.good()) {
        std::string substr;
        getline(ss, substr, ',');
        j_position[i] = std::stof(substr);
        i++;
      }
      // std::cout<<j_position[0]<<","<<j_position[1]<<","<<j_position[2]<<","<<j_position[3]<<","<<j_position[4]<<","<<j_position[5]<<","<<j_position[6]<<"\n";
      output.push_back(j_position);
    }
  }  
  //std::cout<<"length of traj"<<output.size()<<"\n";
  return output;
}

void run(){
    franka::Robot robot_right("192.168.2.109");
    robot_right.control(  [&](const franka::RobotState&robot_state, franka::Duration period) -> franka::JointPositions {     
          if (print_data_right.mutex.try_lock()) {
            print_data_right.has_data = true;
            print_data_right.robot_state = robot_state;
            print_data_right.mutex.unlock();
          }  
        franka::JointPositions output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        if(dualarm::right_time_step >= dualarm::right_arm_traj.size()){
          right_running = false; 
          return franka::MotionFinished(output);
        }
        else{
          dualarm::right_time_step += int(1000*period.toSec());
          output = dualarm::right_arm_traj[dualarm::right_time_step];
        }
        return output;          
        });
}

int main(int argc, char** argv) {

  std::string right_traj_file_name = "/home/pairlab/franka_dual_arm/traj_right.txt";
  std::string left_traj_file_name = "/home/pairlab/franka_dual_arm/traj_left.txt";
  dualarm::right_arm_traj =  parse_file(right_traj_file_name);
  dualarm::left_arm_traj =  parse_file(left_traj_file_name);



  try {
    franka::Robot robot_left("192.168.1.107");
    
    // std::cout << "moving robot to default position..." << std::endl;
     std::array<double, 7> qRest = dualarm::left_arm_traj.front();
    //std::array<double, 7> qRest = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motionGenerator(0.5, qRest);
    robot_left.control(motionGenerator);

    franka::Robot robot_right("192.168.2.109");
    qRest = dualarm::right_arm_traj.front();
    MotionGenerator motionGenerator_(0.2, qRest);
    robot_right.control(motionGenerator_);

    std::string file_name_left = "/home/pairlab/franka_dual_arm/log.txt";  
    std::string file_name_right = "/home/pairlab/franka_dual_arm/log.txt";  

    bool log_joint_space=true;
    std::thread print_thread_left = std::thread(log_data, std::cref(print_rate), std::ref(print_data_left), std::ref(left_running), std::ref(file_name_left), std::cref(log_joint_space));
    std::thread print_thread_right= std::thread(log_data, std::cref(print_rate), std::ref(print_data_right), std::ref(right_running), std::ref(file_name_right), std::cref(log_joint_space));

    std::thread t1(&run);
    //robot_left.control(left_joint_pos_motion_generator);
    robot_left.control(  [&](const franka::RobotState&robot_state, franka::Duration period) -> franka::JointPositions {     
          if (print_data_left.mutex.try_lock()) {
            print_data_left.has_data = true;
            print_data_left.robot_state = robot_state;
            print_data_left.mutex.unlock();
          }  
        franka::JointPositions output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        if(dualarm::left_time_step >= dualarm::left_arm_traj.size()){
          left_running = false; 
          return franka::MotionFinished(output);
        }
        else{
          dualarm::left_time_step += int(1000*period.toSec());
          output = dualarm::left_arm_traj[dualarm::left_time_step];
        }
        return output;          
        });    
    t1.join();

    if (print_thread_left.joinable()) {
      print_thread_left.join();
    }     
    if (print_thread_right.joinable()) {
      print_thread_right.join();
    }     

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
