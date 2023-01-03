#include <string>
#include <iostream>
#include <unistd.h>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/rate_limiting.h>
#include "common.cpp"

namespace dualarm{
  double grasp_width = 0.025;
  std::string left_ip = "192.168.2.107";
  std::string right_ip = "192.168.2.109";
  std::array<double, 7> qinit = {0.0, -0.7, 0, -1.2, 0, 1.9, M_PI_4};
}

void catch_franka_error(int &res, std::string where_from) {
    try{
        res = 1;
        throw;
    }
    catch (franka::Exception const& e) {
        std::cout << "exception! " << where_from << " : " << e.what() << std::endl;
    } catch(std::exception const& e) {
        std::cout  << "exception! " << where_from << " : " << e.what() << std::endl;
    } catch(...) {
        std::cout  << "exception! " << where_from << " : " << "unknown exception!" << std::endl;
    }
}

void init_one(std::string robot_ip, int &res) {
    try
    {
        franka::Robot robot(robot_ip);
        MotionGenerator motionGenerator(0.5, dualarm::qinit);
        robot.control(motionGenerator);
        franka::Gripper left_gripper(robot_ip);
        left_gripper.homing();
        left_gripper.move(0.08, 0.1);
    }
    catch (...)
    {
        catch_franka_error(res, "init");
    }
}

void grasp (std::string robot_ip, int &res) {
    try {
        franka::Gripper gripper(robot_ip);
        // assume homing already done?
        // gripper.homing();
        gripper.grasp(dualarm::grasp_width, 0.1, 80,0.005,0.005);
    } catch(...) {
        catch_franka_error(res, "grasp");
    }
}

void release (std::string robot_ip, int &res) {
    try {
        franka::Gripper gripper(robot_ip);
        // assume homing already done?
        // gripper.homing();
        gripper.move(0.08, 0.1);
    } catch(...) {
        catch_franka_error(res, "release");
    }
}

void move(std::string robot_ip, std::vector<std::array<double, 7>> traj, int &res) {
    try {
        franka::Robot robot(robot_ip);
        int time_step = 0;
        robot.control([&](const franka::RobotState &robot_state, franka::Duration period) -> franka::JointPositions {
            if (time_step >= traj.size())
            {
                franka::JointPositions output = traj[traj.size() - 1];
                return franka::MotionFinished(output);
            }
            else
            {
                std::array<double, 7> output = traj[time_step];
                time_step += int(1000 * period.toSec());
                return output;
            }
        });
    } catch(...) {
        catch_franka_error(res, "grasp");
    }
}

void get_conf(std::string robot_ip, std::array<double, 7> &q, int &res) {
    try {
        franka::Robot robot(robot_ip);
        franka::RobotState state = robot.readOnce();
        for (int i = 0; i <= 7; i ++) {
            q[i] = state.q[i];
        }
    } catch(...) {
        catch_franka_error(res, "grasp");
    }
}
void get_ee_in_base(std::string robot_ip, std::array<double, 16> &X, int &res) {
    try {
        franka::Robot robot(robot_ip);
        franka::RobotState state = robot.readOnce();
        for (int i = 0; i <= 16; i ++) {
            X[i] = state.O_T_EE[i];
        }
    } catch(...) {
        catch_franka_error(res, "grasp");
    }
}