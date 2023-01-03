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

void catch_franka_error(std::string where_from) {
    try{
        throw;
    }
    catch (franka::Exception const& e) {
        std::cout << where_from << " : " << e.what() << std::endl;
    }
}

void init() {
    try {
        franka::Robot robot_left(dualarm::left_ip);
        MotionGenerator motionGenerator_left(0.5, dualarm::qinit);
        robot_left.control(motionGenerator_left);
        franka::Gripper left_gripper(dualarm::left_ip);
        left_gripper.homing();
        left_gripper.move(0.08, 0.1);


        franka::Robot robot_right(dualarm::right_ip);
        MotionGenerator motionGenerator_right(0.5, dualarm::qinit);
        robot_left.control(motionGenerator_right);
        franka::Gripper right_gripper(dualarm::right_ip);
        right_gripper.homing();
        right_gripper.move(0.08, 0.1);
    } catch (...) {
        catch_franka_error("init");
    }
}

void grasp (std::string robot_ip) {
    try {
        franka::Gripper gripper(robot_ip);
        // assume homing already done?
        // gripper.homing();
        gripper.grasp(dualarm::grasp_width, 0.1, 80,0.005,0.005);
    } catch(...) {
        catch_franka_error("grasp");
    }
}

void release (std::string robot_ip) {
    try {
        franka::Gripper gripper(robot_ip);
        // assume homing already done?
        // gripper.homing();
        gripper.move(0.08, 0.1);
    } catch(...) {
        catch_franka_error("release");
    }
}

void move(std::string robot_ip, std::vector<std::array<double, 7>> traj) {
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
        catch_franka_error("grasp");
    }
}
