#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <string>
#include <iostream>
#include <unistd.h>
#include "commands.cpp"

std::vector<double> vector_from_message(zmq::message_t &msg) {
    int numValues = msg.size() / sizeof(double);
    std::vector<double> data;
    for(int i = 0; i < numValues; i++) {
        data.push_back(*reinterpret_cast<double*>(msg.data()+i*sizeof(double)));
    }
    return data;
}

void api_move_one (std::string arm, std::vector<double> traj) {
    // the traj vector contains a flattened joint space
    // trajectory sampled at 1Khz
    // arm is one of ["left_panda", "right_panda"]

    // we want to move the arm along the desired trajectory
    // before returning to the caller
    std::cout << "move_one " << arm << " ";

    std::string robot_ip;
    if (arm == "left_panda")
        robot_ip = dualarm::left_ip;
    else
        robot_ip = dualarm::right_ip;

    std::vector<std::array<double, 7>> *traj_mat = reinterpret_cast<std::vector<std::array<double, 7>> *>(&traj);

    std::thread move_thread = std::thread(move, std::cref(robot_ip), std::cref(*traj_mat));
    move_thread.join();
    
    std::cout << "done move_one " << arm << " ";

}
void api_move_both (std::vector<double> left_traj, std::vector<double> right_traj) {
    // this may not be a good impl because there's no explicit sync.
    // if so, we switch to a single time keeping mechanism
    std::cout << "move_both " << " ";
    std::vector<std::array<double, 7>> *traj_mat_left = reinterpret_cast<std::vector<std::array<double, 7>> *>(&left_traj);
    std::vector<std::array<double, 7>> *traj_mat_right = reinterpret_cast<std::vector<std::array<double, 7>> *>(&right_traj);

    std::thread gripper_thread_left = std::thread(move, std::cref(dualarm::left_ip), std::cref(*traj_mat_left));
    std::thread gripper_thread_right = std::thread(move, std::cref(dualarm::right_ip), std::cref(*traj_mat_right));
    gripper_thread_left.join();
    gripper_thread_right.join();

    std::cout << "done move_both " << " ";
}

void api_grasp_one (std::string arm) {
    std::cout << "grasp_one " << arm << " ";

    std::string robot_ip;
    if (arm == "left_panda")
        robot_ip = dualarm::left_ip;
    else
        robot_ip = dualarm::right_ip;

    std::thread gripper_thread = std::thread(grasp, std::cref(robot_ip));
    gripper_thread.join();

    std::cout << "done grasp_one " << arm << " ";
}

void api_release_one (std::string arm) {
    std::cout << "release_one " << arm << " ";

    std::string robot_ip;
    if (arm == "left_panda")
        robot_ip = dualarm::left_ip;
    else
        robot_ip = dualarm::right_ip;

    std::thread gripper_thread = std::thread(release, std::cref(robot_ip));
    gripper_thread.join();

    std::cout << "done release_one " << arm << " ";
}

void api_grasp_both () {
    std::cout << "grasp_both " << " ";

    std::thread gripper_thread_left = std::thread(grasp, std::cref(dualarm::left_ip));
    std::thread gripper_thread_right = std::thread(grasp, std::cref(dualarm::right_ip));
    gripper_thread_left.join();
    gripper_thread_right.join();

    std::cout << "done grasp_both " << " ";
}
void api_release_both () {
    std::cout << "release_both " << " ";

    std::thread gripper_thread_left = std::thread(release, std::cref(dualarm::left_ip));
    std::thread gripper_thread_right = std::thread(release, std::cref(dualarm::right_ip));
    gripper_thread_left.join();
    gripper_thread_right.join();

    std::cout << "done release_both " << " ";
}

int main () {
    //  Prepare our context and socket
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REP);
    socket.bind ("tcp://127.0.0.1:5555");
    // socket.bind ("ipc:///tmp/test");

    while (true) {
      std::cout << "waiting for message"
                << "\n";
      // // receive a multi part message
      // std::vector<zmq::message_t> request;
      // while (true) {
      //     zmq::message_t msg;
      //     auto res = socket.recv(&msg);
      //     request.push_back(msg);
      //     if (!msg.more()) {
      //         break;
      //     }
      // }

      std::vector<zmq::message_t> request;
      const auto ret = zmq::recv_multipart(socket, std::back_inserter(request));
      if (!ret)
        return 1;

      std::string command = std::string(static_cast<char *>(request[0].data()), request[0].size());
      if (command == "move_one")
      {
        std::string arm = std::string(static_cast<char *>(request[1].data()), request[1].size());
        std::vector<double> traj = vector_from_message(request[2]);
        api_move_one(arm, traj);
        }
        else if (command == "move_both"){
            std::vector<double> left_traj = vector_from_message(request[1]);    
            std::vector<double> right_traj = vector_from_message(request[2]);
            api_move_both(left_traj, right_traj);
        }
        else if (command == "grasp_one") {
            std::string arm = std::string(static_cast<char*>(request[1].data()), request[1].size());
            api_grasp_one(arm);
        }
        else if (command == "release_one") {
            std::string arm = std::string(static_cast<char*>(request[1].data()), request[1].size());
            api_release_one(arm);
        }
        else if (command == "grasp_both"){
            api_grasp_both();
        }
        else if (command == "grasp_both"){
            api_release_both();
        }

        //  Send reply back to client
        zmq::message_t reply (5);
        memcpy ((void *) reply.data (), "World", 5);
        socket.send(reply);
    }
    return 0;
}
