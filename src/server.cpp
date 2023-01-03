#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <string>
#include <iostream>
#include <unistd.h>

std::vector<double> vector_from_message(zmq::message_t &msg) {
    int numValues = msg.size() / sizeof(double);
    std::vector<double> data;
    for(int i = 0; i < numValues; i++) {
        data.push_back(*reinterpret_cast<double*>(msg.data()+i*sizeof(double)));
    }
    return data;
}
void move_one (std::string arm, std::vector<double> traj) {
    // the traj vector contains a flattened joint space
    // trajectory sampled at 1Khz
    // arm is one of ["left_panda", "right_panda"]

    // we want to move the arm along the desired trajectory
    // before returning to the caller
    std::cout << "move_one " << arm << " ";
}
void move_both (std::vector<double> left_traj, std::vector<double> right_traj) {
    std::cout << "move_both " << " ";
}

void grasp_one (std::string arm) {
    // std::string robot_ip;
    // if (arm == "left_panda")
    //     robot_ip = "192.168.2.107"
    // else
    //     robot_ip = "192.168.2.109"

    // franka::Gripper gripper(robot_ip);
    // grasp = gripper.grasp(grasp_width, 0.1, 80,0.005,0.005); //width, speed, force
    
    std::cout << "grasp_one " << arm << " ";
}
void release_one (std::string arm) {
    std::cout << "release_one " << arm << " ";
}
void grasp_both () {
    std::cout << "grasp_both " << " ";
}
void release_both () {
    std::cout << "release_both " << " ";
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
        move_one(arm, traj);
        }
        else if (command == "move_both"){
            std::vector<double> left_traj = vector_from_message(request[1]);    
            std::vector<double> right_traj = vector_from_message(request[2]);
            move_both(left_traj, right_traj);
        }
        else if (command == "grasp_one") {
            std::string arm = std::string(static_cast<char*>(request[1].data()), request[1].size());
            grasp_one(arm);
        }
        else if (command == "release_one") {
            std::string arm = std::string(static_cast<char*>(request[1].data()), request[1].size());
            release_one(arm);
        }
        else if (command == "grasp_both"){
            grasp_both();
        }
        else if (command == "grasp_both"){
            release_both();
        }

        //  Send reply back to client
        zmq::message_t reply (5);
        memcpy ((void *) reply.data (), "World", 5);
        socket.send(reply);
    }
    return 0;
}
