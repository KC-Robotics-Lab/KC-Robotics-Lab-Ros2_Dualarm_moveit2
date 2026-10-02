#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace std::chrono_literals;
using namespace rb;

int main() {
  try {
    // Make connection
    auto robot = podo::Cobot("192.168.0.6", 5000);
    auto rc = podo::ResponseCollector();

    // while(1)
    {
    robot.gripper_inspire_humanoid_hand_initialization(rc, 2);
    std::this_thread::sleep_for(0.5s);

    robot.gripper_inspire_humanoid_hand_set_finger(rc, 1, 50, 50, 50, 50, 50, 50);
    std::this_thread::sleep_for(2s);

    robot.gripper_inspire_humanoid_hand_set_finger(rc, 1, 50, 50, 50, 50, 100, 50);
    std::this_thread::sleep_for(2s);
    }

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}

// g++ -std=c++17 gripper.cpp -o gripper_test     -I/usr/local/include     /usr/local/lib/librbpodo.a     -pthread     //컴파일
// ./gripper_test     //실행

// int main() {
//   try {
//     // Make connection
//     auto robot = podo::Cobot("192.168.0.6");
//     auto rc = podo::ResponseCollector();

//         robot.gripper_rts_rhp12rn_select_mode(rc, podo::GripperConnectionPoint::ToolFlange_Advanced, true);
//         std::this_thread::sleep_for(0.5s);

//         robot.gripper_rts_rhp12rn_force_control(rc, podo::GripperConnectionPoint::ToolFlange_Advanced, 100);
//         std::this_thread::sleep_for(2s);

//         robot.gripper_rts_rhp12rn_force_control(rc, podo::GripperConnectionPoint::ToolFlange_Advanced, -100);
//         std::this_thread::sleep_for(2s);

//         robot.gripper_rts_rhp12rn_force_control(rc, podo::GripperConnectionPoint::ToolFlange_Advanced, 0);
//   } catch (const std::exception& e) {
//     std::cerr << e.what() << std::endl;
//     return 1;
//   }
//   return 0;
// }