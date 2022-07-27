/*
 * @Author: likecanyon 1174578375@qq.com
 * @Date: 2022-07-25 22:19:57
 * @LastEditors: likecanyon 1174578375@qq.com
 * @LastEditTime: 2022-07-26 12:16:41
 * @FilePath: /gzq_ur_test/ForcePlan.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>

#include <ruckig/ruckig.hpp>
using namespace ur_rtde;
using namespace ruckig;
int main(int argc, char *argv[])
{
    RTDEControlInterface rtde_control("192.168.3.101");
    RTDEReceiveInterface rtde_receive("192.168.3.101");

    std::vector<double> TCPForce(6, 0.0);
    TCPForce = rtde_receive.getActualTCPForce();

    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<3> otg{0.01}; // control cycle
    InputParameter<3> input;
    OutputParameter<3> output;

    // Set input parameters
    input.current_position = {0, 0, 0};
    input.current_velocity = {0.0, 0, 0};
    input.current_acceleration = {0.0, 0, 0};

    input.target_position = {TCPForce[0], TCPForce[1], TCPForce[2]};

    input.target_velocity = {0.0, 0, 0};
    input.target_acceleration = {0.0, 0.0, 0.5};

    input.max_velocity = {3.0, 1.0, 3.0};
    input.max_acceleration = {3.0, 2.0, 1.0};
    input.max_jerk = {4.0, 3.0, 2.0};

    // Generate the trajectory within the control loop
    std::cout << "t | p1 | p2 | p3" << std::endl;
    while (otg.update(input, output) == Result::Working)
    {
        auto &p = output.new_position;
        std::cout << output.time << " " << p[0] << " " << p[1] << " " << p[2] << " " << std::endl;

        output.pass_to_input(input);
        TCPForce = rtde_receive.getActualTCPForce();
        input.target_position = {TCPForce[0], TCPForce[1], TCPForce[2]};
    }

    std::cout << "Trajectory duration: " << output.trajectory.get_duration() << " [s]." << std::endl;

    return 0;
}