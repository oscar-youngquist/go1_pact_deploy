#pragma once

#include <array>
#include <iostream>

#include "../3rd_parties/unitree_legged_sdk/include/unitree_legged_sdk/comm.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class BasicRobotInterface
{
public:
    BasicRobotInterface()
    {
        jpos_des.fill(0.);
        jvel_des.fill(0.);
        kp.fill(0.);
        kd.fill(0.);
        tau_ff.fill(0.);
        projected_gravity.fill(0.);
        projected_gravity.at(2) = -1.0;
    }


    void SetState(LowState state)
    {
        // std::cout << "state.imu.gyroscope - " << state.imu.gyroscope << std::endl;
        // std::cout << "state.imu.quaternion - " << state.imu.quaternion << std::endl;
        // std::cout << "state.imu.accelerometer - " << state.imu.accelerometer << std::endl;
        // std::cout << "state.imu.rpy - " << state.imu.rpy << std::endl;

        // std::cout << "state.imu.gyroscope elements: ";
        // for (const auto& element : state.imu.gyroscope) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;
        
        // std::cout << "state.imu.quaternion elements: ";
        // for (const auto& element : state.imu.quaternion) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "state.imu.accelerometer elements: ";
        // for (const auto& element : state.imu.accelerometer) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;

        //         std::cout << "state.imu.rpy elements: ";
        // for (const auto& element : state.imu.rpy) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;

        std::copy(state.imu.gyroscope.begin(), state.imu.gyroscope.end(), gyro.begin());
        std::copy(state.imu.quaternion.begin(), state.imu.quaternion.end(), quat.begin());
        std::copy(state.imu.accelerometer.begin(), state.imu.accelerometer.end(), acc.begin());
        std::copy(state.imu.rpy.begin(), state.imu.rpy.end(), rpy.begin());

        UpdateProjectedGravity();

        // motor
        for (size_t i = 0; i < 12; ++i)
        {
            const MotorState &m = state.motorState.at(i);
            jpos.at(i) = m.q;
            jvel.at(i) = m.dq;
            tau.at(i) = m.tauEst;
        }

        // std::cout << "gyro elements: ";
        // for (const auto& element : gyro) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;
        
        // std::cout << "quat elements: ";
        // for (const auto& element : quat) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "acc elements: ";
        // for (const auto& element : acc) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;

        //         std::cout << "rpy elements: ";
        // for (const auto& element : rpy) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "-------------------------------------------" << std::endl;
    }

    std::array<float, 12> jpos, jvel, tau;
    std::array<float, 4> quat;
    std::array<float, 3> rpy, gyro, projected_gravity, acc;
    std::array<float, 12> jpos_des, jvel_des, kp, kd, tau_ff;

private:
    inline void UpdateProjectedGravity()
    {
        // inverse quat
        float w = quat.at(0);
        float x = -quat.at(1);
        float y = -quat.at(2);
        float z = -quat.at(3);

        float x2 = x * x;
        float y2 = y * y;
        float z2 = z * z;
        float w2 = w * w;
        float xz = x * z;
        float yz = y * z;
        float wx = w * x;
        float wy = w * y;

        projected_gravity.at(0) = -2 * (xz + wy);
        projected_gravity.at(1) = -2 * (yz - wx);
        projected_gravity.at(2) = -(w2 - x2 - y2 + z2);
    }
};
