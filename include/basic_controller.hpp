#pragma once

#include <array>
#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <deque>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <limits>

#include "torch/script.h"
#include <torch/torch.h>

#include "gamepad.hpp"
#include "robot_interface.hpp"
#include "param/cfg_param.hpp"
#include "../3rd_parties/unitree_legged_sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"

namespace fs = std::filesystem;
using namespace std;
using namespace UNITREE_LEGGED_SDK;

class BasicUserController
{
public:
    BasicUserController() {}

    virtual void loadParam() = 0;

    virtual void loadPolicy() = 0;

    virtual void reset(BasicRobotInterface &robot_interface, Gamepad &gamepad) = 0;

    virtual void GetInput(BasicRobotInterface &robot_interface, Gamepad &gamepad) = 0;

    virtual void DummyCalculate() = 0;

    virtual void Calculate() = 0;

    virtual std::vector<float> GetLog() = 0;

    void save_jpos(BasicRobotInterface &robot_interface)
    {
        std::copy(robot_interface.jpos.begin(), robot_interface.jpos.end(), start_pos.begin());
    }

    float dt;
    float stand_kp;
    float stand_kd;
    float ctrl_kp;
    float ctrl_kd;
    std::array<float, 12> start_pos;
    std::array<float, 12> stand_pos;
    std::array<float, 12> jpos_des;
    std::array<float, 12> jtau_des;
    std::array<float, 12> jvel_des;
    std::array<float, 12> sit_pos;
    std::array<float, 12> jpos_lim_min;
    std::array<float, 12> jpos_lim_max;
    std::array<float, 12> jtau_lim;
};