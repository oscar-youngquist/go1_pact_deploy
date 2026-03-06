#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <filesystem>
#include <mutex>
#include <fstream>
#include <future>
#include <algorithm>
#include <sstream>

#include "../3rd_parties/unitree_legged_sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"

#include "robot_interface.hpp"
#include "gamepad.hpp"
#include "pact_controller.hpp"
#include "state_machine.hpp"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class HardwareBridge
{
    public:
        HardwareBridge(std::string config_file_path);
        void UDPRecv();
        void UDPSend();
        void RobotControl();
        void run();
        void loadParam();
        void loadPolicy();
        void StartControl();
        void updateCommand();

        void InteprateGamePad();
        void set_cmd();
        void set_debug_cmd();
        void UpdateStateMachine();
        
        void Sitting();
        void Standing();
        void Damping(float kd = 3.0);
        void UserControlStep(bool send_cmd = false);

        void SitCallBack();
        void StandCallBack();
        void CtrlCallBack();


        Safety _safe;
        UDP _udp;
        LowCmd _leg_cmd = {0};
        LowState _state = {0};
        xRockerBtnDataStruct _keyData = {};
        
        Gamepad _gamepad;
        PACTStateMachine _state_machine;
        BasicUserController* _ctrl;
        BasicRobotInterface _robot_interface;

        std::mutex state_mutex, cmd_mutex;

        int _iter = 0;
        float _update_wbic_dt = 0.01;
        float _update_dt = 0.002;
};