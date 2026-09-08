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
#include <memory>
#include <sstream>
#include <atomic>
#include <iomanip>

#include "../3rd_parties/unitree_legged_sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"

#include "robot_interface.hpp"
#include "gamepad.hpp"
#include "pact_controller.hpp"
#include "posistion_controller.hpp"
#include "state_machine.hpp"
#include "linear_kf_observer.hpp"

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

        void HandleLoggingButtons();
        void StartLogging();
        void StopLogging();
        void LogCurrentRobotInterface();
        void LoggingLoop();
        void InitializeLogFile(const std::string& prefix);
        void WriteLogHeader();


        Safety _safe;
        UDP _udp;
        LowCmd _leg_cmd = {0};
        LowState _state = {0};
        xRockerBtnDataStruct _keyData = {};
        
        Gamepad _gamepad;
        PACTStateMachine _state_machine;
        BasicUserController* _ctrl;
        BasicRobotInterface _robot_interface;
        std::unique_ptr<LinearKFObserver> _torso_estimator;
        std::array<float, 3> _torso_pos_est = {0.f, 0.f, 0.f};
        std::array<float, 3> _torso_vel_world_est = {0.f, 0.f, 0.f};
        std::array<float, 3> _torso_vel_body_est = {0.f, 0.f, 0.f};

        std::mutex state_mutex, cmd_mutex;
        std::mutex log_mutex;

        int _iter = 0;
        float _update_wbic_dt = 0.01;
        float _update_dt = 0.002;
        std::string _config_file_path;

        struct RobotLogEntry
        {
            int iter = 0;
            double time_sec = 0.0;
            BasicRobotInterface robot_interface;
        };
        void WriteBufferedLogEntries(const std::vector<RobotLogEntry>& entries);

        std::atomic<bool> _logging_enabled{false};
        std::atomic<bool> _log_flush_requested{false};
        std::vector<RobotLogEntry> _log_buffer;
        std::ofstream _log_file;
        std::string _log_file_path;
        std::string _log_prefix = "pact_log";
        std::string _log_directory;
        size_t _log_flush_count = 500;
        float _log_loop_dt = 0.05f;
        bool _log_file_ready = false;
        std::chrono::steady_clock::time_point _log_time_zero = std::chrono::steady_clock::now();
};
