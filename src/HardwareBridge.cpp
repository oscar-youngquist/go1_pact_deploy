#include "HardwareBridge.hpp"


using namespace std;
using namespace UNITREE_LEGGED_SDK;

namespace
{
template <size_t N>
void AppendArrayHeaders(std::ostream& os, const std::string& name)
{
    for (size_t i = 0; i < N; ++i)
    {
        os << "," << name << "_" << i;
    }
}

template <size_t N>
void AppendArrayValues(std::ostream& os, const std::array<float, N>& values)
{
    for (const float value : values)
    {
        os << "," << value;
    }
}

std::filesystem::path GetPackageRootFromConfig(const std::string& config_file_path)
{
    namespace fs = std::filesystem;

    const fs::path config_path = fs::absolute(config_file_path).lexically_normal();
    if (config_path.parent_path().filename() == "params")
    {
        return config_path.parent_path().parent_path();
    }
    return config_path.parent_path();
}
}

HardwareBridge::HardwareBridge(std::string config_file_path):
_config_file_path(config_file_path),
_safe(LeggedType::Go1),
_udp(LOWLEVEL, 8090, "192.168.123.10", 8007)
{
    _udp.InitCmdData(_leg_cmd);

    std::cout << "Initilized Leg Command Struct" << std::endl;

    const PACTRLCfg cfg(config_file_path);
    if (cfg.command_limit_max_abs.size() >= 3)
    {
        _gamepad.setCommandLimits({cfg.command_limit_max_abs[0],
                                   cfg.command_limit_max_abs[1],
                                   cfg.command_limit_max_abs[2]});
    }
    _log_prefix = cfg.log_prefix;
    _log_directory = cfg.log_directory;
    _log_flush_count = std::max(1, cfg.log_flush_count);
    _log_loop_dt = cfg.log_loop_dt > 0.0f ? cfg.log_loop_dt : 0.05f;
    _log_buffer.reserve(_log_flush_count);

    if (cfg.ctrl_type == "pact")
    {
        _ctrl = new PACTController(config_file_path);
    }
    else if (cfg.ctrl_type == "position")
    {
        _ctrl = new PositionController(config_file_path);
    }
    else
    {
        throw std::invalid_argument("ctrl_type must be pact or position");
    }

    std::cout << "Constructed " << cfg.ctrl_type << " controller" << std::endl;
}

void HardwareBridge::UDPRecv()
{
    // Recieve the unitree state
    _udp.Recv();
   
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        
        // Decode the state
        _udp.GetRecv(_state);

        // Update the robot-interface with the current state
        _robot_interface.SetState(_state);
    }
}

void HardwareBridge::UDPSend()
{
    {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        _udp.Send();
    }
}

void HardwareBridge::loadParam()
{
    _ctrl->loadParam();
}

void HardwareBridge::loadPolicy()
{
    _ctrl->loadPolicy();
}

void HardwareBridge::InteprateGamePad()
{
    memcpy(&_keyData, &_state.wirelessRemote, 40);
    _gamepad.update(_keyData);
}

void HardwareBridge::StartControl()
{
    // waiting for _gamepad command to start the control thread
    std::chrono::milliseconds duration(100); // 100ms
    // listen to _gamepad command
    std::cout << "Press START button to start!" << std::endl;
    while (true)
    {
        std::this_thread::sleep_for(duration);

        InteprateGamePad();
        if (_gamepad.start.on_press)
        {
            break;
        }
    }

    // prepare for start
    std::cout << "Start!" << std::endl;
    Damping();
}


void HardwareBridge::UpdateStateMachine()
{
    // L1 + R1 -> Sit
    // L1 + R2 -> Stand
    // L1 + A -> Ctrl
    // select -> Stop
    if(_gamepad.R1.on_press && _gamepad.L1.pressed)
    {
        if(_state_machine.Sit())
        {
            SitCallBack();
        }
    }
    if (_gamepad.R2.on_press && _gamepad.L1.pressed)
    {
        if (_state_machine.Stand())
        {
            StandCallBack();
        }
    }
    if (_gamepad.A.on_press && _gamepad.L1.pressed)
    {
        if (_state_machine.Ctrl())
        {
            CtrlCallBack();
        }
    }
    if (_gamepad.B.on_press && _gamepad.L1.pressed)
    {
        _state_machine.Stop();
    }
}

void HardwareBridge::HandleLoggingButtons()
{
    if (_gamepad.left.on_press)
    {
        StartLogging();
    }
    if (_gamepad.right.on_press)
    {
        StopLogging();
    }
}

void HardwareBridge::StartLogging()
{
    if (_logging_enabled.load(std::memory_order_relaxed))
    {
        return;
    }

    {
        std::lock_guard<std::mutex> log_lock(log_mutex);
        _log_buffer.clear();
        _log_flush_requested.store(false);
        _log_file_ready = false;
        if (_log_file.is_open())
        {
            _log_file.close();
        }

        InitializeLogFile(_log_prefix);
        if (!_log_file_ready)
        {
            std::cout << "Logging requested, but log file is not ready." << std::endl;
            return;
        }

        _log_time_zero = std::chrono::steady_clock::now();
    }

    _logging_enabled.store(true);
    std::cout << "Started robot interface logging: " << _log_file_path << std::endl;
}

void HardwareBridge::StopLogging()
{
    const bool was_enabled = _logging_enabled.exchange(false);
    if (was_enabled)
    {
        std::lock_guard<std::mutex> log_lock(log_mutex);

        std::cout << "Stopped robot interface logging: " << _log_file_path << std::endl;

        WriteBufferedLogEntries(_log_buffer);
        _log_buffer.clear();
        _log_file.flush();
        _log_file.close();
        _log_file_ready = false;
        _log_flush_requested.store(false);

        std::cout << "Finished robot interface logging: " << _log_file_path << std::endl;
    }
}

void HardwareBridge::InitializeLogFile(const std::string& prefix)
{
    namespace fs = std::filesystem;

    const fs::path package_root = GetPackageRootFromConfig(_config_file_path);
    fs::path requested(prefix.empty() ? "pact_log" : prefix);
    if (requested.is_relative())
    {
        fs::path log_directory(_log_directory);
        if (log_directory.empty())
        {
            log_directory = package_root;
        }
        else if (log_directory.is_relative())
        {
            log_directory = package_root / log_directory;
        }
        requested = log_directory / requested;
    }
    if (!requested.has_extension())
    {
        requested += ".csv";
    }
    if (requested.has_parent_path())
    {
        fs::create_directories(requested.parent_path());
    }

    const fs::path parent = requested.parent_path();
    const std::string stem = requested.stem().string();
    const std::string ext = requested.extension().string();

    fs::path candidate = requested;
    int suffix = 1;
    while (fs::exists(candidate))
    {
        std::ostringstream name;
        name << stem << "_" << std::setw(2) << std::setfill('0') << suffix << ext;
        candidate = parent.empty() ? fs::path(name.str()) : parent / name.str();
        ++suffix;
    }

    candidate = fs::absolute(candidate).lexically_normal();

    _log_file.open(candidate, std::ios::out);
    if (!_log_file.is_open())
    {
        std::cerr << "Failed to open robot interface log file: " << candidate << std::endl;
        _log_file_ready = false;
        return;
    }

    _log_file_path = candidate.string();
    _log_file_ready = true;
    WriteLogHeader();
    std::cout << "Robot interface log file ready: " << _log_file_path << std::endl;
}

void HardwareBridge::WriteLogHeader()
{
    _log_file << "iter,time_sec";
    AppendArrayHeaders<12>(_log_file, "jpos");
    AppendArrayHeaders<12>(_log_file, "jvel");
    AppendArrayHeaders<12>(_log_file, "tau");
    AppendArrayHeaders<4>(_log_file, "quat");
    AppendArrayHeaders<3>(_log_file, "rpy");
    AppendArrayHeaders<3>(_log_file, "gyro");
    AppendArrayHeaders<3>(_log_file, "projected_gravity");
    AppendArrayHeaders<3>(_log_file, "acc");
    AppendArrayHeaders<3>(_log_file, "torso_pos_est");
    AppendArrayHeaders<3>(_log_file, "torso_vel_world_est");
    AppendArrayHeaders<3>(_log_file, "torso_vel_body_est");
    AppendArrayHeaders<3>(_log_file, "cmd");
    AppendArrayHeaders<12>(_log_file, "jpos_des");
    AppendArrayHeaders<12>(_log_file, "jvel_des");
    AppendArrayHeaders<12>(_log_file, "kp");
    AppendArrayHeaders<12>(_log_file, "kd");
    AppendArrayHeaders<12>(_log_file, "tau_ff");
    _log_file << "\n";
}

void HardwareBridge::WriteBufferedLogEntries(const std::vector<RobotLogEntry>& entries)
{
    if (!_log_file_ready)
    {
        return;
    }

    for (const RobotLogEntry& entry : entries)
    {
        const BasicRobotInterface& robot = entry.robot_interface;
        _log_file << entry.iter << "," << entry.time_sec;
        AppendArrayValues<12>(_log_file, robot.jpos);
        AppendArrayValues<12>(_log_file, robot.jvel);
        AppendArrayValues<12>(_log_file, robot.tau);
        AppendArrayValues<4>(_log_file, robot.quat);
        AppendArrayValues<3>(_log_file, robot.rpy);
        AppendArrayValues<3>(_log_file, robot.gyro);
        AppendArrayValues<3>(_log_file, robot.projected_gravity);
        AppendArrayValues<3>(_log_file, robot.acc);
        AppendArrayValues<3>(_log_file, robot.torso_pos_est);
        AppendArrayValues<3>(_log_file, robot.torso_vel_world_est);
        AppendArrayValues<3>(_log_file, robot.torso_vel_body_est);
        AppendArrayValues<3>(_log_file, robot.cmd);
        AppendArrayValues<12>(_log_file, robot.jpos_des);
        AppendArrayValues<12>(_log_file, robot.jvel_des);
        AppendArrayValues<12>(_log_file, robot.kp);
        AppendArrayValues<12>(_log_file, robot.kd);
        AppendArrayValues<12>(_log_file, robot.tau_ff);
        _log_file << "\n";
    }
}

void HardwareBridge::LogCurrentRobotInterface()
{
    if (!_logging_enabled.load(std::memory_order_relaxed))
    {
        return;
    }

    RobotLogEntry entry;
    entry.iter = _iter;
    entry.time_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - _log_time_zero).count();

    {
        std::lock_guard<std::mutex> state_lock(state_mutex);
        entry.robot_interface = _robot_interface;
    }

    {
        std::lock_guard<std::mutex> log_lock(log_mutex);
        if (!_logging_enabled.load(std::memory_order_relaxed) || !_log_file_ready)
        {
            return;
        }
        _log_buffer.push_back(entry);
    }
}

void HardwareBridge::LoggingLoop()
{
    std::vector<RobotLogEntry> entries;
    const bool flush_requested = _log_flush_requested.exchange(false);

    {
        std::lock_guard<std::mutex> lock(log_mutex);
        if (!_log_file_ready)
        {
            return;
        }

        if (_log_buffer.size() >= _log_flush_count || (flush_requested && !_log_buffer.empty()))
        {
            entries.swap(_log_buffer);
        }

        WriteBufferedLogEntries(entries);
        if (!entries.empty() || flush_requested)
        {
            _log_file.flush();
        }
    }
}


void HardwareBridge::RobotControl()
{
    // update state
    InteprateGamePad(); 

    // left d-pad starts logging, right d-pad stops and flushes pending entries
    HandleLoggingButtons();

    // Update the control state
    UpdateStateMachine();

    {
        std::lock_guard<std::mutex> lock(state_mutex);
        if (_torso_estimator)
        {
            _torso_estimator->Update(_state);
            _torso_pos_est = _torso_estimator->GetEstimatedPosition();
            _torso_vel_world_est = _torso_estimator->GetEstimatedVelocityWorld();
            _torso_vel_body_est = _torso_estimator->GetEstimatedVelocityBody();
            _robot_interface.SetTorsoEstimate(_torso_pos_est, _torso_vel_world_est, _torso_vel_body_est);
        }

        _ctrl->GetInput(_robot_interface, _gamepad);
    }

    // Processes the state we are in
    if(_state_machine.state == STATES::SIT)
    {
        Sitting();
    }
    if (_state_machine.state == STATES::STAND)
    {
        Standing();
    }
    if (_state_machine.state == STATES::DAMPING)
    {
        Damping();
    }
    if (_state_machine.state == STATES::CTRL)
    {
        UserControlStep(true);
    }
    
    // update desired WB command
    updateCommand();
    LogCurrentRobotInterface();
    _iter++;
}

void HardwareBridge::set_cmd()
{
    for(int i = 0; i < 12; i++)
    {
        _leg_cmd.motorCmd[i].q = _robot_interface.jpos_des.at(i);
        _leg_cmd.motorCmd[i].dq = _robot_interface.jvel_des.at(i);
        _leg_cmd.motorCmd[i].Kp = _robot_interface.kp.at(i);
        _leg_cmd.motorCmd[i].Kd = _robot_interface.kd.at(i);
        _leg_cmd.motorCmd[i].tau = _robot_interface.tau_ff.at(i);
    }
}

void HardwareBridge::set_debug_cmd()
{
    float debug_pose[12] = {0.0941026, 1.30853, -2.79159, -0.0208309, 1.22672, -2.8016, 0.134917, 1.27723, -2.79994, -0.0611606, 1.21782, 2.80245};

    for(int i = 0; i < 12; i++)
    {
        _leg_cmd.motorCmd[i].q = debug_pose[i];
        _leg_cmd.motorCmd[i].dq = 0.0;
        _leg_cmd.motorCmd[i].Kp = 0.0;
        _leg_cmd.motorCmd[i].Kd = 0.0;
        _leg_cmd.motorCmd[i].tau = 0.0;
    }
    std::cout << "DEFAULT POSE MODE: " << _iter << std::endl;
}

void HardwareBridge::updateCommand()
{
    {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        // set_debug_cmd();
        set_cmd();

        _safe.PositionLimit(_leg_cmd);
        int res = _safe.PowerProtect(_leg_cmd, _state, 9);

        if(res < 0)
        {
            std::cout << "Cutting power to motors via PowerProtect" << std::endl;
            exit(-1);
        }
    }

    
    _udp.SetSend(_leg_cmd);
}


void HardwareBridge::run(){    
    loadParam();

    std::cout << "Loaded params!" << std::endl;

    loadPolicy();
    
    std::cout << "Loaded Policy!" << std::endl;

    _update_wbic_dt = _ctrl->dt;
    _torso_estimator = std::make_unique<LinearKFObserver>(_config_file_path, _update_wbic_dt);

    LoopFunc loop_control("control_loop",       _update_wbic_dt,    boost::bind(&HardwareBridge::RobotControl, this));
    LoopFunc loop_udpSend("udp_send",           _update_dt,      3, boost::bind(&HardwareBridge::UDPSend, this));
    LoopFunc loop_udpRecv("udp_recv",           _update_dt,      3, boost::bind(&HardwareBridge::UDPRecv, this));
    LoopFunc loop_logging("logging_loop",       _log_loop_dt,       boost::bind(&HardwareBridge::LoggingLoop, this));

    // Start sending and receving data
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_logging.start();

    // Run a _ctrl bootup routine
    //    waits for user-input before starting control + senters damping state
    StartControl();

    // Start control loop
    loop_control.start();

    while(1){
        sleep(1);
    };
}

void HardwareBridge::SitCallBack()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        _ctrl->save_jpos(_robot_interface);
    }
    _robot_interface.jpos_des = _ctrl->start_pos; // this value is set by the save_jpos call above
    _robot_interface.kp.fill(_ctrl->stand_kp);
    _robot_interface.kd.fill(_ctrl->stand_kd);
    
    // Zero out unused commands
    _robot_interface.jvel_des.fill(0.);
    _robot_interface.tau_ff.fill(0.);
}

void HardwareBridge::StandCallBack()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        _ctrl->save_jpos(_robot_interface);
    }
    _robot_interface.jpos_des = _ctrl->start_pos; // this value is set by the save_jpos call above
    _robot_interface.kp.fill(_ctrl->stand_kp);
    _robot_interface.kd.fill(_ctrl->stand_kd);

    // Zero out unused commands
    _robot_interface.jvel_des.fill(0.);
    _robot_interface.tau_ff.fill(0.);
}

void HardwareBridge::CtrlCallBack()
{
    _ctrl->reset(_robot_interface, _gamepad); // zero-out current and history of obs

    // Dummy out the robot-interface commands in preperation to be filled by controller
    _robot_interface.jpos_des = _ctrl->stand_pos;
    _robot_interface.kp.fill(_ctrl->ctrl_kp);
    _robot_interface.kd.fill(_ctrl->ctrl_kd);

    // _robot_interface.kp.fill(_ctrl->stand_kp);
    // _robot_interface.kd.fill(_ctrl->stand_kd);
    
    _robot_interface.jvel_des.fill(0.);
    _robot_interface.tau_ff.fill(0.);
}

void HardwareBridge::Damping(float kd)
{
    _robot_interface.jpos_des.fill(0.);
    _robot_interface.jvel_des.fill(0.);
    _robot_interface.kp.fill(0.);
    _robot_interface.kd.fill(kd);
    _robot_interface.tau_ff.fill(0.);
}

void HardwareBridge::Sitting()
{
    // Calcualte intermediate "sitting" joint pose goals
    _state_machine.Sitting(*_ctrl);
    // copy over the indermediate goale
    _robot_interface.jpos_des = _ctrl->jpos_des;
}

void HardwareBridge::Standing()
{
    _ctrl->DummyCalculate(); // warmup the neural network
    _state_machine.Standing(*_ctrl);
    _robot_interface.jpos_des = _ctrl->jpos_des;
}

void HardwareBridge::UserControlStep(bool send_cmd)
{
    _ctrl->Calculate();
    
    if (send_cmd)
    {
        _robot_interface.jpos_des = _ctrl->jpos_des;
        _robot_interface.tau_ff = _ctrl->jtau_des;
        // _robot_interface.jvel_des = _ctrl->jvel_des;
        
        // _robot_interface.jpos_des = _ctrl->stand_pos;
        // _robot_interface.tau_ff.fill(0.0);
        _robot_interface.jvel_des.fill(0.0);
    }
    // _robot_interface.jpos_des = _ctrl->stand_pos;
    // _robot_interface.tau_ff.fill(0.0);
    // _robot_interface.jvel_des.fill(0.0);
}
