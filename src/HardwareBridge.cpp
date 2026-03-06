#include "HardwareBridge.hpp"


using namespace std;
using namespace UNITREE_LEGGED_SDK;

HardwareBridge::HardwareBridge(std::string config_file_path):
_safe(LeggedType::Go1),
_udp(LOWLEVEL, 8090, "192.168.123.10", 8007)
{
    _udp.InitCmdData(_leg_cmd);

    std::cout << "Initilized Leg Command Struct" << std::endl;

    _ctrl = new PACTController(config_file_path); 

    std::cout << "Constructed Controller" << std::endl;
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


void HardwareBridge::RobotControl()
{
    // update state
    InteprateGamePad(); 

    // Update the control state
    UpdateStateMachine();

    {
        std::lock_guard<std::mutex> lock(state_mutex);
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

    LoopFunc loop_control("control_loop",       _update_wbic_dt,    boost::bind(&HardwareBridge::RobotControl, this));
    LoopFunc loop_udpSend("udp_send",           _update_dt,      3, boost::bind(&HardwareBridge::UDPSend, this));
    LoopFunc loop_udpRecv("udp_recv",           _update_dt,      3, boost::bind(&HardwareBridge::UDPRecv, this));

    // Start sending and receving data
    loop_udpSend.start();
    loop_udpRecv.start();

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

