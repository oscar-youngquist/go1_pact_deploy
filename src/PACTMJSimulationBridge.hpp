#ifndef __PACT_MJ_SIMULATION_BRIDGE_H__
#define __PACT_MJ_SIMULATION_BRIDGE_H__

#include "MujocoSimulationBridge.hpp"
#include <math.h>
#include <iostream>
#include <stdint.h>
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
#include "physics_joystick.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;


class PACTMJSimulationBridge: public MujocoSimulationBridge{
  public:
    PACTMJSimulationBridge(const std::string & config_file, const std::string& ctrl_config_file);
    virtual ~PACTMJSimulationBridge(){}

    // virtual void _UpKeyAction();
    // virtual void _DownKeyAction();
    // virtual void _LeftKeyAction();
    // virtual void _RightKeyAction();
    // virtual void _A_KeyAction();
    // virtual void _D_KeyAction();
    // virtual void _W_KeyAction();
    // virtual void _S_KeyAction();
    // virtual void _Comma_KeyAction();
    // virtual void _Slash_KeyAction();

    // // Used for user-input styel control (replicating simialr behavior to the Unity simulation)
    // virtual void _LeftKeyRelease();
    // virtual void _RightKeyRelease();
    // virtual void _UpKeyRelease();
    // virtual void _DownKeyRelease();
    // virtual void _Comma_KeyRelease();
    // virtual void _Slash_KeyRelease();

    // // Used to switch between keyboard control and predefiend ctrl 
    // virtual void _K_KeyAction();
    
    // // enable/Disable testing and enabling adaptive control
    // virtual void _Y_KeyAction();
    // virtual void _U_KeyAction();

    // Increase / Decrease extenral forces for debugging
    virtual void _Z_KeyAction();
    virtual void _X_KeyAction();

    virtual float _getCtrlDt();
    virtual void _runCtrl();

    float _lateral_time = 0.0;

    void StartControl();
    void updateCommand();

    void InteprateGamePad();
    void UpdateStateMachine();
    
    void Sitting();
    void Standing();
    void Damping(float kd = 3.0);
    void UserControlStep(bool send_cmd = false);

    void SitCallBack();
    void StandCallBack();
    void CtrlCallBack();

  protected:
    xRockerBtnDataStruct _keyData = {};
    LowState _state = {0};
    
    std::mutex state_mutex, cmd_mutex;

    Gamepad _gamepad;
    PACTStateMachine _state_machine;
    BasicUserController* _ctrl;
    BasicRobotInterface _robot_interface;
    std::unique_ptr<XBoxJoystick> _joystick;

    virtual void _UpdateSystemObserver();
    virtual void _UpdateControlCommand();
    virtual void _RenderCameraData() override {}
    virtual void _ApplyExternalForces();
    virtual void _onestep_forward();

    void _RunAdaptiveCtrl();


    void _UpdateIMUData();
    void _UpdateTouchData();
    void _ReadConfig(const std::string& config_file);
    bool _CollisionChecking();
    
    void _ApplyDebuggingExternalForces();

    float _userIn_external_z;

    float _external_z_step;

    std::vector<int> _good_contacts;


    float _pd_time;
    float _pd_dt;

    // Quick and dirty variables for applying random velocities (translated into forces)
    //     during training data collection
    float _summer_mass = 13.0672;
    float _v_max = 0.02;
    float _current_sample_time;

    Vec3<float> _torso_velo;


    mjtNum rand_dist[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    DVec<float> _j_cmd;
    DVec<float> _tau_cmd;

    int _num_joints;
    int _num_acts;

    // Device path
    std::string _js_device;

    bool _updated_command = false;
};

#endif
