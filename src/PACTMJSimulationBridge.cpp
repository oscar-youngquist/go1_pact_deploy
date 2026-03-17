#include <ParamHandler/ParamHandler.hpp>
#include "PACTMJSimulationBridge.hpp"
#include <Configuration.h>
#include "utilities.h"
#include <orientation_tools.h>


using namespace std;

PACTMJSimulationBridge::PACTMJSimulationBridge(const std::string &sim_config_file, const std::string& ctrl_config_file) : MujocoSimulationBridge(sim_config_file)
{
    _ReadConfig(sim_config_file);

    _joystick = std::make_unique<XBoxJoystick>(_js_device);

    _ctrl = new PACTController(ctrl_config_file);

    // run simulation before run controller
    _ctrl_time += _getCtrlDt();

    _pd_time = 0.0;

    _j_cmd = DVec<float>::Zero(_num_acts);
    _tau_cmd = DVec<float>::Zero(_num_acts);
    
    _userIn_external_z = 0;
    _current_sample_time = 0.0;

    // IDs for the feet contacts and the floor
    _good_contacts = {19,20,30,31,42,43,54,55,0};

    // Load the parameters into the controller
    _ctrl->loadParam();
    
    // Load the actual model/policy trained weights
    _ctrl->loadPolicy();

    printf("[Summer Mujoco Simulation Bridge] Constructed\n");

    // Run a _ctrl bootup routine
    //    waits for user-input before starting control + senters damping state
    StartControl();
}

void PACTMJSimulationBridge::_onestep_forward()
{
    if (_mjData->time >= _ctrl_time)
    {
        _ctrl_time += _getCtrlDt();
        _UpdateSystemObserver(); // copy to state_save

        _runCtrl();

        _UpdateControlCommand();

        if (!_updated_command)
        {
            _updated_command = true;
        }
        // std::cout << "stepping ctrl" << std::endl;
        // std::cout << "_mjData->time: " << _mjData->time << std::endl;
        // std::cout << "_ctrl_time: " << _ctrl_time << std::endl;
    }

    if (_mjData->time >= _pd_time && _updated_command)
    {
        _pd_time += _pd_dt;
        _UpdateControlCommand();
    }

    // _ApplyExternalForces();
    // _ApplyDebuggingExternalForces();

    // std::cout << "stepping simulation" << std::endl;
    mj_step(_mjModel, _mjData);

    // if (_CollisionChecking())
    // {
    //   this->_sim_duration = _mjData->time;
    // }
}

float PACTMJSimulationBridge::_getCtrlDt()
{
    return _ctrl->dt;
}

bool PACTMJSimulationBridge::_CollisionChecking()
{
  for (int i =0; i < _mjData->ncon; ++i)
  {
    if (std::find(_good_contacts.begin(), _good_contacts.end(), _mjData->contact[i].geom[0]) == _good_contacts.end() ||
        std::find(_good_contacts.begin(), _good_contacts.end(), _mjData->contact[i].geom[1]) == _good_contacts.end())
      {
        if (_mjModel->geom_bodyid[_mjData->contact[i].geom[0]] ==0 || _mjModel->geom_bodyid[_mjData->contact[i].geom[1]] == 0)
        {
          const char* name_1 = mj_id2name(_mjModel, mjOBJ_BODY, _mjModel->geom_bodyid[_mjData->contact[i].geom[0]]);
          const char* name_2 = mj_id2name(_mjModel, mjOBJ_BODY, _mjModel->geom_bodyid[_mjData->contact[i].geom[1]]);
          std::cout << "Detected Invalid Collision! Geom-IDs of contacts: " << _mjData->contact[i].geom[0] <<"(" << name_1 << ") , " << _mjData->contact[i].geom[1] << "(" << name_2 << ")" << std::endl;
          return true;
        }
      }
  }
  return false;
}

void PACTMJSimulationBridge::_UpdateIMUData()
{
    int acc_sensor_id = mj_name2id(_mjModel, mjOBJ_SENSOR, "accelerometer");
    int gyro_sensor_id = mj_name2id(_mjModel, mjOBJ_SENSOR, "gyro");
    
    // Access accelerometer data
    if (acc_sensor_id != -1)
    {
        const double *acc_data = _mjData->sensordata + _mjModel->sensor_adr[acc_sensor_id];
        for (int i(0); i < 3; i++)
        {
            _state.imu.accelerometer[i] = acc_data[i];
        }
        // printf("acc_sensor_id: %d, gyro_sensor_id: %d\n", acc_sensor_id, gyro_sensor_id);
    }

    // Access gyro data
    if (gyro_sensor_id != -1)
    {
        const double *gyro_data = _mjData->sensordata + _mjModel->sensor_adr[gyro_sensor_id];
        for (int i(0); i < 3; i++)
        {
            _state.imu.gyroscope[i] = gyro_data[i];
        }
        // printf("gyro_data: %f, %f, %f\n", gyro_data[0], gyro_data[1], gyro_data[2]);
    }

    // Access quaternion
    for (int i(0); i < 4; i++)
    {
        _state.imu.quaternion[i] = _mjData->qpos[i+3];
    }

    // Store eluer angles
    Quat<float> body_quat(_mjData->qpos[3], _mjData->qpos[4], _mjData->qpos[5], _mjData->qpos[6]);
    Vec3<float> body_rpy = ori::quatToRPY(body_quat);

    // Access quaternion
    for (int i(0); i < 3; i++)
    {
        _state.imu.rpy[i] = body_rpy[i+3];
    }
}

void PACTMJSimulationBridge::_UpdateSystemObserver()
{
    // Copy IMU data (acc, rpy, quat, ang. velo.) into unitree's state structure
    _UpdateIMUData();


    // Copy joint position and velocity data into unitree's state structure
    for (int i(0); i < _num_joints; i++)
    {
        _state.motorState.at(i).q  = _mjData->qpos[i+7];
        _state.motorState.at(i).dq = _mjData->qvel[i+6];
    }

    // Now copy fom _state structure into robot_interface
    _robot_interface.SetState(_state);
}

void PACTMJSimulationBridge::_UpdateControlCommand()
{
    DVec<float> jtorque_output(_num_acts);
    DVec<float> jpos(_num_acts);
    DVec<float> jvel(_num_acts);

    for (int i = 0; i < _num_acts; ++i)
    {
        jpos[i] = static_cast<float>(_mjData->qpos[i + 7]);
        jvel[i] = static_cast<float>(_mjData->qvel[i + 6]);
    }

    for (int i = 0; i < _num_acts; ++i)
    {
        _j_cmd[i] = _robot_interface.jpos_des.at(i);
        _tau_cmd[i] = _robot_interface.tau_ff.at(i);
        jtorque_output[i] = _tau_cmd[i] + _robot_interface.kp.at(i)*(_j_cmd[i] - jpos[i]) - _robot_interface.kd.at(i) * jvel[i];
    }

    // std::cout << "_j_cmd -> " << _j_cmd.transpose() << std::endl;
    // std::cout << "_tau_cmd -> " << _tau_cmd.transpose() << std::endl;

    for (int i = 0; i < _num_acts; ++i)
    {
        _mjData->ctrl[i] = jtorque_output[i];
    }
}

void PACTMJSimulationBridge::_ApplyExternalForces()
{
  // UserInputManager *user_input = UserInputManager::GetInstance();
  // int body_id;
  // mjtNum force[3];
  // mjtNum torque[3] = {0.0, 0.0, 0.0};
  // mjtNum point[3] = {0.0, 0.0, 0.0};
  //
  // for (size_t i(0); i < user_input->ext_forces.size(); ++i)
  // {
  //   if (_mjData->time > user_input->ext_forces[i].start_time && 
  //     _mjData->time < user_input->ext_forces[i].start_time + user_input->ext_forces[i].duration &&
  //   !(user_input->ext_forces[i].b_applied))
  //   {
  //     force[0] = user_input->ext_forces[i].force[0];
  //     force[1] = user_input->ext_forces[i].force[1];
  //     force[2] = user_input->ext_forces[i].force[2];
  //     torque[0] = user_input->ext_forces[i].torque[0];
  //     torque[1] = user_input->ext_forces[i].torque[1];
  //     torque[2] = user_input->ext_forces[i].torque[2];
  //
  //     _mjData->qfrc_applied[0] = force[0];
  //     _mjData->qfrc_applied[1] = force[1];
  //     _mjData->qfrc_applied[2] = force[2];
  //     _mjData->qfrc_applied[3] = torque[0];
  //     _mjData->qfrc_applied[4] = torque[1];
  //     _mjData->qfrc_applied[5] = torque[2];
  //
  //     _userIn_external_z = user_input->ext_forces[i].force[2];
  //
  //     user_input->ext_forces[i].b_applied = true;
  //     // simulation runtime is 0.001
  //     user_input->ext_forces[i].run_time += 0.001;
  //
  //     std::cout << "ext_force: " << force[0] << ", " << force[1] << ", " << force[2] << std::endl;
  //   }
  //   if (_mjData->time > user_input->ext_forces[i].start_time && 
  //     _mjData->time < user_input->ext_forces[i].start_time + user_input->ext_forces[i].duration &&
  //   (user_input->ext_forces[i].b_applied))
  //   {
  //     // apply random torques every 0.01 seconds to simulate sloshing...
  //     if (user_input->ext_forces[i].run_time >= 0.3)
  //     {
  //       user_input->ext_forces[i].run_time = 0;
  //       rand_dist[0] = random_in_range(user_input->ext_forces[i].torque_min[0], user_input->ext_forces[i].torque_max[0]);
  //       rand_dist[1] = random_in_range(user_input->ext_forces[i].torque_min[1], user_input->ext_forces[i].torque_max[1]);
  //       rand_dist[2] = random_in_range(user_input->ext_forces[i].torque_min[2], user_input->ext_forces[i].torque_max[2]);
  //      
  //       _mjData->qfrc_applied[3] = user_input->ext_forces[i].torque[0] + 0.005*rand_dist[0];
  //       _mjData->qfrc_applied[4] = user_input->ext_forces[i].torque[1] + 0.005*rand_dist[1];
  //       _mjData->qfrc_applied[5] = user_input->ext_forces[i].torque[2] + 0.005*rand_dist[2];
  //
  //       // std::cout << "New random torques: (" << _mjData->qfrc_applied[3] <<", " << _mjData->qfrc_applied[4] << ", " << _mjData->qfrc_applied[5] << ")" << std::endl; 
  //
  //       // // Randomly generate lateral disterbances 
  //       // rand_dist[0] = random_in_range(-20.0, 20.0);
  //       // rand_dist[1] = random_in_range(-20.0, 20.0);
  //
  //       // _mjData->qfrc_applied[0] = rand_dist[0];
  //       // _mjData->qfrc_applied[1] = rand_dist[1];
  //
  //     }
  //     else
  //     {
  //       // accumulates the random torques over the 200 timesteps before the next random torques are sampled...
  //       float random_stepsize = random_in_range(0.001, 0.01);
  //
  //       _mjData->qfrc_applied[3] += random_stepsize*rand_dist[0];
  //       _mjData->qfrc_applied[4] += random_stepsize*rand_dist[1];
  //       _mjData->qfrc_applied[5] += random_stepsize*rand_dist[2];
  //
  //       // std::cout << "Stepping applied-torques: (" << _mjData->qfrc_applied[3] <<", " << _mjData->qfrc_applied[4] << ", " << _mjData->qfrc_applied[5] << ")" << std::endl; 
  //       // std::cout << "\tStep Size: (" << random_stepsize*rand_dist[0] <<", " << random_stepsize*rand_dist[1] << ", " << random_stepsize*rand_dist[2] << ")" << std::endl; 
  //     }
  //     user_input->ext_forces[i].run_time += 0.001;
  //   }
  //   if (_mjData->time > user_input->ext_forces[i].start_time + user_input->ext_forces[i].duration &&
  //       user_input->ext_forces[i].b_applied)
  //   {
  //     // reset applied forces
  //     _mjData->qfrc_applied[0] = 0.0;
  //     _mjData->qfrc_applied[1] = 0.0;
  //     _mjData->qfrc_applied[2] = 0.0;
  //     _mjData->qfrc_applied[3] = 0.0;
  //     _mjData->qfrc_applied[4] = 0.0;
  //     _mjData->qfrc_applied[5] = 0.0;
  //     user_input->ext_forces[i].b_applied = false;
  //   }
  // }
  //
  // // apply random torques every 0.2 seconds to simulate sloshing...
  // if (_lateral_time > 0.2)
  // {
  //   _lateral_time = 0.0;
  //   // Randomly generate lateral disterbances 
  //   rand_dist[0] = random_in_range(-20.0, 20.0);
  //   rand_dist[1] = random_in_range(-20.0, 20.0);
  //
  //   _mjData->qfrc_applied[0] = rand_dist[0];
  //   _mjData->qfrc_applied[1] = rand_dist[1];
  // }
  // _lateral_time += 0.001;
  //
  // randomly sample a set of torso velocities (x,y,z) at 1 Hz
  //     and convert it into a "force" via F = m * (v_sample - v_current) / dt
  //     This effectively approxmiates the applying the force required to accelerate
  //     the torso to the desire velocities. This follows the procedure from DreamWaQ eval - https://arxiv.org/pdf/2301.10602
  //     We are doing this to introduce noise into the data collection process to train
  //     locomotion control policies from collected WBIC data.

  if (_current_sample_time < _pd_dt)
  {
    rand_dist[0] = random_in_range(-10.0, 10.0);
    rand_dist[1] = random_in_range(-10.0, 10.0);
    rand_dist[2] = random_in_range(-20.0, 5.0);
    rand_dist[3] = random_in_range(-2.00, 2.00);
    rand_dist[4] = random_in_range(-3.0, 3.0);
    rand_dist[5] = random_in_range(-0.2, 0.2);

    float zero_prob = random_in_range(0.0, 0.60);

    // Randomly zero out some forces...
    if (random_in_range(0.0, 1.0) < zero_prob)
    {
      rand_dist[0] = 0.0;
    }
    if (random_in_range(0.0, 1.0) < zero_prob)
    {
      rand_dist[1] = 0.0;
    }
    if (random_in_range(0.0, 1.0) < zero_prob)
    {
      rand_dist[2] = 0.0;
    }
    if (random_in_range(0.0, 1.0) < zero_prob)
    {
      rand_dist[3] = 0.0;
    }
    if (random_in_range(0.0, 1.0) < zero_prob)
    {
      rand_dist[4] = 0.0;
    }
    if (random_in_range(0.0, 1.0) < zero_prob)
    {
      rand_dist[5] = 0.0;
    }

    if (random_in_range(0.0, 1.0) < 0.10)
    {
      rand_dist[0] = 0.0;
      rand_dist[1] = 0.0;
      rand_dist[2] = 0.0;
      rand_dist[3] = 0.0;
      rand_dist[4] = 0.0;
      rand_dist[5] = 0.0;
    }

    _mjData->qfrc_applied[0] = rand_dist[0];
    _mjData->qfrc_applied[1] = rand_dist[1];
    _mjData->qfrc_applied[2] = rand_dist[2];
    _mjData->qfrc_applied[3] = rand_dist[3];
    _mjData->qfrc_applied[4] = rand_dist[4];
    _mjData->qfrc_applied[5] = rand_dist[5];

    // std::cout << "New Sample - Applied Forques: (" << _mjData->qfrc_applied[0] <<", " << _mjData->qfrc_applied[1] << ", " << _mjData->qfrc_applied[2] << ")" << std::endl; 
    // std::cout << "New Sample - Applied Torques: (" << _mjData->qfrc_applied[3] <<", " << _mjData->qfrc_applied[4] << ", " << _mjData->qfrc_applied[5] << ")" << std::endl; 

    _current_sample_time += _pd_dt;    
  }
  else if (_current_sample_time < 2.6)
  {
    _mjData->qfrc_applied[0] = rand_dist[0];
    _mjData->qfrc_applied[1] = rand_dist[1];
    _mjData->qfrc_applied[2] = rand_dist[2];
    _mjData->qfrc_applied[3] = rand_dist[3];
    _mjData->qfrc_applied[4] = rand_dist[4];
    _mjData->qfrc_applied[5] = rand_dist[5];

    // std::cout << "Applied Forques: (" << _mjData->qfrc_applied[0] <<", " << _mjData->qfrc_applied[1] << ", " << _mjData->qfrc_applied[2] << ")" << std::endl; 

    _current_sample_time += _pd_dt;    
  }
  else
  {
    _current_sample_time = 0.0;
    _mjData->qfrc_applied[0] = 0.0;
    _mjData->qfrc_applied[1] = 0.0;
    _mjData->qfrc_applied[2] = 0.0;
  }
}

void PACTMJSimulationBridge::_ApplyDebuggingExternalForces()
{
    if (_userIn_external_z != 0.0)
    {
        _mjData->qfrc_applied[2] = _userIn_external_z;
    }
}

// Key Press Actions for Debugging Payloads
void PACTMJSimulationBridge::_Z_KeyAction()
{
    // Decrease applied force
    _userIn_external_z += _external_z_step;

    // Cap the force at zero, we do not want to pull the robot off of the ground
    if (_userIn_external_z > 0)
    {
    _userIn_external_z = 0.0;
    }

    std::cout << "applied force : " << _userIn_external_z << "N" << std::endl;  
}

void PACTMJSimulationBridge::_X_KeyAction()
{
    // Increase the applied force
    _userIn_external_z -= _external_z_step;

    std::cout << "applied force : " << _userIn_external_z << "N" << std::endl;
}

void PACTMJSimulationBridge::_ReadConfig(const std::string &config_file)
{
    ParamHandler handler(config_file);
    handler.getValue("external_z_step", _external_z_step);
    handler.getInteger("num_joints", _num_joints);
    handler.getInteger("num_joints", _num_acts);
    handler.getValue("pd_timestep", _pd_dt);
    handler.getString("js_device", _js_device);
}

void PACTMJSimulationBridge::InteprateGamePad()
{
    // Use the mapping class to fill the _keyData struct
    _joystick->update(_keyData);

    // inteprate the _keyData values
    _gamepad.update(_keyData);
}

void PACTMJSimulationBridge::StartControl()
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

void PACTMJSimulationBridge::_runCtrl()
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
}

void PACTMJSimulationBridge::Damping(float kd)
{
    _robot_interface.jpos_des.fill(0.);
    _robot_interface.jvel_des.fill(0.);
    _robot_interface.kp.fill(0.);
    _robot_interface.kd.fill(kd);
    _robot_interface.tau_ff.fill(0.);
}

void PACTMJSimulationBridge::Sitting()
{
    // Calcualte intermediate "sitting" joint pose goals
    _state_machine.Sitting(*_ctrl);
    // copy over the indermediate goale
    _robot_interface.jpos_des = _ctrl->jpos_des;
}

void PACTMJSimulationBridge::Standing()
{
    _ctrl->DummyCalculate(); // warmup the neural network
    _state_machine.Standing(*_ctrl);
    _robot_interface.jpos_des = _ctrl->jpos_des;
}

void PACTMJSimulationBridge::UserControlStep(bool send_cmd)
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

void PACTMJSimulationBridge::UpdateStateMachine()
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


void PACTMJSimulationBridge::SitCallBack()
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

void PACTMJSimulationBridge::StandCallBack()
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

void PACTMJSimulationBridge::CtrlCallBack()
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

// // Start Key Press Actions
// void PACTMJSimulationBridge::_UpKeyAction()
// {
//   // Increase the forward velocity 
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->leftStickAnalogue[0] = 1.0;
//   }
//   else
//   {
//     UserInputManager::GetInstance()->v_des[0] += UserInputManager::GetInstance()->x_velo_step;
//   }
// }

// void PACTMJSimulationBridge::_DownKeyAction()
// {
//   // Decrease forward velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->leftStickAnalogue[0] = -1.0;
//   }
//   else
//   {
//     UserInputManager::GetInstance()->v_des[0] -= UserInputManager::GetInstance()->x_velo_step;
//   }
// }

// void PACTMJSimulationBridge::_LeftKeyAction()
// {
//   // Decrease lateral velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->leftStickAnalogue[1] = 1.0;
//   }
//   else
//   {
//     UserInputManager::GetInstance()->v_des[1] += UserInputManager::GetInstance()->y_velo_step;
//   }
// }

// void PACTMJSimulationBridge::_RightKeyAction()
// {
//   // Increase lateral velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->leftStickAnalogue[1] = -1.0;
//   }
//   else
//   {
//     UserInputManager::GetInstance()->v_des[1] -= UserInputManager::GetInstance()->y_velo_step;
//   }
// }

// void PACTMJSimulationBridge::_Comma_KeyAction()
// {
//   // Decrease yaw turn velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->rightStickAnalogue[1] = 1.0;
//   }
//   else
//   {
//     UserInputManager::GetInstance()->w_des[2] += UserInputManager::GetInstance()->yaw_velo_step;
//   }
// }

// void PACTMJSimulationBridge::_Slash_KeyAction()
// {
//   // Decrease yaw turn velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->rightStickAnalogue[1] = -1.0;
//   }
//   else
//   {
//     UserInputManager::GetInstance()->w_des[2] -= UserInputManager::GetInstance()->yaw_velo_step;
//   }
// }

// // Start Release Actions
// void PACTMJSimulationBridge::_UpKeyRelease()
// {
//   // Increase the forward velocity 
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->leftStickAnalogue[0] = 0.0;
//   }
// }

// void PACTMJSimulationBridge::_DownKeyRelease()
// {
//   // Decrease forward velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->leftStickAnalogue[0] = 0.0;
//   }
// }

// void PACTMJSimulationBridge::_LeftKeyRelease()
// {
//   // Decrease lateral velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->leftStickAnalogue[1] = 0.0;
//   }
// }

// void PACTMJSimulationBridge::_RightKeyRelease()
// {
//   // Increase lateral velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->leftStickAnalogue[1] = 0.0;
//   }
// }

// void PACTMJSimulationBridge::_Comma_KeyRelease()
// {
//   // Decrease yaw turn velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->rightStickAnalogue[1] = 0.0;
//   }
// }

// void PACTMJSimulationBridge::_Slash_KeyRelease()
// {
//   // Decrease yaw turn velocity
//   if (UserInputManager::GetInstance()->use_keyboard_ctrl)
//   {
//     UserInputManager::GetInstance()->rightStickAnalogue[1] = 0.0;
//   }
// }

// // Start Press Actions w/o overlap
// void PACTMJSimulationBridge::_K_KeyAction()
// {
//   // std::cout << "UserInputManager::GetInstance()->use_keyboard_ctrl - " << UserInputManager::GetInstance()->use_keyboard_ctrl << std::endl;
//   UserInputManager::GetInstance()->use_keyboard_ctrl = !UserInputManager::GetInstance()->use_keyboard_ctrl;
//   // std::cout << "UserInputManager::GetInstance()->use_keyboard_ctrl - " << UserInputManager::GetInstance()->use_keyboard_ctrl << std::endl;
// }

// void PACTMJSimulationBridge::_A_KeyAction()
// {
//   UserInputManager::GetInstance()->rpy_des[0] -= 0.02;
// }

// void PACTMJSimulationBridge::_D_KeyAction()
// {
//   UserInputManager::GetInstance()->rpy_des[0] += 0.02;
// }

// void PACTMJSimulationBridge::_W_KeyAction()
// {
//   UserInputManager::GetInstance()->rpy_des[1] -= 0.02;
// }

// void PACTMJSimulationBridge::_S_KeyAction()
// {
//   UserInputManager::GetInstance()->rpy_des[1] += 0.02;
// }
