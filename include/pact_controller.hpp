#pragma once

#include <array>
#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <deque>

#include "torch/script.h"
#include <torch/torch.h>

#include "gamepad.hpp"
#include "basic_controller.hpp"
#include "robot_interface.hpp"
#include "param/cfg_param.hpp"
#include "../3rd_parties/unitree_legged_sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"

namespace fs = std::filesystem;
using namespace std;
using namespace UNITREE_LEGGED_SDK;

float clip(float value, float low_limit, float high_limit)
{
    if(value <= low_limit) return low_limit;
    if(value >= high_limit) return high_limit;
    return value;
}

class PACTController : public BasicUserController
{
public:
    PACTController(const std::string& cfg_file)
    {
        // observation init to 0
        base_ang_vel.fill(0.0);
        projected_gravity.fill(0.);
        projected_gravity.at(2) = -1.0;
        cmd.fill(0.0);
        jpos_processed.fill(0.0);
        jvel.fill(0.0);
        pos_actions.fill(0.0);
        tau_actions.fill(0.0);
        config_file_name = cfg_file;
    }
    void loadParam()
    {
        PACTRLCfg cfg(config_file_name);
        dt = cfg.dt;
        stand_kp = cfg.stand_kp;
        stand_kd = cfg.stand_kd;
        pos_action_scale = cfg.pos_action_scale;
        tau_action_scale = cfg.tau_action_scale;
        
        lin_vel_scale = cfg.lin_vel_scale;
        ang_vel_scale = cfg.ang_vel_scale;
        dof_pos_scale = cfg.dof_pos_scale;
        dof_vel_scale = cfg.dof_vel_scale;

        action_alpha_pos = cfg.ema_alpha_pos;
        action_alpha_tau = cfg.ema_alpha_tau;

        obs_clip = cfg.clip_obs;
        act_clip = cfg.clip_actions;
        
        policy_name = cfg.policy_name;
        
        ctrl_kp = cfg.ctrl_kp;
        ctrl_kd = cfg.ctrl_kd;
        
        frame_stack = cfg.frame_stack;
        num_single_obs = cfg.num_single_obs;
        
        for (int i = 0; i < 12; ++i)
        {
            stand_pos.at(i) = cfg.stand_pos.at(i);
            sit_pos.at(i) = cfg.sit_pos.at(i);

            jpos_lim_min.at(i) = cfg.pos_limit_min.at(i);
            jpos_lim_max.at(i) = cfg.pos_limit_max.at(i);
            jtau_lim.at(i) = cfg.tau_limit.at(i);
        }


        std::cout << "\n===== OBS META =====\n";
        std::cout << "frame_stack: " << frame_stack << "\n";
        std::cout << "num_single_obs: " << num_single_obs << "\n";

        std::cout << "\n===== NORMALIZATION =====\n";
        std::cout << "lin_vel_scale: " << lin_vel_scale << "\n";
        std::cout << "ang_vel_scale: " << ang_vel_scale << "\n";
        std::cout << "dof_pos_scale: " << dof_pos_scale << "\n";
        std::cout << "dof_vel_scale: " << dof_vel_scale << "\n";

        std::cout << "\n===== CONTROL =====\n";
        std::cout << "pos_action_scale: " << pos_action_scale << "\n";
        std::cout << "tau_action_scale: " << tau_action_scale << "\n";
        std::cout << "obs_clip: " << obs_clip << "\n";
        std::cout << "act_clip: " << act_clip << "\n";

        std::cout << "\n===== ADDITIONAL PARAMS =====\n";
        std::cout << "dt: " << dt << "\n";
        std::cout << "stand_kp: " << stand_kp << "\n";
        std::cout << "stand_kd: " << stand_kd << "\n";
        std::cout << "ctrl_kp: " << ctrl_kp << "\n";
        std::cout << "ctrl_kd: " << ctrl_kd << "\n";
        std::cout << "action_alpha_pos: " << action_alpha_pos << "\n";
        std::cout << "action_alpha_tau: " << action_alpha_tau << "\n";
        std::cout << "policy_name: " << policy_name << "\n";

        // std::cout << "\n===== JOINT LIMITS / TARGETS =====\n";
        // print_array("jpos_lim_min", jpos_lim_min);
        // print_array("jpos_lim_max", jpos_lim_max);
        // print_array("jtau_lim", jtau_lim);

        std::cout << "=================================\n";

        // initialize history observation buffer
        // prepare history buffers
        single_step_obs.resize(num_single_obs);
        single_step_obs.clear();
        last_obs.resize(num_single_obs);
        last_obs.clear();
        for(int i = 0; i < num_single_obs; ++i)
        {
            single_step_obs.push_back(0.0);
            last_obs.push_back(0.0);
        }
        history_obs.resize(frame_stack);
        for(int i = 0; i < frame_stack; ++i)
        {
            history_obs.at(i).resize(num_single_obs);
            history_obs.at(i) = single_step_obs;
        }
    }
    void loadPolicy()
    {
        fs::path model_path = fs::current_path() / "../models";
        policy = torch::jit::load(model_path / policy_name);
        policy.eval();
        std::cout << "Load policy from: " << model_path / policy_name << std::endl;
    }
    
    void reset(BasicRobotInterface &robot_interface, Gamepad &gamepad)
    {
        // reset history observation buffer
        for(int i = 0; i < num_single_obs; ++i)
        {
            single_step_obs.at(i) = 0.0;
            last_obs.at(i) = 0.0;
        }
        for(int i = 0; i < frame_stack; ++i)
        {
            history_obs.at(i).resize(num_single_obs);
            history_obs.at(i) = last_obs;
        }
    }

    void GetInput(BasicRobotInterface &robot_interface, Gamepad &gamepad)
    {
        // save necessary data from input
        std::copy(robot_interface.gyro.begin(), robot_interface.gyro.end(), base_ang_vel.begin());
        std::copy(robot_interface.projected_gravity.begin(), robot_interface.projected_gravity.end(), projected_gravity.begin());
        // record command
        cmd.at(0) = gamepad.ly; // linear_x: [-1,1]
        cmd.at(1) = -gamepad.lx; // linear_y; [-1,1]
        cmd.at(2) = -gamepad.rx; // angular_z: [-1,1]
        
        // record robot state
        for (int i = 0; i < 12; ++i) // same index
        {
            jpos_processed.at(i) = robot_interface.jpos.at(i) - stand_pos.at(i);
            jvel.at(i) = robot_interface.jvel.at(i);
        }
        // for(int i = 0; i < 12; ++i) // real2sim index mapping
        // {
        //     jpos_processed.at(r2s_index_map[i]) = robot_interface.jpos.at(i) - stand_pos.at(i);
        //     jvel.at(r2s_index_map[i]) = robot_interface.jvel.at(i);
        // }
        
    }

    void DummyCalculate()
    {
        // Disable gradient tracking within this scope.
        torch::NoGradGuard no_grad; 
        torch::InferenceMode guard; // Enables inference mode
        // warmup the neural network
        for(int i = 0; i < num_single_obs; ++i)
        {
            last_obs.at(i) = 0.0;
            single_step_obs.at(i) = 0.0;
        }
        history_obs.pop_front();
        history_obs.push_back(single_step_obs);
        torch::Tensor obs_tensor = torch::zeros({1, num_single_obs});
        for(int i = 0; i < num_single_obs; i++)
            obs_tensor[0][i] = single_step_obs.at(i);
        torch::Tensor obs_his_tensor = torch::zeros({1, frame_stack*num_single_obs});
        for(int i = 0; i < frame_stack; ++i)
        {
            for(int j = 0; j < num_single_obs; ++j)
            {
                obs_his_tensor[0][i*num_single_obs + j] = history_obs.at(i).at(j);
            }
        }
        std::vector<torch::jit::IValue> policy_input;
        policy_input.push_back(obs_tensor);
        policy_input.push_back(obs_his_tensor);
        torch::Tensor policy_output_tensor = policy.get_method("act_inference")(policy_input).toTensor();
    }

    void Calculate()
    {
        torch::NoGradGuard no_grad; // Disable gradient tracking within this scope.
        torch::InferenceMode guard; // Enables inference mode
        // Fill observation
        fill_single_step_obs();
        // std::cout << "single_step_obs - " << single_step_obs << std::endl;
        // put current observation into history buffer
        history_obs.pop_front();
        history_obs.push_back(single_step_obs);
        
        torch::Tensor obs_tensor = torch::zeros({1, num_single_obs});
        for(int i = 0; i < num_single_obs; i++)
            obs_tensor[0][i] = single_step_obs.at(i);
        
            torch::Tensor obs_his_tensor = torch::zeros({1, frame_stack*num_single_obs});
        for(int i = 0; i < frame_stack; ++i)
        {
            for(int j = 0; j < num_single_obs; ++j)
            {
                obs_his_tensor[0][i*num_single_obs + j] = history_obs.at(i).at(j);
            }
        }
        
        std::vector<torch::jit::IValue> policy_input;
        policy_input.push_back(obs_tensor);
        policy_input.push_back(obs_his_tensor);
        
        torch::Tensor policy_output_tensor = policy.get_method("act_inference")(policy_input).toTensor();
        
        // // Initialize EMA on first call
        // if (!ema_initialized)
        // {
        //     for (int i = 0; i < 24; ++i)
        //         action_ema[i] = policy_output_tensor[0][i].item<float>();
        //     ema_initialized = true;
        // }
        // else
        // {
        //     for (int i = 0; i < 12; ++i)
        //     {
        //         float raw_pos = policy_output_tensor[0][i].item<float>();
        //         action_ema[i] = action_alpha_pos * raw_pos + (1.0f - action_alpha_pos) * action_ema[i];
        //         pos_actions.at(i) = clip(raw_pos, -act_clip, act_clip);

        //         float raw_tau = policy_output_tensor[0][i+12].item<float>();
        //         action_ema[i+12] = action_alpha_tau * raw_tau + (1.0f - action_alpha_tau) * action_ema[i+12];
        //         tau_actions.at(i) = clip(raw_tau, -act_clip, act_clip);
        //     }
        // }

        
        // std::array<float, 12> temp_tau_actions;
        // std::array<float, 12> temp_pos_actions;
        // std::array<float, 12> pos_actions_scaled;

        // // Clip the raw action prediction and the resulting joint pose/torque targets
        // for(int i = 0; i < 12; ++i)
        // {
        //     // pos_actions.at(i) = clip(policy_output_tensor[0][i].item<float>(), -act_clip, act_clip);
        //     temp_pos_actions.at(i) = clip(action_ema[i], -act_clip, act_clip);
        //     pos_actions_scaled.at(i) = temp_pos_actions.at(i) * pos_action_scale;
        //     jpos_des.at(i) = clip(pos_actions_scaled.at(i) + stand_pos.at(i), jpos_lim_min.at(i), jpos_lim_max.at(i));

        //     // tau_actions.at(i) = clip(policy_output_tensor[0][i+12].item<float>(), -act_clip, act_clip);
        //     temp_tau_actions.at(i) = clip(action_ema[i+12], -act_clip, act_clip);
        //     jtau_des.at(i) = clip(temp_tau_actions.at(i) * tau_action_scale, -jtau_lim.at(i), jtau_lim.at(i));
        // }


        for(int i = 0; i < 12; ++i)
        {
            pos_actions.at(i) = clip(policy_output_tensor[0][i].item<float>(), -act_clip, act_clip);
            // pos_actions_scaled.at(i) = pos_actions.at(i) * pos_action_scale;
            jpos_des.at(i) = clip(pos_actions.at(i) * pos_action_scale + stand_pos.at(i), jpos_lim_min.at(i), jpos_lim_max.at(i));

            tau_actions.at(i) = clip(policy_output_tensor[0][i+12].item<float>(), -act_clip, act_clip);
            jtau_des.at(i) = clip(tau_actions.at(i) * tau_action_scale, -jtau_lim.at(i), jtau_lim.at(i));
        }

        jvel_des.fill(0.0);

        // // std::cout << "jpos_des: " << jpos_des << std::endl;
        // // std::cout << "jtau_des: " << jtau_des << std::endl;
        // std::cout << "jpos_des elements: ";
        // for (const auto& element : jpos_des) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;
        // // 
        // std::cout << "jtau_des elements: ";
        // for (const auto& element : jtau_des) {
        //     std::cout << element << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "-------------------------------------------" << std::endl;
    }
    std::vector<float> GetLog()
    {
        // record input, output and other info into a vector
        std::vector<float> log;
        log.push_back(cmd.at(0));
        log.push_back(cmd.at(1));
        log.push_back(cmd.at(2));
        return log;
    }

    // real2sim joint index mapping, for policies trained in IsaacGym
    // real: 0-FR_hip, 1-FR_thigh, 2-FR_calf,
    //       3-FL_hip, 4-FL_thigh, 5-FL_calf,
    //       6-RR_hip, 7-RR_thigh, 8-RR_calf,
    //       9-RL_hip, 10-RL_thigh, 11-RL_calf
    // sim:  0-FL_hip, 1-FL_thigh, 2-FL_calf,
    //       3-FR_hip, 4-FR_thigh, 5-FR_calf,
    //       6-RL_hip, 7-RL_thigh, 8-RL_calf,
    //       9-RR_hip, 10-RR_thigh, 11-RR_calf
    
    // PACT genesis training joint order
    // dof_names = [        # specify the sequence of actions
            // 'FR_hip_joint',
            // 'FR_thigh_joint',
            // 'FR_calf_joint',
            // 'FL_hip_joint',
            // 'FL_thigh_joint',
            // 'FL_calf_joint',
            // 'RR_hip_joint',
            // 'RR_thigh_joint',
            // 'RR_calf_joint',
            // 'RL_hip_joint',
            // 'RL_thigh_joint',
            // 'RL_calf_joint',]
    uint16_t r2s_index_map[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

    std::string config_file_name;
    
    // observation
    std::array<float, 3> base_ang_vel;
    std::array<float, 3> projected_gravity;
    std::array<float, 3> cmd;
    std::array<float, 12> jpos_processed;       //joint sequence: sim
    std::array<float, 12> jvel;
    std::array<float, 12> pos_actions;
    std::array<float, 12> tau_actions;
    
    int frame_stack;
    
    int num_single_obs; // length of single step observation
    
    std::vector<float> single_step_obs; // 用于记录单步观测数据
    std::vector<float> last_obs;        // 用于记录上一步的观测数据
    
    std::deque<std::vector<float>> history_obs; // 用于记录历史观测数据

    // normalization parameters
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    // control params
    float pos_action_scale;
    float tau_action_scale;

    float obs_clip;
    float act_clip;

    // Smoothing factor (0 = no update, 1 = no smoothing)
    float action_alpha_pos;
    float action_alpha_tau;  

    std::array<float, 24> action_ema;   // 12 pos + 12 torque
    bool ema_initialized = false;

    // NN model
    std::string policy_name;
    torch::jit::script::Module policy;

private:
    void fill_single_step_obs()
    {
        // save last observation
        std::copy(single_step_obs.begin(), single_step_obs.end(), last_obs.begin());
        // update current observation
        single_step_obs.at(0) = clip(cmd.at(0) * lin_vel_scale, -obs_clip, obs_clip);
        single_step_obs.at(1) = clip(cmd.at(1) * lin_vel_scale, -obs_clip, obs_clip);
        single_step_obs.at(2) = clip(cmd.at(2) * ang_vel_scale, -obs_clip, obs_clip); // 3
        for(int i = 0; i < 3; ++i)
        {
            single_step_obs.at(i+3) = clip(projected_gravity.at(i), -obs_clip, obs_clip);
            single_step_obs.at(i+6) = clip(base_ang_vel.at(i) * ang_vel_scale, -obs_clip, obs_clip);
        }                                                  // 6, 9
        for(int i = 0; i < 12; ++i)
        {
            single_step_obs.at(i+9) = clip(jpos_processed.at(i) * dof_pos_scale, -obs_clip, obs_clip);
            single_step_obs.at(i+21) = clip(jvel.at(i) * dof_vel_scale, -obs_clip, obs_clip);
            single_step_obs.at(i+33) = clip(pos_actions.at(i), -obs_clip, obs_clip);
            single_step_obs.at(i+45) = clip(tau_actions.at(i), -obs_clip, obs_clip);
        }                                                  // 21, 33, 45, 57
    }
};