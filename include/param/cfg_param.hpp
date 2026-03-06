#pragma once

#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

class BaseCfg
{
public:
    BaseCfg(const std::string &filename){};

};

class PACTRLCfg : public BaseCfg
{
public:
    PACTRLCfg(const std::string &filename) : BaseCfg(filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            policy_name = cfg["policy_name"].as<std::string>();
            dt = cfg["dt"].as<float>();
            
            stand_kp = cfg["stand_kp"].as<float>();
            stand_kd = cfg["stand_kd"].as<float>();
            ctrl_kp = cfg["ctrl_kp"].as<float>();
            ctrl_kd = cfg["ctrl_kd"].as<float>();
            
            pos_action_scale = cfg["pos_action_scale"].as<float>();
            tau_action_scale = cfg["tau_action_scale"].as<float>();
            
            lin_vel_scale = cfg["lin_vel_scale"].as<float>();
            ang_vel_scale = cfg["ang_vel_scale"].as<float>();
            dof_pos_scale = cfg["dof_pos_scale"].as<float>();
            dof_vel_scale = cfg["dof_vel_scale"].as<float>();
            clip_actions = cfg["clip_actions"].as<float>();
            clip_obs = cfg["clip_observations"].as<float>();

            ema_alpha_pos = cfg["ema_alpha_pos"].as<float>();
            ema_alpha_tau = cfg["ema_alpha_tau"].as<float>();

            
            num_single_obs = cfg["num_single_obs"].as<int>();
            frame_stack = cfg["frame_stack"].as<int>();
            
            for(const auto& v : cfg["stand_pos"])
            {
                stand_pos.push_back(v.as<float>());
            }
            for(const auto& v : cfg["sit_pos"])
            {
                sit_pos.push_back(v.as<float>());
            }
            
            for(const auto& v : cfg["joint_limit_max"])
            {
                pos_limit_max.push_back(v.as<float>());
            }
            for(const auto& v : cfg["joint_limit_min"])
            {
                pos_limit_min.push_back(v.as<float>());
            }
            for(const auto& v : cfg["joint_tau_max"])
            {
                tau_limit.push_back(v.as<float>());
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
    
    float stand_kp;
    float stand_kd;
    float ctrl_kp;
    float ctrl_kd;
    float dt;
    float pos_action_scale;
    float tau_action_scale;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_actions;
    float clip_obs;
    float ema_alpha_pos;
    float ema_alpha_tau;
    int num_single_obs;
    int frame_stack;
    std::string policy_name;
    std::vector<float> stand_pos;
    std::vector<float> sit_pos;
    std::vector<float> pos_limit_min;
    std::vector<float> pos_limit_max;
    std::vector<float> tau_limit;
};