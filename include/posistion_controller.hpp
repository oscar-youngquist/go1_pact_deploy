#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include "basic_controller.hpp"

// The filename retains the spelling used by the existing repository.
// Joint order and observation normalization match PACTController.
class PositionController : public BasicUserController
{
public:
    explicit PositionController(const std::string& config_file)
        : config_file_(config_file), cfg_(config_file) {}

    void loadParam() override
    {
        if ((cfg_.num_single_obs != 45 && cfg_.num_single_obs != 57) ||
            cfg_.frame_stack <= 0 || !std::isfinite(cfg_.dt) || cfg_.dt <= 0)
            throw std::invalid_argument("Position policy requires 45 or 57 observations, positive frame_stack and dt");
        if (cfg_.stand_pos.size() != 12 || cfg_.sit_pos.size() != 12 ||
            cfg_.pos_limit_min.size() != 12 || cfg_.pos_limit_max.size() != 12 ||
            cfg_.tau_limit.size() != 12)
            throw std::invalid_argument("Position policy joint configuration must contain 12 values per array");

        dt = cfg_.dt;
        stand_kp = cfg_.stand_kp;
        stand_kd = cfg_.stand_kd;
        ctrl_kp = cfg_.ctrl_kp;
        ctrl_kd = cfg_.ctrl_kd;
        std::copy_n(cfg_.stand_pos.begin(), 12, stand_pos.begin());
        std::copy_n(cfg_.sit_pos.begin(), 12, sit_pos.begin());
        std::copy_n(cfg_.pos_limit_min.begin(), 12, jpos_lim_min.begin());
        std::copy_n(cfg_.pos_limit_max.begin(), 12, jpos_lim_max.begin());
        std::copy_n(cfg_.tau_limit.begin(), 12, jtau_lim.begin());
        ClearHistory();
        std::cout << "Position policy: " << cfg_.num_single_obs << " observations, "
                  << cfg_.frame_stack << " frames, 12 position outputs, dt=" << dt << std::endl;
    }

    void loadPolicy() override
    {
        namespace fs = std::filesystem;
        fs::path model_path(cfg_.policy_name);
        if (model_path.is_relative())
        {
            // params lives in the nested deploy repo; models lives in the outer sim repo.
            const auto params = fs::absolute(config_file_).lexically_normal().parent_path();
            model_path = params.parent_path().parent_path() / "models" / model_path;
        }
        policy = torch::jit::load(model_path.string(), torch::kCPU);
        policy.eval();
        DummyCalculate(); // Validate the exported input/output contract before control starts.
        std::cout << "Loaded position policy: " << model_path << std::endl;
    }

    void reset(BasicRobotInterface&, Gamepad&) override { ClearHistory(); }

    void GetInput(BasicRobotInterface& robot, Gamepad& gamepad) override
    {
        cmd = {-gamepad.ly, -gamepad.lx, -gamepad.rx};
        gravity_ = robot.projected_gravity;
        gyro_ = robot.gyro;
        for (size_t i = 0; i < 12; ++i)
        {
            joint_offset_[i] = robot.jpos[i] - stand_pos[i];
            joint_velocity_[i] = robot.jvel[i];
        }
    }

    void DummyCalculate() override
    {
        torch::InferenceMode guard;
        ValidateOutput(policy.forward({
            torch::zeros({1, cfg_.num_single_obs}),
            torch::zeros({1, cfg_.frame_stack * cfg_.num_single_obs})}).toTensor());
    }

    void Calculate() override
    {
        torch::InferenceMode guard;
        FillObservation();
        history_obs.pop_front();
        history_obs.push_back(single_step_obs); // Include the current frame, as in PACT.
        auto obs = torch::tensor(single_step_obs).unsqueeze(0);
        std::vector<float> flat_history;
        flat_history.reserve(cfg_.frame_stack * cfg_.num_single_obs);
        for (const auto& frame : history_obs)
            flat_history.insert(flat_history.end(), frame.begin(), frame.end());
        auto history = torch::tensor(flat_history).unsqueeze(0);
        const auto output = policy.forward({obs, history}).toTensor();
        ValidateOutput(output);
        // Match PACT's direct clipped actions (its EMA settings are currently inactive).
        for (size_t i = 0; i < 12; ++i)
        {
            pos_actions[i] = std::clamp(output[0][i].item<float>(),
                                        -cfg_.clip_actions, cfg_.clip_actions);
            jpos_des[i] = std::clamp(stand_pos[i] + cfg_.pos_action_scale * pos_actions[i],
                                     jpos_lim_min[i], jpos_lim_max[i]);
        }
        jvel_des.fill(0.0f);
        jtau_des.fill(0.0f);
    }

    std::vector<float> GetLog() override { return {cmd[0], cmd[1], cmd[2]}; }

    std::array<float, 3> cmd{};
    std::array<float, 12> pos_actions{};
    std::vector<float> single_step_obs;
    std::deque<std::vector<float>> history_obs;
    torch::jit::script::Module policy;

private:
    static void ValidateOutput(const torch::Tensor& output)
    {
        if (output.dim() != 2 || output.size(0) != 1 || output.size(1) != 12 ||
            !output.is_floating_point() || !torch::isfinite(output).all().item<bool>())
            throw std::runtime_error("Position forward must return a finite [1, 12] tensor");
    }

    void ClearHistory()
    {
        pos_actions.fill(0.0f);
        single_step_obs.assign(cfg_.num_single_obs, 0.0f);
        history_obs.assign(cfg_.frame_stack, single_step_obs);
        jpos_des = stand_pos;
        jvel_des.fill(0.0f);
        jtau_des.fill(0.0f);
    }

    void FillObservation()
    {
        auto clip = [this](float value) {
            return std::clamp(value, -cfg_.clip_obs, cfg_.clip_obs);
        };
        single_step_obs[0] = clip(cmd[0] * cfg_.lin_vel_scale);
        single_step_obs[1] = clip(cmd[1] * cfg_.lin_vel_scale);
        single_step_obs[2] = clip(cmd[2] * cfg_.ang_vel_scale);
        for (size_t i = 0; i < 3; ++i)
        {
            single_step_obs[i + 3] = clip(gravity_[i]);
            single_step_obs[i + 6] = clip(gyro_[i] * cfg_.ang_vel_scale);
        }
        for (size_t i = 0; i < 12; ++i)
        {
            single_step_obs[i + 9] = clip(joint_offset_[i] * cfg_.dof_pos_scale);
            single_step_obs[i + 21] = clip(joint_velocity_[i] * cfg_.dof_vel_scale);
            single_step_obs[i + 33] = clip(pos_actions[i]);
        }
        // Only for exports explicitly trained with zero-padded PACT observations.
        // No torque prediction or synthetic PD-torque feedback is added.
        if (cfg_.num_single_obs == 57)
            std::fill(single_step_obs.begin() + 45, single_step_obs.end(), 0.0f);
    }

    std::string config_file_;
    PositionRLCfg cfg_;
    std::array<float, 3> gravity_{0.0f, 0.0f, -1.0f}, gyro_{};
    std::array<float, 12> joint_offset_{}, joint_velocity_{};
};
