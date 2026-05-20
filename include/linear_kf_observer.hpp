#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "../3rd_parties/unitree_legged_sdk/include/unitree_legged_sdk/comm.h"

class LinearKFObserver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LinearKFObserver(const std::string &config_file_path, float dt)
        : _dt(dt)
    {
        LoadParams(config_file_path);
        Setup();
    }

    void Update(const UNITREE_LEGGED_SDK::LowState &state)
    {
        constexpr int kNumLegs = 4;
        constexpr int kJointsPerLeg = 3;
        constexpr int kNumActuatedJoints = 12;

        Eigen::Matrix<float, 12, 1> joint_vel = Eigen::Matrix<float, 12, 1>::Zero();
        for (int i = 0; i < kNumActuatedJoints; ++i)
        {
            joint_vel[i] = state.motorState.at(i).dq;
        }

        Eigen::Matrix<float, 18, 18> Q = Eigen::Matrix<float, 18, 18>::Identity();
        Q.block<3, 3>(0, 0) = _Q0.block<3, 3>(0, 0) * _process_noise_pimu;
        Q.block<3, 3>(3, 3) = _Q0.block<3, 3>(3, 3) * _process_noise_vimu;
        Q.block<12, 12>(6, 6) = _Q0.block<12, 12>(6, 6) * _process_noise_pfoot;

        Eigen::Matrix<float, 28, 28> R = Eigen::Matrix<float, 28, 28>::Identity();
        R.block<12, 12>(0, 0) = _R0.block<12, 12>(0, 0) * _sensor_noise_pimu_rel_foot;
        R.block<12, 12>(12, 12) = _R0.block<12, 12>(12, 12) * _sensor_noise_vimu_rel_foot;
        R.block<4, 4>(24, 24) = _R0.block<4, 4>(24, 24) * _sensor_noise_zfoot;

        const Eigen::Vector3f acc(state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]);
        const Eigen::Vector3f gyro(state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);

        Eigen::Quaternionf body_quat(state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3]);
        if (body_quat.norm() <= 1e-6f)
        {
            body_quat = Eigen::Quaternionf::Identity();
        }
        else
        {
            body_quat.normalize();
        }

        _Rbw = body_quat.toRotationMatrix();
        const Eigen::Matrix3f Rwb = _Rbw.transpose();
        const Eigen::Vector3f gravity(0.0f, 0.0f, -9.81f);
        const Eigen::Vector3f a_world = Rwb * acc + gravity;

        Eigen::Matrix<float, 4, 1> foot_heights = Eigen::Matrix<float, 4, 1>::Zero();
        const Eigen::Vector3f base_pos = _xhat.block<3, 1>(0, 0);
        const Eigen::Vector3f base_vel = _xhat.block<3, 1>(3, 0);

        for (int leg = 0; leg < kNumLegs; ++leg)
        {
            const int i1 = kJointsPerLeg * leg;
            const int q_index = 6 + i1;
            const int r_index_1 = i1;
            const int r_index_2 = 12 + i1;
            const int r_index_3 = 24 + leg;

            const Eigen::Vector3f joints(
                state.motorState.at(i1 + 0).q,
                state.motorState.at(i1 + 1).q,
                state.motorState.at(i1 + 2).q);

            Eigen::Vector3f p_rel = Eigen::Vector3f::Zero();
            Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
            ComputeLegPositionAndJacobian(joints, leg, p_rel, J);

            const Eigen::Vector3f qd(joint_vel.segment<3>(i1));
            const Eigen::Vector3f qd_flipped(qd[0], -qd[1], -qd[2]);
            const Eigen::Vector3f foot_v = J * qd_flipped;

            const Eigen::Vector3f p_f = Rwb * p_rel;
            const Eigen::Vector3f dp_f = Rwb * (gyro.cross(p_rel) + foot_v);

            const float raw_contact = static_cast<float>(state.footForce.at(leg));
            const float trust = std::clamp(raw_contact / _sensor_airbag_contact_lower, 0.0f, 1.0f);
            constexpr float kHighSuspectNumber = 100.0f;

            Q.block<3, 3>(q_index, q_index) *= (1.0f + (1.0f - trust) * kHighSuspectNumber);
            R.block<3, 3>(r_index_2, r_index_2) *= (1.0f + (1.0f - trust) * kHighSuspectNumber);
            R(r_index_3, r_index_3) *= (1.0f + (1.0f - trust) * kHighSuspectNumber);

            _ps.segment<3>(i1) = -p_f;
            _vs.segment<3>(i1) = (1.0f - trust) * base_vel + trust * (-dp_f);
            foot_heights(leg) = (1.0f - trust) * (base_pos.z() + p_f.z());
        }

        Eigen::Matrix<float, 28, 1> y;
        y << _ps, _vs, foot_heights;

        _xhat = _A * _xhat + _B * a_world;
        const Eigen::Matrix<float, 18, 18> predicted_cov = _A * _P * _A.transpose() + Q;

        const Eigen::Matrix<float, 28, 1> innovation = y - _C * _xhat;
        const Eigen::Matrix<float, 28, 28> innovation_cov = _C * predicted_cov * _C.transpose() + R;
        const Eigen::Matrix<float, 28, 1> solved_innovation = innovation_cov.lu().solve(innovation);

        _xhat += predicted_cov * _C.transpose() * solved_innovation;

        const Eigen::Matrix<float, 28, 18> solved_C = innovation_cov.lu().solve(_C);
        _P = (Eigen::Matrix<float, 18, 18>::Identity() - predicted_cov * _C.transpose() * solved_C) * predicted_cov;
        _P = (_P + _P.transpose()) * 0.5f;

        if (_P.block<2, 2>(0, 0).determinant() > 1e-6f)
        {
            _P.block<2, 16>(0, 2).setZero();
            _P.block<16, 2>(2, 0).setZero();
            _P.block<2, 2>(0, 0) /= 10.0f;
        }
    }

    std::array<float, 3> GetEstimatedPosition() const
    {
        return ToArray(_xhat.block<3, 1>(0, 0));
    }

    std::array<float, 3> GetEstimatedVelocityWorld() const
    {
        return ToArray(_xhat.block<3, 1>(3, 0));
    }

    std::array<float, 3> GetEstimatedVelocityBody() const
    {
        return ToArray(_Rbw * _xhat.block<3, 1>(3, 0));
    }

private:
    void LoadParams(const std::string &config_file_path)
    {
        try
        {
            const YAML::Node cfg = YAML::LoadFile(config_file_path);
            LoadOptional(cfg, "estimator_dt", _dt);
            LoadOptional(cfg, "imu_process_noise_position", _process_noise_pimu);
            LoadOptional(cfg, "imu_process_noise_velocity", _process_noise_vimu);
            LoadOptional(cfg, "foot_process_noise_position", _process_noise_pfoot);
            LoadOptional(cfg, "foot_sensor_noise_position", _sensor_noise_pimu_rel_foot);
            LoadOptional(cfg, "foot_sensor_noise_velocity", _sensor_noise_vimu_rel_foot);
            LoadOptional(cfg, "foot_height_sensor_noise", _sensor_noise_zfoot);
            LoadOptional(cfg, "foot_airbag_contact_lower", _sensor_airbag_contact_lower);

            LoadOptional(cfg, "go1_body_length", _body_length);
            LoadOptional(cfg, "go1_body_width", _body_width);
            LoadOptional(cfg, "go1_abad_link_length", _abad_link_length);
            LoadOptional(cfg, "go1_hip_link_length", _hip_link_length);
            LoadOptional(cfg, "go1_knee_link_length", _knee_link_length);
            LoadOptional(cfg, "go1_knee_link_y_offset", _knee_link_y_offset);
        }
        catch (const std::exception &)
        {
        }
    }

    void Setup()
    {
        _xhat.setZero();
        _ps.setZero();
        _vs.setZero();

        _A.setZero();
        _A.block<3, 3>(0, 0).setIdentity();
        _A.block<3, 3>(0, 3) = _dt * Eigen::Matrix3f::Identity();
        _A.block<3, 3>(3, 3).setIdentity();
        _A.block<12, 12>(6, 6).setIdentity();

        _B.setZero();
        _B.block<3, 3>(3, 0) = _dt * Eigen::Matrix3f::Identity();

        Eigen::Matrix<float, 3, 6> C1 = Eigen::Matrix<float, 3, 6>::Zero();
        C1.block<3, 3>(0, 0).setIdentity();
        Eigen::Matrix<float, 3, 6> C2 = Eigen::Matrix<float, 3, 6>::Zero();
        C2.block<3, 3>(0, 3).setIdentity();

        _C.setZero();
        _C.block<3, 6>(0, 0) = C1;
        _C.block<3, 6>(3, 0) = C1;
        _C.block<3, 6>(6, 0) = C1;
        _C.block<3, 6>(9, 0) = C1;
        _C.block<12, 12>(0, 6) = -Eigen::Matrix<float, 12, 12>::Identity();
        _C.block<3, 6>(12, 0) = C2;
        _C.block<3, 6>(15, 0) = C2;
        _C.block<3, 6>(18, 0) = C2;
        _C.block<3, 6>(21, 0) = C2;
        _C(24, 8) = 1.0f;
        _C(25, 11) = 1.0f;
        _C(26, 14) = 1.0f;
        _C(27, 17) = 1.0f;

        _P.setIdentity();
        _P *= 100.0f;

        _Q0.setIdentity();
        _Q0.block<3, 3>(0, 0) = (_dt / 20.0f) * Eigen::Matrix3f::Identity();
        _Q0.block<3, 3>(3, 3) = (_dt * 9.8f / 20.0f) * Eigen::Matrix3f::Identity();
        _Q0.block<12, 12>(6, 6) = _dt * Eigen::Matrix<float, 12, 12>::Identity();

        _R0.setIdentity();
        _Rbw.setIdentity();
    }

    void ComputeLegPositionAndJacobian(const Eigen::Vector3f &q,
                                       int leg,
                                       Eigen::Vector3f &p,
                                       Eigen::Matrix3f &J) const
    {
        const float side_sign = (leg == 0 || leg == 2) ? -1.0f : 1.0f;
        const float s1 = std::sin(q[0]);
        const float s2 = std::sin(-q[1]);
        const float s3 = std::sin(-q[2]);
        const float c1 = std::cos(q[0]);
        const float c2 = std::cos(-q[1]);
        const float c3 = std::cos(-q[2]);
        const float c23 = c2 * c3 - s2 * s3;
        const float s23 = s2 * c3 + c2 * s3;

        p[0] = _knee_link_length * s23 + _hip_link_length * s2;
        p[1] = (_abad_link_length + _knee_link_y_offset) * side_sign * c1 + _knee_link_length * (s1 * c23) + _hip_link_length * c2 * s1;
        p[2] = (_abad_link_length + _knee_link_y_offset) * side_sign * s1 - _knee_link_length * (c1 * c23) - _hip_link_length * c1 * c2;
        p += GetHipLocation(leg);

        J(0, 0) = 0.0f;
        J(0, 1) = _knee_link_length * c23 + _hip_link_length * c2;
        J(0, 2) = _knee_link_length * c23;
        J(1, 0) = _knee_link_length * c1 * c23 + _hip_link_length * c1 * c2 - (_abad_link_length + _knee_link_y_offset) * side_sign * s1;
        J(1, 1) = -_knee_link_length * s1 * s23 - _hip_link_length * s1 * s2;
        J(1, 2) = -_knee_link_length * s1 * s23;
        J(2, 0) = _knee_link_length * s1 * c23 + _hip_link_length * c2 * s1 + (_abad_link_length + _knee_link_y_offset) * side_sign * c1;
        J(2, 1) = _knee_link_length * c1 * s23 + _hip_link_length * c1 * s2;
        J(2, 2) = _knee_link_length * c1 * s23;
    }

    Eigen::Vector3f GetHipLocation(int leg) const
    {
        return Eigen::Vector3f(
            (leg == 0 || leg == 1) ? 0.5f * _body_length : -0.5f * _body_length,
            (leg == 1 || leg == 3) ? 0.5f * _body_width : -0.5f * _body_width,
            0.0f);
    }

    static std::array<float, 3> ToArray(const Eigen::Vector3f &vec)
    {
        return {vec[0], vec[1], vec[2]};
    }

    template <typename T>
    static void LoadOptional(const YAML::Node &cfg, const char *key, T &value)
    {
        if (cfg[key])
        {
            value = cfg[key].as<T>();
        }
    }

    Eigen::Matrix<float, 18, 1> _xhat = Eigen::Matrix<float, 18, 1>::Zero();
    Eigen::Matrix<float, 12, 1> _ps = Eigen::Matrix<float, 12, 1>::Zero();
    Eigen::Matrix<float, 12, 1> _vs = Eigen::Matrix<float, 12, 1>::Zero();
    Eigen::Matrix<float, 18, 18> _A = Eigen::Matrix<float, 18, 18>::Zero();
    Eigen::Matrix<float, 18, 18> _Q0 = Eigen::Matrix<float, 18, 18>::Zero();
    Eigen::Matrix<float, 18, 18> _P = Eigen::Matrix<float, 18, 18>::Zero();
    Eigen::Matrix<float, 28, 28> _R0 = Eigen::Matrix<float, 28, 28>::Zero();
    Eigen::Matrix<float, 18, 3> _B = Eigen::Matrix<float, 18, 3>::Zero();
    Eigen::Matrix<float, 28, 18> _C = Eigen::Matrix<float, 28, 18>::Zero();
    Eigen::Matrix3f _Rbw = Eigen::Matrix3f::Identity();

    float _dt = 0.01f;

    float _process_noise_pimu = 0.02f;
    float _process_noise_vimu = 0.02f;
    float _process_noise_pfoot = 0.002f;
    float _sensor_noise_pimu_rel_foot = 0.001f;
    float _sensor_noise_vimu_rel_foot = 0.1f;
    float _sensor_noise_zfoot = 0.001f;
    float _sensor_airbag_contact_lower = 15.0f;

    float _body_length = 0.3762f;
    float _body_width = 0.0935f;
    float _abad_link_length = 0.08f;
    float _hip_link_length = 0.213f;
    float _knee_link_length = 0.223f;
    float _knee_link_y_offset = 0.0f;
};
