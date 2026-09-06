/*! @file LinearKFObserver.cpp
 *  @brief Linear Kalman filter for torso position and velocity estimation.
 */

#include "LinearKFObserver.h"

#include <algorithm>
#include <cmath>

#include <ParamHandler/ParamHandler.hpp>

#include "Utils/orientation_tools.h"

template <typename T>
LinearKFObserver<T>::LinearKFObserver(mjData* data,
                                      const std::string& config_path,
                                      T dt,
                                      TorsoEstimatorSensorData* sensor_data)
    : _q(go1_kf::kNumPosDofs),
      _dq(go1_kf::kNumVelDofs),
      _mjData(data),
      _sensor_data(sensor_data),
      _dt(dt),
      _config_path(config_path) {
  printf("[LinearKFObserver] Constructed\n");
  setup();
}

template <typename T>
void LinearKFObserver<T>::setup() {
  ParamHandler param_handler(_config_path);
  if (_dt <= T(0)) {
    param_handler.getValue("estimator_dt", _dt);
  }

  fk_ = std::make_unique<rapid::robot::dynamics::ForwardKinematics<T>>(_config_path);

  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();

  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = _dt * Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();

  _B.setZero();
  _B.block(3, 0, 3, 3) = _dt * Eigen::Matrix<T, 3, 3>::Identity();

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();

  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(24, 8) = T(1);
  _C(25, 11) = T(1);
  _C(26, 14) = T(1);
  _C(27, 17) = T(1);

  _P.setIdentity();
  _P *= T(100);

  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (_dt / T(20)) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (_dt * T(9.8) / T(20)) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) = _dt * Eigen::Matrix<T, 12, 12>::Identity();

  _R0.setIdentity();

  param_handler.getValue("imu_process_noise_position", process_noise_pimu);
  param_handler.getValue("imu_process_noise_velocity", process_noise_vimu);
  param_handler.getValue("foot_process_noise_position", process_noise_pfoot);
  param_handler.getValue("foot_sensor_noise_position", sensor_noise_pimu_rel_foot);
  param_handler.getValue("foot_sensor_noise_velocity", sensor_noise_vimu_rel_foot);
  param_handler.getValue("foot_height_sensor_noise", sensor_noise_zfoot);
  param_handler.getValue("foot_airbag_contact_lower", sensor_contact_lower);
  if (!std::isfinite(sensor_contact_lower) || sensor_contact_lower <= T(0)) {
    sensor_contact_lower = T(15);
  }
}

template <typename T>
void LinearKFObserver<T>::Update() {
  for (int i = 0; i < go1_kf::kNumPosDofs; ++i) {
    _q[i] = static_cast<T>(_mjData->qpos[i]);
  }
  for (int i = 0; i < go1_kf::kNumVelDofs; ++i) {
    _dq[i] = static_cast<T>(_mjData->qvel[i]);
  }

  Vec12<T> jvel = Vec12<T>::Zero();
  for (int i = 0; i < go1_kf::kNumActuatedJoints; ++i) {
    jvel[i] = _dq[i + 6];
  }

  Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) =
      _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  Vec3<T> acc(static_cast<T>(_sensor_data->imu_acc[0]),
              static_cast<T>(_sensor_data->imu_acc[1]),
              static_cast<T>(_sensor_data->imu_acc[2]));
  Vec3<T> gyro(static_cast<T>(_sensor_data->imu_ang_vel[0]),
               static_cast<T>(_sensor_data->imu_ang_vel[1]),
               static_cast<T>(_sensor_data->imu_ang_vel[2]));

  Quat<T> body_quat(_q[3], _q[4], _q[5], _q[6]);
  Mat3<T> Rwb = ori::quaternionToRotationMatrix(body_quat).transpose();

  Vec3<T> gravity(0, 0, T(-9.81));
  Vec3<T> a_world = Rwb * acc + gravity;

  Vec4<T> pzs = Vec4<T>::Zero();
  Vec3<T> p0 = _xhat.template block<3, 1>(0, 0);
  Vec3<T> v0 = _xhat.template block<3, 1>(3, 0);

  for (int leg = 0; leg < 4; ++leg) {
    const int i1 = 3 * leg;
    const int qindex = 6 + i1;
    const int rindex1 = i1;
    const int rindex2 = 12 + i1;
    const int rindex3 = 24 + leg;

    Vec3<T> joints(_q[leg * 3 + 7], _q[leg * 3 + 8], _q[leg * 3 + 9]);
    Vec3<T> p_rel;
    Mat3<T> J;
    fk_->computeLegPositionAndJacobian(joints, p_rel, J, leg);

    Vec3<T> qd(jvel.segment(i1, 3));
    Vec3<T> qd_flipped(qd[0], -qd[1], -qd[2]);
    Vec3<T> foot_v = J * qd_flipped;

    Vec3<T> p_f = Rwb * p_rel;
    Vec3<T> dp_f = Rwb * (gyro.cross(p_rel) + foot_v);

    const T raw_contact = static_cast<T>(_sensor_data->contact_foot[leg]);
    const T trust = std::clamp(raw_contact / sensor_contact_lower, T(0), T(1));
    const T high_suspect_number = T(100);

    Q.block(qindex, qindex, 3, 3) *=
        (T(1) + (T(1) - trust) * high_suspect_number);
    R.block(rindex1, rindex1, 3, 3) *= T(1);
    R.block(rindex2, rindex2, 3, 3) *=
        (T(1) + (T(1) - trust) * high_suspect_number);
    R(rindex3, rindex3) *=
        (T(1) + (T(1) - trust) * high_suspect_number);

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (T(1) - trust) * v0 + trust * (-dp_f);
    pzs(leg) = (T(1) - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<T, 28, 1> y;
  y << _ps, _vs, pzs;

  _xhat = _A * _xhat + _B * a_world;
  const Eigen::Matrix<T, 18, 18> Pm = _A * _P * _A.transpose() + Q;

  const Eigen::Matrix<T, 28, 1> y_model = _C * _xhat;
  const Eigen::Matrix<T, 28, 1> ey = y - y_model;
  const Eigen::Matrix<T, 28, 28> S = _C * Pm * _C.transpose() + R;
  const Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);

  _xhat += Pm * _C.transpose() * S_ey;

  const Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * _C.transpose() * S_C) * Pm;
  _P = (_P + _P.transpose()) / T(2);

  if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) {
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= T(10);
  }
}

template <typename T>
Eigen::Matrix<T, 3, 1> LinearKFObserver<T>::GetEstimatedPosition() const {
  return _xhat.template block<3, 1>(0, 0);
}

template <typename T>
Eigen::Matrix<T, 3, 1> LinearKFObserver<T>::GetEstimatedVelocityWorld() const {
  return _xhat.template block<3, 1>(3, 0);
}

template <typename T>
Eigen::Matrix<T, 3, 1> LinearKFObserver<T>::GetEstimatedVelocityBody() const {
  const Quat<T> body_quat(_q[3], _q[4], _q[5], _q[6]);
  const Mat3<T> Rbw = ori::quaternionToRotationMatrix(body_quat);
  return Rbw * GetEstimatedVelocityWorld();
}

template class LinearKFObserver<float>;
template class LinearKFObserver<double>;
