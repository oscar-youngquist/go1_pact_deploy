/*! @file LinearKFObserver.h
 *  @brief Linear Kalman filter for torso position and velocity estimation.
 */

#ifndef PROJECT_LINEARKF_OBSERVER_H
#define PROJECT_LINEARKF_OBSERVER_H

#include <array>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include "Utils/cppTypes.h"
#include "forward_kinematics.h"

namespace go1_kf {
constexpr int kNumActuatedJoints = 12;
constexpr int kNumVelDofs = 18;
constexpr int kNumPosDofs = 19;
}  // namespace go1_kf

struct TorsoEstimatorSensorData {
  std::array<double, 3> imu_acc{};
  std::array<double, 3> imu_ang_vel{};
  std::array<double, 4> contact_foot{};
};

template <typename T>
class LinearKFObserver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LinearKFObserver(mjData* data,
                   const std::string& config_path,
                   T dt,
                   TorsoEstimatorSensorData* sensor_data);

  void Update();
  void setup();

  Eigen::Matrix<T, 3, 1> GetEstimatedPosition() const;
  Eigen::Matrix<T, 3, 1> GetEstimatedVelocityWorld() const;
  Eigen::Matrix<T, 3, 1> GetEstimatedVelocityBody() const;

 private:
  Eigen::Matrix<T, 18, 1> _xhat;
  Eigen::Matrix<T, 12, 1> _ps;
  Eigen::Matrix<T, 12, 1> _vs;
  Eigen::Matrix<T, 18, 18> _A;
  Eigen::Matrix<T, 18, 18> _Q0;
  Eigen::Matrix<T, 18, 18> _P;
  Eigen::Matrix<T, 28, 28> _R0;
  Eigen::Matrix<T, 18, 3> _B;
  Eigen::Matrix<T, 28, 18> _C;

  std::unique_ptr<rapid::robot::dynamics::ForwardKinematics<T>> fk_ = nullptr;
  DVec<T> _q;
  DVec<T> _dq;

  mjData* _mjData = nullptr;
  TorsoEstimatorSensorData* _sensor_data = nullptr;

  T _dt = T(0);
  std::string _config_path;

  T process_noise_pimu = T(0);
  T process_noise_vimu = T(0);
  T process_noise_pfoot = T(0);
  T sensor_noise_pimu_rel_foot = T(0);
  T sensor_noise_vimu_rel_foot = T(0);
  T sensor_noise_zfoot = T(0);
  T sensor_contact_lower = T(15);
};

#endif  // PROJECT_LINEARKF_OBSERVER_H
