#pragma once

#include <cassert>
#include <cmath>
#include <string>

#include <ParamHandler/ParamHandler.hpp>

#include "Utils/cppTypes.h"

namespace rapid::robot::dynamics {

template <typename T>
class ForwardKinematics {
 public:
  explicit ForwardKinematics(const std::string& config_name) {
    ParamHandler param_handler(config_name);
    param_handler.getValue("go1_body_length", _bodyLength);
    param_handler.getValue("go1_body_width", _bodyWidth);
    param_handler.getValue("go1_abad_link_length", _abadLinkLength);
    param_handler.getValue("go1_hip_link_length", _hipLinkLength);
    param_handler.getValue("go1_knee_link_length", _kneeLinkLength);
    param_handler.getValue("go1_knee_link_y_offset", _kneeLinkY_offset);
    _abadLocation = Vec3<T>(_bodyLength * T(0.5), _bodyWidth * T(0.5), T(0));
  }

  void computeLegPositionAndJacobian(const Vec3<T>& q, Vec3<T>& p, Mat3<T>& J, int leg) const {
    assert(leg >= 0 && leg < 4);

    const T sideSign = (leg == 0 || leg == 2) ? T(-1) : T(1);
    const T s1 = std::sin(q(0));
    const T s2 = std::sin(-q(1));
    const T s3 = std::sin(-q(2));
    const T c1 = std::cos(q(0));
    const T c2 = std::cos(-q(1));
    const T c3 = std::cos(-q(2));
    const T c23 = c2 * c3 - s2 * s3;
    const T s23 = s2 * c3 + c2 * s3;
    const T l1 = _abadLinkLength;
    const T l2 = _hipLinkLength;
    const T l3 = _kneeLinkLength;
    const T l4 = _kneeLinkY_offset;

    p[0] = l3 * s23 + l2 * s2;
    p[1] = (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p[2] = (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
    p += getHipLocation(leg);

    J(0, 0) = T(0);
    J(0, 1) = l3 * c23 + l2 * c2;
    J(0, 2) = l3 * c23;
    J(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
    J(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J(1, 2) = -l3 * s1 * s23;
    J(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
    J(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J(2, 2) = l3 * c1 * s23;
  }

 private:
  Vec3<T> getHipLocation(int leg) const {
    return Vec3<T>((leg == 0 || leg == 1) ? _abadLocation(0) : -_abadLocation(0),
                   (leg == 1 || leg == 3) ? _abadLocation(1) : -_abadLocation(1),
                   _abadLocation(2));
  }

  T _bodyLength = T(0.3762);
  T _bodyWidth = T(0.0935);
  T _abadLinkLength = T(0.08);
  T _hipLinkLength = T(0.213);
  T _kneeLinkLength = T(0.223);
  T _kneeLinkY_offset = T(0.0);
  Vec3<T> _abadLocation = Vec3<T>::Zero();
};

}  // namespace rapid::robot::dynamics
