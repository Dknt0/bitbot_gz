#ifndef DEX_GZ_COMMON_HPP
#define DEX_GZ_COMMON_HPP

#include "bitbot_gz/device/gz_depth_camera.hpp"
#include "bitbot_gz/device/gz_imu.h"
#include "bitbot_gz/device/gz_joint.h"
#include "bitbot_gz/kernel/gz_kernel.hpp"

namespace ovinf {

enum class MotorIdx {
  LHipPitchMotor = 0,
  LHipRollMotor = 1,
  LHipYawMotor = 2,
  LKneePitchMotor = 3,
  LAnklePitchMotor = 4,
  LAnkleRollMotor = 5,

  RHipPitchMotor = 6,
  RHipRollMotor = 7,
  RHipYawMotor = 8,
  RKneePitchMotor = 9,
  RAnklePitchMotor = 10,
  RAnkleRollMotor = 11,

  WaistYawMotor = 12,
  WaistRollMotor = 13,
  WaistPitchMotor = 14,

  LShoulderPitchMotor = 15,
  LShoulderRollMotor = 16,
  LShoulderYawMotor = 17,
  LElbowPitchMotor = 18,
  LElbowYawMotor = 19,
  LWristPitchMotor = 20,
  LWristRollMotor = 21,

  RShoulderPitchMotor = 22,
  RShoulderRollMotor = 23,
  RShoulderYawMotor = 24,
  RElbowPitchMotor = 25,
  RElbowYawMotor = 26,
  RWristPitchMotor = 27,
  RWristRollMotor = 28,
};

enum JointIdx {
  LHipPitchJoint = 0,
  LHipRollJoint = 1,
  LHipYawJoint = 2,
  LKneePitchJoint = 3,
  LAnklePitchJoint = 4,
  LAnkleRollJoint = 5,

  RHipPitchJoint = 6,
  RHipRollJoint = 7,
  RHipYawJoint = 8,
  RKneePitchJoint = 9,
  RAnklePitchJoint = 10,
  RAnkleRollJoint = 11,

  WaistYawJoint = 12,
  WaistRollJoint = 13,
  WaistPitchJoint = 14,

  LShoulderPitchJoint = 15,
  LShoulderRollJoint = 16,
  LShoulderYawJoint = 17,
  LElbowPitchJoint = 18,
  LElbowYawJoint = 19,
  LWristPitchJoint = 20,
  LWristRollJoint = 21,

  RShoulderPitchJoint = 22,
  RShoulderRollJoint = 23,
  RShoulderYawJoint = 24,
  RElbowPitchJoint = 25,
  RElbowYawJoint = 26,
  RWristPitchJoint = 27,
  RWristRollJoint = 28,
};

}  // namespace ovinf

using KernelBus = bitbot::GzBus;
using ImuDevice = bitbot::GzImu;
using ImuPtr = ImuDevice*;
using MotorDevice = bitbot::GzJoint;
using MotorPtr = MotorDevice*;
using DepthCameraDevice = bitbot::GzDepthCamera;
using DepthCameraPtr = DepthCameraDevice*;

struct UserData {};

using Kernel = bitbot::GzKernel<UserData>;

#endif  // !DEX_GZ_COMMON_HPP
