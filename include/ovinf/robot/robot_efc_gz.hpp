#ifndef ROBOT_EFC_GZ_HPP
#define ROBOT_EFC_GZ_HPP

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>
#include <vector>

#include "efc_gz_common.h"
#include "filter/filter_factory.hpp"
#include "robot/base/robot_base.hpp"
#include "utils/csv_logger.hpp"

namespace ovinf {

class RobotEfcGz : public RobotBase<float> {
  using VectorT = Eigen::Matrix<float, Eigen::Dynamic, 1>;
  friend class ObserverEfcGz;
  friend class ExecutorEfcGz;

 public:
  using Ptr = std::shared_ptr<RobotEfcGz>;

 private:
  class ObserverEfcGz : public ObserverBase {
   public:
    ObserverEfcGz() = delete;
    ObserverEfcGz(RobotBase<float>* robot, const YAML::Node& config)
        : ObserverBase(robot, config) {
      // Create Filter
      motor_pos_filter_ =
          FilterFactory::CreateFilter(config["motor_pos_filter"]);
      motor_vel_filter_ =
          FilterFactory::CreateFilter(config["motor_vel_filter"]);
      ang_vel_filter_ = FilterFactory::CreateFilter(config["ang_vel_filter"]);
      acc_filter_ = FilterFactory::CreateFilter(config["acc_filter"]);
      eluer_rpy_filter_ = FilterFactory::CreateFilter(config["euler_filter"]);

      // Create Logger
      log_flag_ = config["log_data"].as<bool>();
      if (log_flag_) {
        CreateLog(config);
      }
    }

    virtual VectorT const& Depth() const override {
      auto robot_gz = dynamic_cast<RobotEfcGz*>(robot_);
      return robot_gz->depth_camera_->GetDepthObs();
    }

    virtual bool Update() final {
      auto robot_gz = dynamic_cast<RobotEfcGz*>(robot_);

      for (size_t i = 0; i < motor_size_; ++i) {
        motor_actual_position_[i] = robot_gz->motors_[i]->GetActualPosition() *
                                    robot_->motor_direction_(i, 0);
        motor_actual_velocity_[i] = robot_gz->motors_[i]->GetActualVelocity() *
                                    robot_->motor_direction_(i, 0);
      }

      motor_actual_position_ =
          motor_pos_filter_->Filter(motor_actual_position_);
      motor_actual_velocity_ =
          motor_vel_filter_->Filter(motor_actual_velocity_);

      joint_actual_position_ = motor_actual_position_;
      joint_actual_velocity_ = motor_actual_velocity_;

      euler_rpy_ = eluer_rpy_filter_->Filter(
          (VectorT(3) << robot_gz->imu_->GetRoll(), robot_gz->imu_->GetPitch(),
           robot_gz->imu_->GetYaw())
              .finished());

      acceleration_ = acc_filter_->Filter(
          (VectorT(3) << robot_gz->imu_->GetAccX(), robot_gz->imu_->GetAccY(),
           robot_gz->imu_->GetAccZ())
              .finished());

      angular_velocity_ = ang_vel_filter_->Filter(
          (VectorT(3) << robot_gz->imu_->GetGyroX(), robot_gz->imu_->GetGyroY(),
           robot_gz->imu_->GetGyroZ())
              .finished());

      Eigen::Matrix3f Rwb(
          Eigen::AngleAxisf(euler_rpy_[2], Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(euler_rpy_[1], Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(euler_rpy_[0], Eigen::Vector3f::UnitX()));
      proj_gravity_ =
          VectorT(Rwb.transpose() * Eigen::Vector3f{0.0, 0.0, -1.0});

      if (log_flag_) {
        WriteLog();
      }
      return true;
    }

   private:
    inline void CreateLog(YAML::Node const& config);
    inline void WriteLog();

   private:
    FilterBase<VectorT>::Ptr motor_pos_filter_;
    FilterBase<VectorT>::Ptr motor_vel_filter_;
    FilterBase<VectorT>::Ptr ang_vel_filter_;
    FilterBase<VectorT>::Ptr acc_filter_;
    FilterBase<VectorT>::Ptr eluer_rpy_filter_;

    bool log_flag_ = false;
    CsvLogger::Ptr csv_logger_;
  };

  class ExecutorEfcGz : public ExecutorBase {
   public:
    ExecutorEfcGz() = delete;
    ExecutorEfcGz(RobotBase<float>* robot, const YAML::Node& config)
        : ExecutorBase(robot, config) {}

    virtual bool ExecuteJointTorque() final {
      motor_target_position_ = joint_target_position_;
      motor_target_torque_ = joint_target_torque_;

      ExecuteMotorTorque();
      return true;
    }

    virtual bool ExecuteMotorTorque() final {
      auto robot_gz = dynamic_cast<RobotEfcGz*>(robot_);
      for (size_t i = 0; i < motor_size_; ++i) {
        // Torque limit
        if (motor_target_torque_[i] > torque_limit_[i]) {
          motor_target_torque_[i] = torque_limit_[i];
        } else if (motor_target_torque_[i] < -torque_limit_[i]) {
          motor_target_torque_[i] = -torque_limit_[i];
        }

        // Position limit
        if (robot_->Observer()->MotorActualPosition()[i] >
            motor_upper_limit_[i]) {
          motor_target_torque_[i] = 0.0;
        } else if (robot_->Observer()->MotorActualPosition()[i] <
                   motor_lower_limit_[i]) {
          motor_target_torque_[i] = 0.0;
        }
      }

      // Set target
      for (size_t i = 0; i < motor_size_; ++i) {
        robot_gz->motors_[i]->SetTargetTorque(motor_target_torque_[i] *
                                              robot_->motor_direction_(i, 0));
        robot_gz->motors_[i]->SetTargetPosition(motor_target_position_[i] *
                                                robot_->motor_direction_(i, 0));
      }
      return true;
    }

    virtual bool ExecuteMotorCurrent() final {
      throw std::runtime_error(
          "ExecuteMotorCurrent is not supported in mujoco");
      return false;
    }

   private:
  };

 public:
  RobotEfcGz() = delete;
  RobotEfcGz(const YAML::Node& config) : RobotBase(config) {
    motors_.resize(motor_size_);
    this->observer_ = std::make_shared<ObserverEfcGz>((RobotBase<float>*)this,
                                                      config["observer"]);
    this->executor_ = std::make_shared<ExecutorEfcGz>((RobotBase<float>*)this,
                                                      config["executor"]);
  }

  inline void GetDevice(const KernelBus& bus);

  void SetExtraData(Kernel::ExtraData& extra_data) {
    extra_data_ = &extra_data;
  }

  virtual void PrintInfo() final {
    for (auto const& pair : motor_names_) {
      std::cout << "Motor id: " << pair.second << ", name: " << pair.first
                << std::endl;
      std::cout << "  - direction: " << motor_direction_(pair.second, 0)
                << std::endl;
      std::cout << "  - upper limit: "
                << executor_->MotorUpperLimit()(pair.second, 0) << std::endl;
      std::cout << "  - lower limit: "
                << executor_->MotorLowerLimit()(pair.second, 0) << std::endl;
      std::cout << "  - torque limit: "
                << executor_->TorqueLimit()(pair.second, 0) << std::endl;
    }
    for (auto const& pair : joint_names_) {
      std::cout << "Joint id: " << pair.second << " name: " << pair.first
                << std::endl;
    }
  }

 private:
  std::vector<MotorPtr> motors_ = {};
  ImuPtr imu_;
  DepthCameraPtr depth_camera_;

  Kernel::ExtraData* extra_data_;
  // std::vector<AnklePtr> ankles_;
};

void RobotEfcGz::GetDevice(const KernelBus& bus) {
  for (size_t i = 0; i < motor_size_; ++i) {
    motors_[i] = bus.GetDevice<MotorDevice>(i).value();
  }

  imu_ = bus.GetDevice<ImuDevice>(motor_size_).value();
  depth_camera_ = bus.GetDevice<DepthCameraDevice>(motor_size_ + 1).value();
}

void RobotEfcGz::ObserverEfcGz::CreateLog(YAML::Node const& config) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm* now_tm = std::localtime(&now_time);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
  std::string current_time = ss.str();

  std::string log_dir = config["log_dir"].as<std::string>();
  std::filesystem::path config_file_path(log_dir);
  if (config_file_path.is_relative()) {
    config_file_path = canonical(config_file_path);
  }

  std::string logger_file =
      config_file_path.string() + "/" + current_time + "_extra.csv";

  if (!exists(config_file_path)) {
    create_directories(config_file_path);
  }

  // Get headers
  std::vector<std::string> headers;

  // Motor actual pos
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_actual_pos_" + std::to_string(i));
  }

  // Motor actual vel
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_actual_vel_" + std::to_string(i));
  }

  // Joint actual pos
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_actual_pos_" + std::to_string(i));
  }

  // Joint actual vel
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_actual_vel_" + std::to_string(i));
  }

  // Motor target pos
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_target_pos_" + std::to_string(i));
  }

  // Motor target torque
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_target_torque_" + std::to_string(i));
  }

  // Joint target pos
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_target_pos_" + std::to_string(i));
  }

  // Joint target torque
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_target_torque_" + std::to_string(i));
  }

  // Acc
  headers.push_back("acc_x");
  headers.push_back("acc_y");
  headers.push_back("acc_z");

  // Ang vel
  headers.push_back("ang_vel_x");
  headers.push_back("ang_vel_y");
  headers.push_back("ang_vel_z");

  // Euler RPY
  headers.push_back("euler_roll");
  headers.push_back("euler_pitch");
  headers.push_back("euler_yaw");

  // Proj gravity
  headers.push_back("proj_gravity_x");
  headers.push_back("proj_gravity_y");
  headers.push_back("proj_gravity_z");

  csv_logger_ = std::make_shared<CsvLogger>(logger_file, headers);
}

void RobotEfcGz::ObserverEfcGz::WriteLog() {
  std::vector<CsvLogger::Number> datas;

  // Motor actual pos
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(motor_actual_position_[i]);
  }

  // Motor actual vel
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(motor_actual_velocity_[i]);
  }

  // Joint actual pos
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(joint_actual_position_[i]);
  }

  // Joint actual vel
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(joint_actual_velocity_[i]);
  }

  // Motor target pos
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(robot_->Executor()->MotorTargetPosition()[i]);
  }

  // Motor target torque
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(robot_->Executor()->MotorTargetTorque()[i]);
  }

  // Joint target pos
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(robot_->Executor()->JointTargetPosition()[i]);
  }

  // Joint target torque
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(robot_->Executor()->JointTargetTorque()[i]);
  }

  // Acc
  datas.push_back(acceleration_[0]);
  datas.push_back(acceleration_[1]);
  datas.push_back(acceleration_[2]);

  // Ang vel
  datas.push_back(angular_velocity_[0]);
  datas.push_back(angular_velocity_[1]);
  datas.push_back(angular_velocity_[2]);

  // Euler RPY
  datas.push_back(euler_rpy_[0]);
  datas.push_back(euler_rpy_[1]);
  datas.push_back(euler_rpy_[2]);

  // Proj gravity
  datas.push_back(proj_gravity_[0]);
  datas.push_back(proj_gravity_[1]);
  datas.push_back(proj_gravity_[2]);

  csv_logger_->Write(datas);
}

}  // namespace ovinf

#endif  // !ROBOT_EFC_GZ_HPP
