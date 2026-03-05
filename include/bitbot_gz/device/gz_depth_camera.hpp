#ifndef GZ_DEPTH_CAMERA_HPP
#define GZ_DEPTH_CAMERA_HPP

#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "bitbot_gz/device/gz_device.hpp"

namespace bitbot {

class GzDepthCamera final : public GzDevice {
 public:
  GzDepthCamera(const pugi::xml_node& device_node);
  ~GzDepthCamera();

 private:
  virtual void Input(const RosInterface::Ptr ros_interface) final;
  virtual void Output(const RosInterface::Ptr ros_interface) final;
  virtual void UpdateModel(const RosInterface::Ptr ros_interface) final;
  virtual void UpdateRuntimeData() final;

 private:
  cv::Mat depth_image_;
  bool debug_ = false;
  double frequency_ = 0.0;
  uint counter_ = 0;
  std::chrono::steady_clock::time_point last_update_time_;
};

}  // namespace bitbot

#endif  // !GZ_DEPTH_CAMERA_HPP
