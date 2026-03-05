#include "bitbot_gz/device/gz_depth_camera.hpp"

namespace bitbot {

GzDepthCamera::GzDepthCamera(const pugi::xml_node& device_node)
    : GzDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::USER_DEFINE;
  type_ = (uint32_t)GzDeviceType::GZ_DEPTH_CAMERA;
  monitor_header_.headers = {"width", "height", "frequency"};
  monitor_data_.resize(monitor_header_.headers.size());
  debug_ = device_node.attribute("debug").as_bool(false);
  last_update_time_ = std::chrono::steady_clock::now();
}

GzDepthCamera::~GzDepthCamera() {}

void GzDepthCamera::Input(const RosInterface::Ptr ros_interface) {
  if (!ros_interface->IsDepthReady()) {
    // RCLCPP_WARN(rclcpp::get_logger("hs"),
    //             "Depth image not ready. Skipping this update.");
    return;
  }

  auto this_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     this_time - last_update_time_)
                     .count() /
                 1000.0;
  last_update_time_ = this_time;

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(ros_interface->GetDepthImage(),
                                 sensor_msgs::image_encodings::TYPE_32FC1);
    counter_++;
    frequency_ =
        frequency_ * (counter_ - 1) / counter_ + (1.0 / elapsed) / counter_;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("hs"), "cv_bridge exception: %s", e.what());

    return;
  }

  depth_image_ = cv_ptr->image.clone();
  if (debug_) {
    cv::Mat color_img;
    cv::pow(depth_image_, 0.6f, color_img);
    color_img.convertTo(color_img, CV_8UC1, 30);
    cv::imshow("Received Depth Image", color_img);
    cv::waitKey(1);
  }
}

void GzDepthCamera::Output(const RosInterface::Ptr ros_interface) {}

void GzDepthCamera::UpdateModel(const RosInterface::Ptr ros_interface) {}

void GzDepthCamera::UpdateRuntimeData() {
  monitor_data_[0] = depth_image_.cols;
  monitor_data_[1] = depth_image_.rows;
  monitor_data_[2] = frequency_;
}

}  // namespace bitbot
