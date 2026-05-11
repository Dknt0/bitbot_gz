#include "bitbot_gz/device/gz_depth_camera.hpp"

namespace bitbot {

GzDepthCamera::GzDepthCamera(const pugi::xml_node& device_node)
    : GzDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::USER_DEFINE;
  type_ = (uint32_t)GzDeviceType::GZ_DEPTH_CAMERA;
  monitor_header_.headers = {"width", "height", "frequency"};
  monitor_data_.resize(monitor_header_.headers.size());
  debug_vis_ = device_node.attribute("debug_vis").as_bool(false);
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
    width_ = cv_ptr->image.cols;
    height_ = cv_ptr->image.rows;

    // TODO: Add method None image checking
    // TODO: Add the processing pipeline in config file.

    // Depth image processing
    // 1. Remap gazebo far-clipped (0) pixels to max depth=2.5 (for sim2sim)
    cv::Mat img = cv_ptr->image.clone();
    float min_depth = 0.0f;
    float max_depth = 2.5f;
    // Set all pixels with 0 (invalid/far clip) to max_depth
    for (int y = 0; y < img.rows; ++y) {
      float* row_ptr = img.ptr<float>(y);
      for (int x = 0; x < img.cols; ++x) {
        if (row_ptr[x] == 0.0f) row_ptr[x] = max_depth;
        // row_ptr[x] += 0.1;
        // row_ptr[x] += 0.05;
        // row_ptr[x] += 0.05;
      }
    }

    // 2. Crop image. Regions: top=18 px, bottom=-0 px, left=16 px, right=-16 px
    int crop_top = 18, crop_bottom = 0, crop_left = 16, crop_right = 16;
    // int crop_top = 0, crop_bottom = 0, crop_left = 0, crop_right = 0;
    cv::Rect roi(crop_left, crop_top, img.cols - crop_left - crop_right,
                 img.rows - crop_top - crop_bottom);
    cv::Mat cropped = img(roi);

    // 3. Gaussian blur (kernel=3, sigma=1, same as training config)
    cv::Mat blurred;
    cv::GaussianBlur(cropped, blurred, cv::Size(3, 3), 1.0, 1.0,
                     cv::BORDER_REFLECT);

    // 4. Normalize blurred depth to [0, 1]
    cv::Mat normalized;
    cv::Mat clipped;
    cv::min(blurred, max_depth, clipped);
    cv::max(clipped, min_depth, clipped);
    normalized = (clipped - min_depth) / (max_depth - min_depth);

    // Store processed result
    depth_obs_.resize(normalized.rows * normalized.cols);
    std::memcpy(depth_obs_.data(), normalized.data,
                depth_obs_.size() * sizeof(float));

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("hs"), "cv_bridge exception: %s", e.what());

    return;
  }

  if (debug_vis_) {
    // Visualization for arbitrary image size, scale by max (Python style)
    cv::Mat vis_image(18, 32, CV_32F, depth_obs_.data());

    double minVal = 0.0, maxVal = 1.0;
    // cv::minMaxLoc(vis_image, &minVal, &maxVal);
    cv::Mat img8u;
    if (maxVal < 1e-6) {
      // Avoid divide by zero if all zero
      img8u = cv::Mat::zeros(vis_image.size(), CV_8U);
    } else {
      vis_image.convertTo(
          img8u, CV_8U,
          255.0 / maxVal);  // Only scale by max, do not subtract min
    }
    cv::Mat img_big;
    int scale = 5;
    cv::resize(img8u, img_big, cv::Size(img8u.cols * scale, img8u.rows * scale),
               0, 0, cv::INTER_AREA);
    cv::namedWindow("depth_obs", cv::WINDOW_NORMAL);
    cv::imshow("depth_obs", img_big);
    cv::waitKey(1);
  }
}

void GzDepthCamera::Output(const RosInterface::Ptr ros_interface) {}

void GzDepthCamera::UpdateModel(const RosInterface::Ptr ros_interface) {}

void GzDepthCamera::UpdateRuntimeData() {
  monitor_data_[0] = width_;
  monitor_data_[1] = height_;
  monitor_data_[2] = frequency_;
}

}  // namespace bitbot
