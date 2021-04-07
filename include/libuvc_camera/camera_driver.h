#pragma once

#include <libuvc/libuvc.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
// #include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread/recursive_mutex.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <astra_camera/astra_device_type.h>
#include "astra_camera/srv/get_camera_info.hpp"
#include "astra_camera/srv/get_device_type.hpp"
#include <astra_camera/srv/get_uvc_exposure.hpp>
#include <astra_camera/srv/set_uvc_exposure.hpp>
#include <astra_camera/srv/get_uvc_gain.hpp>
#include <astra_camera/srv/set_uvc_gain.hpp>
#include <astra_camera/srv/get_uvc_white_balance.hpp>
#include <astra_camera/srv/set_uvc_white_balance.hpp>
// #include <astra_camera/uvc_camera_config.hpp>

#include <string>

namespace libuvc_camera {

class CameraDriver {
public:
  CameraDriver(rclcpp::Node::SharedPtr& nh);
  ~CameraDriver();

  bool Start();
  void Stop();

private:
  enum State {
    kInitial = 0,
    kStopped = 1,
    kRunning = 2,
  };

  // Flags controlling whether the sensor needs to be stopped (or reopened) when changing settings
  static const int kReconfigureClose = 3; // Need to close and reopen sensor to change this setting
  static const int kReconfigureStop = 1; // Need to stop the stream before changing this setting
  static const int kReconfigureRunning = 0; // We can change this setting without stopping the stream

  // void OpenCamera(UVCCameraConfig &new_config);
  void CloseCamera();

  // Accept a reconfigure request from a client
  // void ReconfigureCallback(UVCCameraConfig &config, uint32_t level);
  enum uvc_frame_format GetVideoMode(std::string vmode);
  // Accept changes in values of automatically updated controls
  void AutoControlsCallback(enum uvc_status_class status_class,
                            int event,
                            int selector,
                            enum uvc_status_attribute status_attribute,
                            void *data, size_t data_len);
  static void AutoControlsCallbackAdapter(enum uvc_status_class status_class,
                                          int event,
                                          int selector,
                                          enum uvc_status_attribute status_attribute,
                                          void *data, size_t data_len,
                                          void *ptr);
  // Accept a new image frame from the camera
  void ImageCallback(uvc_frame_t *frame);
  static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
  bool getUVCExposureCb(astra_camera::srv::GetUVCExposure::Request::SharedPtr req, astra_camera::srv::GetUVCExposure::Response::SharedPtr res);
  bool setUVCExposureCb(astra_camera::srv::SetUVCExposure::Request::SharedPtr req, astra_camera::srv::SetUVCExposure::Response::SharedPtr res);
  bool getUVCGainCb(astra_camera::srv::GetUVCGain::Request::SharedPtr req, astra_camera::srv::GetUVCGain::Response::SharedPtr res);
  bool setUVCGainCb(astra_camera::srv::SetUVCGain::Request::SharedPtr req, astra_camera::srv::SetUVCGain::Response::SharedPtr res);
  bool getUVCWhiteBalanceCb(astra_camera::srv::GetUVCWhiteBalance::Request::SharedPtr req, astra_camera::srv::GetUVCWhiteBalance::Response::SharedPtr res);
  bool setUVCWhiteBalanceCb(astra_camera::srv::SetUVCWhiteBalance::Request::SharedPtr req, astra_camera::srv::SetUVCWhiteBalance::Response::SharedPtr res);

  rclcpp::Node::SharedPtr nh_, priv_nh_;

  State state_;
  boost::recursive_mutex mutex_;

  uvc_context_t *ctx_;
  uvc_device_t *dev_;
  uvc_device_handle_t *devh_;
  uvc_frame_t *rgb_frame_;

  image_transport::ImageTransport it_;
  image_transport::CameraPublisher cam_pub_;

  // dynamic_reconfigure::Server<UVCCameraConfig> config_server_;
  // UVCCameraConfig config_;
  // bool config_changed_;

  camera_info_manager::CameraInfoManager cinfo_manager_;
  std::string ns;
  std::string ns_no_slash;

  rclcpp::Service<astra_camera::srv::GetUVCExposure>::SharedPtr get_uvc_exposure_server;
  rclcpp::Service<astra_camera::srv::SetUVCExposure>::SharedPtr set_uvc_exposure_server;
  rclcpp::Service<astra_camera::srv::GetUVCGain>::SharedPtr get_uvc_gain_server;
  rclcpp::Service<astra_camera::srv::SetUVCGain>::SharedPtr set_uvc_gain_server;
  rclcpp::Service<astra_camera::srv::GetUVCWhiteBalance>::SharedPtr get_uvc_white_balance_server;
  rclcpp::Service<astra_camera::srv::SetUVCWhiteBalance>::SharedPtr set_uvc_white_balance_server;

  rclcpp::Client<astra_camera::srv::GetDeviceType>::SharedPtr device_type_client;
  rclcpp::Client<astra_camera::srv::GetCameraInfo>::SharedPtr camera_info_client;

  bool device_type_init_;
  bool camera_info_init_;
  std::string device_type_;
  sensor_msgs::msg::CameraInfo camera_info_;
  int uvc_flip_;
  OB_DEVICE_NO device_type_no_;
  bool camera_info_valid_;
};

}
