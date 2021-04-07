/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

#ifndef ASTRA_DRIVER_H
#define ASTRA_DRIVER_H

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/msg/image.hpp>

// #include <dynamic_reconfigure/server.h>
// #include <astra_camera/AstraConfig.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <string>
#include <vector>

#include "astra_camera/astra_device_manager.h"
#include "astra_camera/astra_device.h"
#include "astra_camera/astra_video_mode.h"
#include "astra_camera/astra_device_type.h"
#include "astra_camera/srv/get_serial.hpp"
#include "astra_camera/srv/get_device_type.hpp"
#include "astra_camera/srv/get_ir_gain.hpp"
#include "astra_camera/srv/set_ir_gain.hpp"
#include "astra_camera/srv/get_ir_exposure.hpp"
#include "astra_camera/srv/set_ir_exposure.hpp"
#include "astra_camera/srv/set_laser.hpp"
#include "astra_camera/srv/set_ldp.hpp"
#include "astra_camera/srv/reset_ir_gain.hpp"
#include "astra_camera/srv/reset_ir_exposure.hpp"
#include "astra_camera/srv/get_camera_info.hpp"
#include "astra_camera/srv/set_ir_flood.hpp"
#include "astra_camera/srv/switch_ir_camera.hpp"

#include <rclcpp/rclcpp.hpp>

namespace astra_wrapper
{

class AstraDriver
{
public:
  AstraDriver(rclcpp::Node::SharedPtr& n);
  ~AstraDriver();

private:
  // typedef astra_camera::AstraConfig Config;
  // typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  void newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
  void newColorFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
  void newDepthFrameCallback(sensor_msgs::msg::Image::SharedPtr image);

  // Methods to get calibration parameters for the various cameras
  sensor_msgs::msg::CameraInfo::SharedPtr getDefaultCameraInfo(int width, int height, double f) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getColorCameraInfo(int width, int height,  builtin_interfaces::msg::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getIRCameraInfo(int width, int height,  builtin_interfaces::msg::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getDepthCameraInfo(int width, int height,  builtin_interfaces::msg::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getProjectorCameraInfo(int width, int height,  builtin_interfaces::msg::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr convertAstraCameraInfo(OBCameraParams p,  builtin_interfaces::msg::Time time) const;

  void readConfigFromParameterServer();

  // resolves non-URI device IDs to URIs, e.g. '#1' is resolved to the URI of the first device
  std::string resolveDeviceURI(const std::string& device_id);
  void initDevice();

  void advertiseROSTopics();

  void imageConnectCb();
  void depthConnectCb();

  bool getSerialCb(astra_camera::srv::GetSerial::Request::SharedPtr req, astra_camera::srv::GetSerial::Response::SharedPtr res);
  bool getDeviceTypeCb(astra_camera::srv::GetDeviceType::Request::SharedPtr req, astra_camera::srv::GetDeviceType::Response::SharedPtr res);
  bool getIRGainCb(astra_camera::srv::GetIRGain::Request::SharedPtr req, astra_camera::srv::GetIRGain::Response::SharedPtr res);
  bool setIRGainCb(astra_camera::srv::SetIRGain::Request::SharedPtr req, astra_camera::srv::SetIRGain::Response::SharedPtr res);
  bool getIRExposureCb(astra_camera::srv::GetIRExposure::Request::SharedPtr req, astra_camera::srv::GetIRExposure::Response::SharedPtr res);
  bool setIRExposureCb(astra_camera::srv::SetIRExposure::Request::SharedPtr req, astra_camera::srv::SetIRExposure::Response::SharedPtr res);
  bool setLaserCb(astra_camera::srv::SetLaser::Request::SharedPtr req, astra_camera::srv::SetLaser::Response::SharedPtr res);
  bool resetIRGainCb(astra_camera::srv::ResetIRGain::Request::SharedPtr req, astra_camera::srv::ResetIRGain::Response::SharedPtr res);
  bool resetIRExposureCb(astra_camera::srv::ResetIRExposure::Request::SharedPtr req, astra_camera::srv::ResetIRExposure::Response::SharedPtr res);
  bool getCameraInfoCb(astra_camera::srv::GetCameraInfo::Request::SharedPtr req, astra_camera::srv::GetCameraInfo::Response::SharedPtr res);
  bool setIRFloodCb(astra_camera::srv::SetIRFlood::Request::SharedPtr req, astra_camera::srv::SetIRFlood::Response::SharedPtr res);
  bool switchIRCameraCb(astra_camera::srv::SwitchIRCamera::Request::SharedPtr req, astra_camera::srv::SwitchIRCamera::Response::SharedPtr res);
  bool setLDPCb(astra_camera::srv::SetLDP::Request::SharedPtr req, astra_camera::srv::SetLDP::Response::SharedPtr res);

  // void configCb(Config &config, uint32_t level);

  void applyConfigToOpenNIDevice();

  void genVideoModeTableMap();
  int lookupVideoModeFromDynConfig(int mode_nr, AstraVideoMode& video_mode);

  sensor_msgs::msg::Image::ConstSharedPtr rawToFloatingPointConversion(sensor_msgs::msg::Image::ConstSharedPtr raw_image);

  void setIRVideoMode(const AstraVideoMode& ir_video_mode);
  void setColorVideoMode(const AstraVideoMode& color_video_mode);
  void setDepthVideoMode(const AstraVideoMode& depth_video_mode);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr pnh_;

  boost::shared_ptr<AstraDeviceManager> device_manager_;
  boost::shared_ptr<AstraDevice> device_;

  std::string device_id_;

  /** \brief get_serial server*/
  rclcpp::Service<astra_camera::srv::GetCameraInfo>::SharedPtr get_camera_info;
  rclcpp::Service<astra_camera::srv::GetSerial>::SharedPtr get_serial_server;
  rclcpp::Service<astra_camera::srv::GetDeviceType>::SharedPtr get_device_type_server;
  rclcpp::Service<astra_camera::srv::GetIRGain>::SharedPtr get_ir_gain_server;
  rclcpp::Service<astra_camera::srv::SetIRGain>::SharedPtr set_ir_gain_server;
  rclcpp::Service<astra_camera::srv::GetIRExposure>::SharedPtr get_ir_exposure_server;
  rclcpp::Service<astra_camera::srv::SetIRExposure>::SharedPtr set_ir_exposure_server;
  rclcpp::Service<astra_camera::srv::SetIRFlood>::SharedPtr set_ir_flood_server;
  rclcpp::Service<astra_camera::srv::SetLaser>::SharedPtr set_laser_server;
  rclcpp::Service<astra_camera::srv::SetLDP>::SharedPtr set_ldp_server;
  rclcpp::Service<astra_camera::srv::ResetIRGain>::SharedPtr reset_ir_gain_server;
  rclcpp::Service<astra_camera::srv::ResetIRExposure>::SharedPtr reset_ir_exposure_server;
  rclcpp::Service<astra_camera::srv::SwitchIRCamera>::SharedPtr switch_ir_camera;

  /** \brief reconfigure server*/
  // boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  // bool config_init_;

  std::set<std::string>  alreadyOpen;
  boost::mutex connect_mutex_;
  // published topics
  image_transport::CameraPublisher pub_color_;
  image_transport::CameraPublisher pub_depth_;
  image_transport::CameraPublisher pub_depth_raw_;
  image_transport::CameraPublisher pub_ir_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_projector_info_;

  /** \brief Camera info manager objects. */
  boost::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_, ir_info_manager_;

  AstraVideoMode ir_video_mode_;
  AstraVideoMode color_video_mode_;
  AstraVideoMode depth_video_mode_;

  std::string ir_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_ ;

  std::string color_info_url_, ir_info_url_;

  bool color_depth_synchronization_;
  bool depth_registration_;

  std::map<int, AstraVideoMode> video_modes_lookup_;

  // dynamic reconfigure config
  double depth_ir_offset_x_;
  double depth_ir_offset_y_;
  int z_offset_mm_;
  double z_scaling_;

  rclcpp::Duration ir_time_offset_;
  rclcpp::Duration color_time_offset_;
  rclcpp::Duration depth_time_offset_;

  int data_skip_;

  int data_skip_ir_counter_;
  int data_skip_color_counter_;
  int data_skip_depth_counter_;

  bool rgb_preferred_;

  bool auto_exposure_;
  bool auto_white_balance_;

  bool ir_subscribers_;
  bool color_subscribers_;
  bool depth_subscribers_;
  bool depth_raw_subscribers_;
  bool projector_info_subscribers_;

  bool use_device_time_;

  // Config old_config_;
  int uvc_flip_;
};

}

#endif
