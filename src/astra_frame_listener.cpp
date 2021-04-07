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
#include "openni2/OpenNI.h"

#include "astra_camera/astra_frame_listener.h"
#include "astra_camera/astra_timer_filter.h"

#include <sensor_msgs/image_encodings.hpp>
#include <rcutils/logging_macros.h>
#include <rcutils/time.h>
#include <rcl/time.h>

#include <rclcpp/rclcpp.hpp>

#define TIME_FILTER_LENGTH 15

namespace astra_wrapper
{

AstraFrameListener::AstraFrameListener() :
    callback_(0),
    user_device_timer_(false),
    timer_filter_(new AstraTimerFilter(TIME_FILTER_LENGTH)),
    prev_time_stamp_(0.0)
{
}

void AstraFrameListener::setUseDeviceTimer(bool enable)
{
  user_device_timer_ = enable;

  if (user_device_timer_)
    timer_filter_->clear();
}

void AstraFrameListener::onNewFrame(openni::VideoStream& stream)
{
  stream.readFrame(&m_frame);

  if (m_frame.isValid() && callback_)
  {
    sensor_msgs::msg::Image::SharedPtr image(new sensor_msgs::msg::Image);

    rcutils_time_point_value_t rcl_sys_now;
    if (rcutils_system_time_now(&rcl_sys_now) != RCUTILS_RET_OK)
    {
      RCUTILS_LOG_ERROR("Failed to get current time; ignoring frame");
      return;
    }
    double current_time_stamp = rcl_sys_now / 1e9;

    if (!user_device_timer_)
    {
    // Time表示的总时间(纳秒)sec*1e9+nanosec
      image->header.stamp.sec = rcl_sys_now / 1000000000;
      image->header.stamp.nanosec = rcl_sys_now % 1000000000;

      RCUTILS_LOG_DEBUG("Time interval between frames: %.4f ms", (float)((current_time_stamp-prev_time_stamp_)*1e6));

      prev_time_stamp_ = current_time_stamp;
    } else
    {
      uint64_t device_time = m_frame.getTimestamp();

      double device_time_in_sec = static_cast<double>(device_time)/1e6;
      double ros_time_in_sec = current_time_stamp;

      double time_diff = ros_time_in_sec-device_time_in_sec;

      timer_filter_->addSample(time_diff);

      double filtered_time_diff = timer_filter_->getMedian();

      double corrected_timestamp = device_time_in_sec+filtered_time_diff;

      image->header.stamp.sec = corrected_timestamp;
      image->header.stamp.nanosec = (int)(corrected_timestamp * 1e9) % 1000000000;

      RCUTILS_LOG_DEBUG("Time interval between frames: %.4f ms", (float)((corrected_timestamp-prev_time_stamp_)*1e3));

      prev_time_stamp_ = corrected_timestamp;
    }

    image->width = m_frame.getWidth();
    image->height = m_frame.getHeight();

    std::size_t data_size = m_frame.getDataSize();

    image->data.resize(data_size);
    memcpy(&image->data[0], m_frame.getData(), data_size);

    image->is_bigendian = 0;

    const openni::VideoMode& video_mode = m_frame.getVideoMode();
    switch (video_mode.getPixelFormat())
    {
      case openni::PIXEL_FORMAT_DEPTH_1_MM:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case openni::PIXEL_FORMAT_DEPTH_100_UM:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case openni::PIXEL_FORMAT_SHIFT_9_2:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case openni::PIXEL_FORMAT_SHIFT_9_3:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;

      case openni::PIXEL_FORMAT_RGB888:
        image->encoding = sensor_msgs::image_encodings::RGB8;
        image->step = sizeof(unsigned char) * 3 * image->width;
        break;
      case openni::PIXEL_FORMAT_YUV422:
        image->encoding = sensor_msgs::image_encodings::YUV422;
        image->step = sizeof(unsigned char) * 4 * image->width;
        break;
      case openni::PIXEL_FORMAT_GRAY8:
        image->encoding = sensor_msgs::image_encodings::MONO8;
        image->step = sizeof(unsigned char) * 1 * image->width;
        break;
      case openni::PIXEL_FORMAT_GRAY16:
        image->encoding = sensor_msgs::image_encodings::MONO16;
        image->step = sizeof(unsigned char) * 2 * image->width;
        break;
      case openni::PIXEL_FORMAT_JPEG:
      default:
        RCUTILS_LOG_ERROR("Invalid image encoding");
        break;
    }

    callback_(image);
  }

}

}

