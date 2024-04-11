//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32

#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#ifdef BUILDING_ROS2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/utils.h>

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq)
    : Lds(publish_freq, kSourceRawLidar),
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }


#ifdef BUILDING_ROS1
bool LdsLidar::InitLdsLidar(const std::string& path_name) {
#elif defined BUILDING_ROS2
bool LdsLidar::InitLdsLidar(const std::string& path_name, rclcpp::Clock::SharedPtr ros_clock) {
#endif
  if (is_initialized_) {
    printf("Lds is already inited!\n");
    return false;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }

  path_ = path_name;
#ifdef BUILDING_ROS1
  if (!InitLidars()) {
#elif defined BUILDING_ROS2
  if (!InitLidars(ros_clock)) {
#endif
    return false;
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

#ifdef BUILDING_ROS1
bool LdsLidar::InitLidars() {
#elif defined BUILDING_ROS2
bool LdsLidar::InitLidars(rclcpp::Clock::SharedPtr ros_clock) {
#endif
  if (!ParseSummaryConfig()) {
    return false;
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
#ifdef BUILDING_ROS1
    if (!InitLivoxLidar()) {
#elif defined BUILDING_ROS2
    if (!InitLivoxLidar(ros_clock)) {
#endif
      return false;
    }
  }
  return true;
}


bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

#ifdef BUILDING_ROS1
bool LdsLidar::InitLivoxLidar() {
#elif defined BUILDING_ROS2
bool LdsLidar::InitLivoxLidar(rclcpp::Clock::SharedPtr ros_clock) {
#endif
#ifdef BUILDING_ROS2
  DisableLivoxSdkConsoleLogger();
#endif

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
    return false;
  }

  // SDK initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // fill in lidar devices
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;

    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // temporary resolution
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler().AddLidarsExtParam(lidar_param);

    if (config.enable_yaw_filter) {
      LidarFilterParameter filter_param;
      filter_param.handle = config.handle;
      filter_param.lidar_type = kLivoxLidarType;
      filter_param.param = config.filter_param;
      filter_param.param.filter_frame_id = config.filter_param.filter_frame_id;
      filter_param.param.filter_yaw_start = config.filter_param.filter_yaw_start;
      filter_param.param.filter_yaw_end = config.filter_param.filter_yaw_end;

#ifdef BUILDING_ROS1
      auto rotation = GetTransformation(config.filter_param.filter_frame_id, config.frame_id);
#elif defined BUILDING_ROS2
      auto rotation = GetTransformation(config.filter_param.filter_frame_id, config.frame_id, ros_clock);
#endif

      if (rotation)
      {
        filter_param.transform.roll = std::get<0>(*rotation);
        filter_param.transform.pitch = std::get<1>(*rotation);
        filter_param.transform.yaw = std::get<2>(*rotation);
        pub_handler().AddLidarsFilterParam(filter_param);
      }
      else
      {
        throw std::runtime_error("Failed to lookup transformation between " + config.frame_id + " and " + config.filter_param.filter_frame_id);
      }
    }

    if (config.enable_rays_filter) {
      LidarFilterRaysParameter rays_param;
      rays_param.handle = config.handle;
      rays_param.lidar_type = kLivoxLidarType;
      rays_param.rays_param = config.filter_rays_param;
      rays_param.rays_param.filter_rays_yaw_start = config.filter_rays_param.filter_rays_yaw_start;
      rays_param.rays_param.filter_rays_yaw_end = config.filter_rays_param.filter_rays_yaw_end;
      rays_param.rays_param.filter_rays_pitch_start = config.filter_rays_param.filter_rays_pitch_start;
      rays_param.rays_param.filter_rays_pitch_end = config.filter_rays_param.filter_rays_pitch_end;

#ifdef BUILDING_ROS1
      auto rotation = GetTransformation(config.filter_rays_param.filter_rays_frame_id, config.frame_id);
#elif defined BUILDING_ROS2
      auto rotation = GetTransformation(config.filter_rays_param.filter_rays_frame_id, config.frame_id, ros_clock);
#endif

      if (rotation)
      {
        rays_param.transform.roll = std::get<0>(*rotation);
        rays_param.transform.pitch = std::get<1>(*rotation);
        rays_param.transform.yaw = std::get<2>(*rotation);
        pub_handler().AddLidarsFilterRaysParam(rays_param);
      }
      else
      {
        throw std::runtime_error("Failed to lookup transformation between " + config.frame_id + " and " + config.filter_rays_param.filter_rays_frame_id);
      }
    }

  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  return true;
}

void LdsLidar::SetLidarPubHandle() {
  pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);
  pub_handler().SetLidarStateInfoCallback(LidarCommonCallback::LidarStateInfoCallback, g_lds_ldiar);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler().SetPointCloudConfig(publish_freq);
}

bool LdsLidar::LivoxLidarStart() {
  return true;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

std::optional<std::tuple<float, float, float>> LdsLidar::GetTransformation(const std::string target_frame, const std::string source_frame, rclcpp::Clock::SharedPtr ros_clock)
{
#ifdef BUILDING_ROS1
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_{buffer_};
  constexpr double transform_timeout {1.0};
  if(!buffer_.canTransform(target_frame, source_frame, ros::Time(0), ros::Duration(transform_timeout)))
  {
    std::cout << "Timout wait for Transformation:" << target_frame << " " << source_frame << std::endl;
    return std::nullopt;
  }

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
  }
  catch (const tf2::TransformException& e)
  {
    std::cout << "TransformException: " << e.what() << std::endl;
    return std::nullopt;
  }
  double roll, pitch, yaw;
  tf2::getEulerYPR(transform.transform.rotation, yaw, pitch, roll);
  return std::tuple(static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw));

#elif defined BUILDING_ROS2
  tf2_ros::Buffer buffer_(ros_clock);
  tf2_ros::TransformListener listener_{buffer_};

  std::string suppressed_error_string;
  constexpr double transform_timeout {2.0};
  if(!buffer_.canTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(transform_timeout), &suppressed_error_string))
  {
    std::cout << "Timout wait for Transformation:" << target_frame << " " << source_frame << std::endl;
    return std::nullopt;
  }
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = buffer_.lookupTransform(target_frame, source_frame, rclcpp::Time(0));
  }
  catch (const tf2::TransformException& e)
  {
    std::cout << "TransformException: " << e.what() << std::endl;
    return std::nullopt;
  }
  double roll, pitch, yaw;
  tf2::getEulerYPR(transform.transform.rotation, yaw, pitch, roll);
  return std::tuple(static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw));
#endif
}
}  // namespace livox_ros
