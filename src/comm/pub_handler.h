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

#ifndef LIVOX_DRIVER_PUB_HANDLER_H_
#define LIVOX_DRIVER_PUB_HANDLER_H_

#include <atomic>
#include <cstring>
#include <condition_variable> // std::condition_variable
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>              // std::mutex
#include <thread>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include "comm/comm.h"

namespace livox_ros {

class LidarPubHandler {
 public:
  LidarPubHandler();
  ~ LidarPubHandler() {}

  void PointCloudProcess(RawPacket& pkt);
  void SetLidarsExtParam(LidarExtParameter param);
  void SetLidarsFilterParam(LidarFilterParameter param);
  void SetLidarsFilterRaysParam(LidarFilterRaysParameter param);
  void GetLidarPointClouds(std::vector<PointXyzltrtp>& points_clouds);

  uint64_t GetRecentTimeStamp();
  uint32_t GetLidarPointCloudsSize();
  uint64_t GetLidarBaseTime();

 private:
  void LivoxLidarPointCloudProcess(RawPacket & pkt);
  void ProcessCartesianHighPoint(RawPacket & pkt);
  void ProcessCartesianLowPoint(RawPacket & pkt);
  void ProcessSphericalPoint(RawPacket & pkt);
  bool FilterYawPoint(const PointXyzltrtp& point);
  bool FilterRay(const PointXyzltrtp& point);

  std::vector<PointXyzltrtp> points_clouds_;
  ExtParameterDetailed extrinsic_ = {
    {0, 0, 0},
    {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1}
    }
  };
  RotationMatrix filter_rotation_ = {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1}
  };

  float filter_yaw_start_;
  float filter_yaw_end_;

  RotationMatrix filter_rays_rotation_ = {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1}
  };

  float filter_rays_yaw_start_;
  float filter_rays_yaw_end_;
  float filter_rays_pitch_start_;
  float filter_rays_pitch_end_;
  bool filter_rays_local_theta_;
  float filter_rays_local_theta_start_;
  float filter_rays_local_theta_end_;
  std::mutex mutex_;
  std::atomic_bool is_set_extrinsic_params_;
  std::atomic_bool is_set_filter_params_;
  std::atomic_bool is_set_filter_rays_params_;
};
  
class PubHandler {
 public:
  using PointCloudsCallback = std::function<void(PointFrame*, void *)>;
  using ImuDataCallback = std::function<void(ImuData*, void*)>;
  using StateInfoCallback = std::function<void(StateInfoData*, void*)>;
  using TimePoint = std::chrono::high_resolution_clock::time_point;

  PubHandler() {}

  ~ PubHandler() { Uninit(); }

  void Uninit();
  void RequestExit();
  void Init();
  void SetPointCloudConfig(const double publish_freq);
  void SetPointCloudsCallback(PointCloudsCallback cb, void* client_data);
  void SetLidarStateInfoCallback(StateInfoCallback cb, void* client_data);

  void AddLidarsExtParam(LidarExtParameter& extrinsic_params);
  void ClearAllLidarsExtrinsicParams();

  void AddLidarsFilterParam(LidarFilterParameter& filter_param);
  void ClearAllLidarsFilterParams();

  void AddLidarsFilterRaysParam(LidarFilterRaysParameter& filter_rays_param);
  void ClearAllLidarsFilterRaysParams();

  void SetImuDataCallback(ImuDataCallback cb, void* client_data);

 private:
  //thread to process raw data
  void RawDataProcess();
  std::atomic<bool> is_quit_{false};
  std::shared_ptr<std::thread> point_process_thread_;
  std::mutex packet_mutex_;
  std::condition_variable packet_condition_;

  //publish callback
  void CheckTimer(uint32_t id);
  void PublishPointCloud();
  static void OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                             LivoxLidarEthernetPacket *data, void *client_data);
  
  static void OnLivoxLidarStateInfoCallback(const uint32_t handle, const uint8_t dev_type,
                                            const char* info, void* client_data);

  static bool GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id);
  static uint64_t GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size);

  PointCloudsCallback points_callback_;
  void* pub_client_data_ = nullptr;

  ImuDataCallback imu_callback_;
  void* imu_client_data_ = nullptr;

  StateInfoCallback state_info_callback_;
  void* state_info_client_data_ = nullptr;

  PointFrame frame_;

  std::deque<RawPacket> raw_packet_queue_;

  //pub config
  uint64_t publish_interval_ = 100000000; //100 ms
  uint64_t publish_interval_tolerance_ = 100000000; //100 ms
  uint64_t publish_interval_ms_ = 100; //100 ms
  TimePoint last_pub_time_;

  std::map<uint32_t, std::unique_ptr<LidarPubHandler>> lidar_process_handlers_;
  std::map<uint32_t, std::vector<PointXyzltrtp>> points_;
  std::map<uint32_t, LidarExtParameter> lidar_extrinsics_;
  std::map<uint32_t, LidarFilterParameter> lidar_filters_;
  std::map<uint32_t, LidarFilterRaysParameter> lidar_rays_filters_;
  static std::atomic<bool> is_timestamp_sync_;
  uint16_t lidar_listen_id_ = 0;
};

PubHandler &pub_handler();

}  // namespace livox_ros

#endif  // LIVOX_DRIVER_PUB_HANDLER_H_
