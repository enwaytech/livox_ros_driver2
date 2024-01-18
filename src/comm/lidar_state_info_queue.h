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

#ifndef LIVOX_ROS_DRIVER_LIDAR_STATE_INFO_QUEUE_H_
#define LIVOX_ROS_DRIVER_LIDAR_STATE_INFO_QUEUE_H_

#include <list>
#include <mutex>
#include <cstdint>
#include <rapidjson/document.h>

namespace livox_ros {

// Based on the Info Data in Livox communication protocol
// https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html#x0102-lidar-information-push
// The information is kept as rapidjson object. It can be extracted by the name of the field depending on need
typedef struct {
  uint8_t lidar_type;   // corresponds to LidarProtoType defined in comm.h
  uint8_t device_type;  // corresponds to LivoxLidarDeviceType
  uint32_t handle;
  uint64_t time_stamp;

  rapidjson::Document info;
} StateInfoData;


class LidarStateInfoQueue {
 public:
  void Push(StateInfoData* state_info_data);
  bool Pop(StateInfoData& state_info_data);
  bool Empty();
  void Clear();

 private:
  std::mutex mutex_;
  std::list<StateInfoData> state_info_queue_;
};

} // namespace

#endif // LIVOX_ROS_DRIVER_LIDAR_STATE_INFO_QUEUE_H_

