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

#ifndef LIVOX_ROS_DRIVER_LIDAR_PACKET_DATA_QUEUE_H_
#define LIVOX_ROS_DRIVER_LIDAR_PACKET_DATA_QUEUE_H_

#include <list>
#include <mutex>
#include <cstdint>
#include <vector>

namespace livox_ros {

typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  bool extrinsic_enable;
  uint32_t point_num;
  uint8_t data_type;
  uint8_t line_num;
  uint64_t time_stamp;
  uint64_t point_interval;
  std::vector<uint8_t> raw_data;
} RawPacketData;

class LidarPacketDataQueue {
 public:
  void Push(RawPacketData* packet_data);
  bool Pop(RawPacketData& packet_data);
  bool Empty();
  void Clear();

 private:
  std::mutex mutex_;
  std::list<RawPacketData> packet_data_queue_;
};

} // namespace

#endif // LIVOX_ROS_DRIVER_LIDAR_PACKET_DATA_QUEUE_H_

