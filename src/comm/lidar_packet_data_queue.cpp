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

#include "lidar_packet_data_queue.h"

namespace livox_ros {

void LidarPacketDataQueue::Push(RawPacketData* packet_data) {
  RawPacketData data;
  data.lidar_type = packet_data->lidar_type;
  data.handle = packet_data->handle;
  data.extrinsic_enable = packet_data->extrinsic_enable;
  data.point_num = packet_data->point_num;
  data.data_type = packet_data->data_type;
  data.line_num = packet_data->line_num;
  data.time_stamp = packet_data->time_stamp;
  data.point_interval = packet_data->point_interval;
  data.raw_data = packet_data->raw_data;

  std::lock_guard<std::mutex> lock(mutex_);
  packet_data_queue_.push_back(std::move(data));
}

bool LidarPacketDataQueue::Pop(RawPacketData& packet_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (packet_data_queue_.empty()) {
    return false;
  }
  packet_data = packet_data_queue_.front();
  packet_data_queue_.pop_front();
  return true;
}

bool LidarPacketDataQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return packet_data_queue_.empty();
}

void LidarPacketDataQueue::Clear() {
  std::list<RawPacketData> tmp_packet_data_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    packet_data_queue_.swap(tmp_packet_data_queue);
  }
}

} // namespace livox_ros
