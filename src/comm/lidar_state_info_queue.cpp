/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "lidar_state_info_queue.h"

namespace livox_ros {

void LidarStateInfoQueue::Push(StateInfoData* state_info_data) {
  StateInfoData data;
  data.lidar_type = state_info_data->lidar_type;
  data.device_type = state_info_data->device_type;
  data.handle = state_info_data->handle;
  data.time_stamp = state_info_data->time_stamp;

  // copy state info (rapidjson::Document type)
  data.info.CopyFrom(state_info_data->info, data.info.GetAllocator());

  std::lock_guard<std::mutex> lock(mutex_);
  state_info_queue_.push_back(std::move(data));
}

bool LidarStateInfoQueue::Pop(StateInfoData& state_info_data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_info_queue_.empty()) {
    return false;
  }
  state_info_data = std::move(state_info_queue_.front());
  state_info_queue_.pop_front();
  return true;
}

bool LidarStateInfoQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_info_queue_.empty();
}

void LidarStateInfoQueue::Clear() {
  std::list<StateInfoData> tmp_state_info_queue;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_info_queue_.swap(tmp_state_info_queue);
  }
}

} // namespace livox_ros
