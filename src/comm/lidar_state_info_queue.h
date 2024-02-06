/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

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

