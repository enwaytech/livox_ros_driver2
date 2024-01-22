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

#ifndef LIVOX_ROS_DRIVER_DIAGNOSTICS_CODES_TABLE_
#define LIVOX_ROS_DRIVER_DIAGNOSTICS_CODES_TABLE_

#include <unordered_map>

namespace livox_ros {

typedef struct ErrorDescription
{
  std::string description;
  std::string suggested_solution;

  ErrorDescription(std::string description, std::string suggested_solution) :
    description(description), suggested_solution(suggested_solution) {}
} ErrorDescription;

// Taken from https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/hms_code_mid360.html#diagnostic-codes-table
const std::unordered_map<int, ErrorDescription> error_code_to_description_mapping {
    // { name, timeout, topic }
    {0x0103, {"Environment temperature is relatively high", "Please check the environment temperature and losing heat measures"}},
    {0x0104, {"The window is dirty, which will influence the reliability of the point cloud", "Please clean the window"}},
    {0x0105, {"An error occurred during device upgrade process", "Please restart the upgrade process"}},
    {0x0111, {"Abnormal temperature of internal components of the device", "Please check the environment temperature and losing heat measures"}},
    {0x0112, {"Abnormal temperature of internal components of the device", "Please check the environment temperature and losing heat measures"}},
    {0x0113, {"IMU stopped working", "Please try to restart the device to restore"}},
    {0x0114, {"Environment temperature is high", "Please check the environment temperature and losing heat measures"}},
    {0x0115, {"Environment temperature beyond the limit, the device has stopped working", "Please check the environment temperature and losing heat measures"}},
    {0x0116, {"Abnormal external voltage", "Please check the external voltage"}},
    {0x0117, {"Abnormal lidar parameters", "Please try to restart the device to restore"}},
    {0x0201, {"Scan module is heating", "Please wait for the scan module heating"}},
    {0x0210, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0211, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0212, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0213, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0214, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0215, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0216, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0217, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0218, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    {0x0219, {"Scan module is abnormal, the system is trying to recover", "Please wait, if it lasts too long, please try restarting the device to restore"}},
    //{0x0210-{0x0219, {"Scan module is abnormal", "Please try to restart the device to restore"}},
    {0x0401, {"Communication link was linked down, now it is recovered", "Please check the communication link"}},
    {0x0402, {"PTP time synchronization stop or time gap is too big", "Please check the PTP time source"}},
    {0x0403, {"The version of PTP is 1588-v2.1, device don’t support this version", "Please replace 1588-v2.1 version with 1588.2.0 version"}},
    {0x0404, {"PPS time synchronization abnormal", "Please check the PPS and GPS signal"}},
    {0x0405, {"There was an exception in time synchronization", "Please check the exception reason"}},
    {0x0406, {"Time synchronization accuracy is low", "Please check the time source"}},
    {0x0407, {"PPS time synchronization fails because of loss of GPS signal", "Please check the GPS signal"}},
    {0x0408, {"PPS time synchronization fails because of loss of PPS signal", "Please check the PPS signal"}},
    {0x0409, {"GPS signal is abnormal", "Please check the GPS time source"}},
    {0x040A, {"The PTP and GPTP signals exist at the same time", "Please check the network topology, use PTP or GPTP alone to synchronize"}},
};

} // namespace

#endif // LIVOX_ROS_DRIVER_DIAGNOSTICS_CODES_TABLE_

