//
// The MIT License (MIT)
//
// Copyright (c) 2019-2020 EAIBOT. All rights reserved.
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

/** @mainpage Compensator
 * Compensator API
    <table>
        <tr><th>Library     <td>Compensator
        <tr><th>File        <td>compensator.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/Compensator
        <tr><th>Version     <td>1.0.0
    </table>

* @copyright    Copyright (c) 2018-2020  EAIBOT
     Jump to the @link ::ydlidar::drivers::Compensator @endlink interface documentation.
*/

#pragma once
#include <memory>
#include <string>
#include <deque>
#include <src/CYdLidar.h>
#include <core/math/angles.h>
#include "matrix/math.hpp"

namespace ydlidar {
namespace drivers {

struct odometry_t {
  uint64_t  stamp; ///< timestamp
  double    x;	   ///< x position
  double    y;	   ///< y position
  double    phi;   ///< yaw angle
  odometry_t &operator=(const odometry_t &o) { //operator=
    this->stamp = o.stamp;
    this->x = o.x;
    this->y = o.y;
    this->phi = o.phi;
    return *this;
  }
  void clear() {
    this->stamp = 0;
    this->x = 0;
    this->y = 0;
    this->phi = 0;
  }
};

class Compensator {
 public:
  Compensator();
  virtual ~Compensator();

  /**
   * @brief insert odometry
   *
   */
  void InsertOdomMsg(const odometry_t &odom);

  /**
   * @brief motion compensation for LaserScan
   */
  bool MotionCompensation(const LaserScan &msg, LaserScan &msg_compensated);

 private:

  /**
   * @brief motion compensation for LaserScan
   */
  void MotionCompensation(const LaserScan &msg, LaserScan &msg_compensated,
                          const matrix::SquareMatrix<double, 3> &pose_min_time);

  /**
   * @brief Query last odometry msg
   */
  void QueryOdomMsg();

  /**
   * @brief waitForOdomAvailable
   * @param time
   * @param threshold_in_sec
   * @return
   */
  bool waitForOdomAvailable(const uint64_t time,
                            double threshold_in_sec = 10.e-3);

  /**
   * @brief QueryOdomCache
   * @return
   */
  bool QueryOdomCache();

  /**
   * @brief canAvailable
   * @param time
   * @return
   */
  bool canAvailable(const uint16_t time);

  /**
   * @brief get odometry pose by time
   * @param time
   * @param odom
   * @param threshold_in_sec
   * @return
   */
  bool GetOdomAtTime(const uint64_t time, odometry_t &odom,
                     double threshold_in_sec = 8.e-3);//8ms

  /**
   * @brief TrimOdometryData
   */
  void TrimOdometryData();

  /**
   * @brief TrimTimedData
   */
  void TrimTimedData(const uint64_t time);

 private:
  //time odom queue
  std::deque< odometry_t > odom_queue;
  //query odom msg
  std::deque< odometry_t > odom_msgs_;
  core::base::Locker       _odom_lock;

  //scan time queue
  std::deque<uint64_t> timed_scan_queue_;
  const uint32_t scan_queue_duration_ = 5 * 1e8;//0.5s
  int m_warn_throttle;
  int first_indices;
  //lidar
  matrix::Vector<double, 3> lidar_sensor_vector;
  matrix::Vector<double, 3> current_sensor_vector;
};

}  // namespace drivers
}  // namespace ydlidar
