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

/****************************************************************************
 *
 *   Copyright (c) 2019-2020 EAI Development Team. All rights reserved.
 *   @author Tony
 *   @created Thu, June 18 14:30:23 CST 2020
 *
 ****************************************************************************/


#include "compensator.h"
#include <limits>
#include <memory>
#include <string>
#include <core/base/timer.h>

namespace ydlidar {
namespace drivers {

Compensator::Compensator() : m_warn_throttle(0), first_indices(0) {
  current_sensor_vector.setOne();
  lidar_sensor_vector.setOne();
  timed_scan_queue_.clear();
  odom_queue.clear();
  odom_msgs_.clear();
}

Compensator::~Compensator() {

}

void Compensator::InsertOdomMsg(const odometry_t &odom) {
  core::base::ScopedLocker l(_odom_lock);
  odom_queue.push_back(odom);
  TrimOdometryData();
}

void Compensator::TrimOdometryData() {
  while (odom_queue.size() > 2 && !timed_scan_queue_.empty() &&
         odom_queue[1].stamp <= timed_scan_queue_.back()) {
    odom_queue.pop_front();
  }
}
void Compensator::TrimTimedData(const uint64_t time) {
  timed_scan_queue_.emplace_back(time);

  while (timed_scan_queue_.size() > 2 &&
         timed_scan_queue_[1] <= timed_scan_queue_.back() - scan_queue_duration_) {
    timed_scan_queue_.pop_front();
  }
}

odometry_t InterpolateOdom(
  const odometry_t &a,  ///< The first odometry.
  const odometry_t &b,  ///< The second odometry.
  const uint64_t time      ///< The interpolation target time.
) {
  odometry_t data;
  data.stamp = time;

  if (b.stamp == a.stamp) {
    data = b;
    return data;
  }

  float factor = static_cast<float>((time - a.stamp) / (b.stamp - a.stamp));

  if (factor < 0 || factor > 1) {
    data.clear();
    return data;
  }

  data.x = a.x + factor * (b.x - a.x);
  data.y = a.y + factor * (b.y - a.y);
  data.phi = a.phi + factor * (b.phi - a.phi);
  data.phi = ydlidar::core::math::normalize_angle(data.phi);
  return data;
}

template <typename MsgType>
std::pair<int, int> TimeStampBinarySearch(const std::deque<MsgType> &msgs,
    const uint64_t time, int start_index = 0) {
  int mid = 0;
  int start = start_index;
  int end = msgs.size() - 1;

  while (end - start > 1) {
    mid = start + (end - start) / 2;

    if (time < msgs.at(mid).stamp) {
      end = mid;
    } else {
      start = mid;
    }
  }

  return std::make_pair(start, end);
}

matrix::SquareMatrix<double, 3> convertOdomToMatrix(const odometry_t &data) {
  matrix::SquareMatrix<double, 3> data_matrix;
  data_matrix.setIdentity();
  //counterclockwisw data
  data_matrix(0, 0) = cos(data.phi);
  data_matrix(0, 1) = -sin(data.phi);
  data_matrix(0, 2) = data.x;
  data_matrix(1, 0) = sin(data.phi);
  data_matrix(1, 1) = cos(data.phi);
  data_matrix(1, 2) = data.y;
  data_matrix(2, 0) = 0;
  data_matrix(2, 1) = 0;
  data_matrix(2, 2) = 1;
  return data_matrix;
}

bool Compensator::waitForOdomAvailable(const uint64_t time,
                                       double threshold_in_sec) {
  uint32_t start_time = impl::getHDTimer();
  const uint32_t sleep_duration = threshold_in_sec * 1000;
  QueryOdomMsg();

  if (!QueryOdomCache()) {
    return false;
  }

  do {
    if (canAvailable(time)) {
      return true;
    }

    delay(sleep_duration);
  } while (impl::getHDTimer() < start_time + sleep_duration);

  QueryOdomMsg();
  return canAvailable(time);


}

bool Compensator::QueryOdomCache() {
  // size == 0
  if (odom_msgs_.empty()) {
    m_warn_throttle++;

    if (m_warn_throttle > 5) {
      fprintf(stderr, "No Odometry Data...\n");
      fflush(stderr);
      m_warn_throttle = 0;
    }

    return false;
  }

  return true;
}

bool Compensator::canAvailable(const uint16_t time) {
  if (odom_msgs_.size() < 2) {
    return false;
  }

  // size >= 2
  if (time < odom_msgs_.front().stamp) {
    fprintf(stderr, "time too old...\n");
    fflush(stderr);
    return false;
  }

  if (time > odom_msgs_.back().stamp) {
    fprintf(stderr, "time too new...\n");
    fflush(stderr);
    return false;
  }

  return true;
}

bool Compensator::GetOdomAtTime(const uint64_t time, odometry_t &odom,
                                double threshold_in_sec) {
  if (threshold_in_sec < 1.e-6) {
    threshold_in_sec = 1.e-6;
  }

  odometry_t former_data;
  former_data.clear();
  odometry_t latter_data;
  latter_data.clear();
  {
    if (!waitForOdomAvailable(time)) {
      return false;
    }

    // binary search for the time period for the target time
    auto indices = TimeStampBinarySearch(odom_msgs_, time, first_indices);
    former_data = odom_msgs_[indices.first];
    latter_data = odom_msgs_[indices.second];
    first_indices = indices.first;
  }

  if (time < former_data.stamp || time > latter_data.stamp) {
    return false;
  }

  // interpolate the data for more accurate odom data
  odom = InterpolateOdom(former_data, latter_data, time);
  return true;
}

void Compensator::QueryOdomMsg() {
  core::base::ScopedLocker l(_odom_lock);
  odom_msgs_ = odom_queue;
}

bool Compensator::MotionCompensation(const LaserScan &msg,
                                     LaserScan &msg_compensated) {
  if (msg.points.size() < 1) {
    fprintf(stderr, "Laser Scan size should not be 0\n");
    fflush(stderr);
    return false;
  }

  uint64_t start = impl::getCurrentTime();
  uint64_t timestamp_min = msg.stamp;
  uint64_t timestamp_max = msg.stamp + static_cast<uint64_t>
                           (msg.config.scan_time * 1e9);

  //Trim timed data
  TrimTimedData(msg.stamp);

  //min time pose
  odometry_t pose_min_time;
  pose_min_time.clear();

  //max time pose
  odometry_t pose_max_time;
  pose_max_time.clear();

  //copy raw msg
  msg_compensated = msg;

  //reset indices
  first_indices = 0;

  //Query first time pose
  if (!GetOdomAtTime(timestamp_min, pose_min_time)) {
    if (!odom_msgs_.empty()) {
      fprintf(stderr, "Failed to get odom at min timestamp[%llu]\n", timestamp_min);
      fflush(stderr);
    }

    return false;
  }

  //Query last time pose
  if (!GetOdomAtTime(timestamp_max, pose_max_time)) {
    if (!odom_msgs_.empty()) {
      fprintf(stderr, "Failed to get odom at max timestamp[%llu]\n", timestamp_max);
      fflush(stderr);
    }

    return false;
  }

  MotionCompensation(msg, msg_compensated, convertOdomToMatrix(pose_min_time));
  uint64_t end = impl::getCurrentTime();
  fprintf(stdout, "compenstator msg diff: %f ms\n", (end - start) / 1000000.0);
  fflush(stdout);
  return true;

}

void Compensator::MotionCompensation(const LaserScan &msg,
                                     LaserScan &msg_compensated,
                                     const matrix::SquareMatrix<double, 3> &pose_min_time) {
  int nRay = msg.points.size();
  //min time pose inverse matrix
  matrix::SquareMatrix<double, 3> pose_min_time_inv = matrix::inv(pose_min_time);

  for (int i = 0; i < nRay; i++) {
    LaserPoint point = msg.points[i];

    //invalid point
    if (point.range < msg.config.min_range) {
      continue;
    }

    //Current laser point time stamp
    uint64_t stamp = msg.stamp + static_cast<uint64_t>(msg.config.time_increment *
                     1e9);

    odometry_t pose_stamp_time;

    //Query odom pose at stamp time
    if (!GetOdomAtTime(stamp, pose_stamp_time)) {
      continue;
    }


    //polar to Cartesian
    current_sensor_vector(0) = point.range * cos(point.angle);
    current_sensor_vector(1) = point.range * sin(point.angle);
    current_sensor_vector(2) = 1;

    //Convert to the first time coordinate system
    lidar_sensor_vector = pose_min_time_inv * convertOdomToMatrix(
                            pose_stamp_time) * current_sensor_vector;

    double lx = lidar_sensor_vector(0);
    double ly = lidar_sensor_vector(1);

    //cartesian to polar
    double new_range = hypot(lx, ly);
    double new_angle = atan2(ly, lx);
    new_angle = ydlidar::core::math::normalize_angle(new_angle);
    point.angle = new_angle;
    point.range = new_range;
    msg_compensated.points[i] = point;
  }
}
}  // namespace drivers
}  // namespace ydlidar
