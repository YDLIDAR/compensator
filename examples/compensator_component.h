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
 *   @created Fri, June 19 14:33:54 CST 2020
 *
 ****************************************************************************/

#pragma once

#include <memory>
#include <vector>
#include "compensator.h"

namespace ydlidar {
namespace drivers {


class CompensatorComponent {
 public:
  CompensatorComponent();
  ~CompensatorComponent();
  /**
   * @brief  Odometer data message callback function
   * @param odom odometry data
   */
  void OdometryMsgCallback(const odometry_t &odom);
  /**
   * @brief Laser Scan data message callback function
   * @param scan Laser scan data
   */
  void LaserScanMsgCallback(const LaserScan &scan);

 private:
  /**
   * @brief Init
   * @return
   */
  bool Init();

  std::unique_ptr<Compensator> compensator_ = nullptr;
  int seq_;
};

}  // namespace drivers
}  // namespace ydlidar
