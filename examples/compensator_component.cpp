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


#include <memory>
#include "compensator_component.h"

namespace ydlidar {
namespace drivers {

CompensatorComponent::CompensatorComponent() {
  Init();
}

CompensatorComponent::~CompensatorComponent() {

}

bool CompensatorComponent::Init() {
  compensator_.reset(new Compensator());
  seq_ = 0;
  return true;
}

void CompensatorComponent::LaserScanMsgCallback(const LaserScan &scan) {
  LaserScan scan_compensated;

  if (compensator_->MotionCompensation(scan, scan_compensated)) {
    seq_++;
  }
}

void CompensatorComponent::OdometryMsgCallback(const odometry_t &odom) {
  compensator_->InsertOdomMsg(odom);
}
}  // namespace drivers
}  // namespace ydlidar
