/*
 * Derived from PortDoubleSum.cpp
 * 
 * Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "PortTwistSum.h"

#include <string>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <eigen_conversions/eigen_msg.h>

using namespace Eigen;

PortTwistSum::PortTwistSum(const std::string& name)
  : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("TwistA", twist_a_port_);
  this->ports()->addPort("TwistB", twist_b_port_);
  this->ports()->addPort("TwistSum", twist_sum_port_);
}

PortTwistSum::~PortTwistSum() {
}

bool PortTwistSum::configureHook() {

  return true;
}

// if any of the inputs is new the new output value is generated
void PortTwistSum::updateHook() {
  if (twist_a_port_.read(twist_sum_) == RTT::NewData) {
    if (twist_b_port_.read(twist_tmp_) == RTT::NewData) {
      // add the linear part
      twist_sum_.linear.x += twist_tmp_.linear.x;
      twist_sum_.linear.y += twist_tmp_.linear.y;
      twist_sum_.linear.z += twist_tmp_.linear.z;
      twist_sum_.angular.x += twist_tmp_.angular.x;
      twist_sum_.angular.y += twist_tmp_.angular.y;
      twist_sum_.angular.z += twist_tmp_.angular.z;
    }
    twist_sum_port_.write(twist_sum_);
  }
  else
  {
    if (twist_b_port_.read(twist_sum_) == RTT::NewData)
      twist_sum_port_.write(twist_sum_);
  }
}

ORO_CREATE_COMPONENT(PortTwistSum)

