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

#include "ApplyTwist.h"

#include <string>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <eigen_conversions/eigen_msg.h>

using namespace Eigen;

ApplyTwist::ApplyTwist(const std::string& name)
  : RTT::TaskContext(name, PreOperational), dt_(0.01) {
  this->ports()->addPort("PoseCmd", pose_command_port_);
  this->ports()->addPort("TwistCmd", twist_command_port_);
  this->ports()->addPort("CombinedPoseCmd", combined_pose_port_);
  this->addProperty("step_duration", dt_);

}

ApplyTwist::~ApplyTwist() {
}

bool ApplyTwist::configureHook() {
  return true;
}

bool ApplyTwist::startHook() {
  return true;
}

void ApplyTwist::stopHook() {

}

// if any of the inputs is new, the new output value is generated
void ApplyTwist::updateHook() {
  double sum = 0.0;
  bool new_data = false;

  if (pose_command_port_.read(combined_pose_) == RTT::NewData) {
    RTT::Logger::log(RTT::Logger::Debug) << "ApplyTwist: received pose"
                                         << RTT::endlog();
    if (twist_command_port_.read(cmd_twist_) == RTT::NewData) {
      RTT::Logger::log(RTT::Logger::Debug) << "ApplyTwist: received twist"
                                         << RTT::endlog();
      // add the linear part
      combined_pose_.position.x += cmd_twist_.linear.x * dt_;
      combined_pose_.position.y += cmd_twist_.linear.y * dt_;
      combined_pose_.position.z += cmd_twist_.linear.z * dt_;

      // create a quaternion for the angular part
      Quaterniond q_twist = Quaterniond(AngleAxisd(cmd_twist_.angular.z * dt_, Vector3d::UnitZ())
                                 * AngleAxisd(cmd_twist_.angular.y * dt_, Vector3d::UnitY())
                                 * AngleAxisd(cmd_twist_.angular.x * dt_, Vector3d::UnitX())); 

      // apply to the commanded orientation
      Quaterniond q_ori;
      tf::quaternionMsgToEigen(combined_pose_.orientation, q_ori);
      q_ori = q_ori * q_twist;  // combine with ori (local rotation, with right multiply)
      tf::quaternionEigenToMsg(q_ori, combined_pose_.orientation);
    }
    combined_pose_port_.write(combined_pose_);
  }
}

ORO_CREATE_COMPONENT(ApplyTwist)

