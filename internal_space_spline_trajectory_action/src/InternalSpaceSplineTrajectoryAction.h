/*
 * Copyright (c) 2010-2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

/*
 * InterrnalSpaceTrajectoryAction.h
 *
 * Action for both the motor and joint spline interpolation
 *
 *  Created on: 23-09-2010
 *      Author: Konrad Banachowicz
 */

#ifndef INTERNALSPACESPLINETRAJECTORYACTION_H_
#define INTERNALSPACESPLINETRAJECTORYACTION_H_

#include <string>
#include <vector>
#include <Eigen/Dense>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <rtt_actionlib/rtt_actionlib.h>
#include <boost/thread/reverse_lock.hpp>
#include <rtt_actionlib/rtt_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

// copied from joint_trajectory_controller/joint_trajectory_msg_utils.h
inline ros::Time startTime(const trajectory_msgs::JointTrajectory& msg,
                           const ros::Time&                        time)
{
  return msg.header.stamp.isZero() ? time : msg.header.stamp;
}

// copied from joint_trajectory_controller/joint_trajectory_msg_utils.h
class IsBeforePoint
{
public:
  IsBeforePoint(const ros::Time& msg_start_time) : msg_start_time_(msg_start_time) {}

  bool operator()(const ros::Time& time, const trajectory_msgs::JointTrajectoryPoint& point)
  {
    const ros::Time point_start_time = msg_start_time_ + point.time_from_start;
    return time < point_start_time;
  }

private:
  ros::Time msg_start_time_;
};



// copied from joint_trajectory_controller/joint_trajectory_msg_utils.h
inline std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator
findPoint(const trajectory_msgs::JointTrajectory& msg,
          const ros::Time&                        time)
{
  // Message trajectory start time
  // If message time is == 0.0, the trajectory should start at the current time
  const ros::Time msg_start_time = msg.header.stamp.isZero() ? time : msg.header.stamp;

  ros::Time point_start_time = msg_start_time_ + point.time_from_start;

  IsBeforePoint isBeforePoint(msg_start_time);

  typedef std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator ConstIterator;
  const ConstIterator first = msg.points.begin();
  const ConstIterator last  = msg.points.end();

  return (first == last || isBeforePoint(time, *first))
         ? last // Optimization when time preceeds all segments, or when an empty range is passed
         : --std::upper_bound(first, last, time, isBeforePoint); // Notice decrement operator
}


class InternalSpaceSplineTrajectoryAction : public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;
  typedef boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> Goal;

 public:
  explicit InternalSpaceSplineTrajectoryAction(const std::string& name);
  virtual ~InternalSpaceSplineTrajectoryAction();

  bool configureHook();
  bool startHook();
  void updateHook();

 protected:
  RTT::OutputPort<trajectory_msgs::JointTrajectoryConstPtr> trajectory_ptr_port_;
  RTT::OutputPort<control_msgs::JointTrajectoryControllerState> state_pub_port_;

  RTT::Property<int> numberOfJoints_prop_;

  RTT::InputPort<trajectory_msgs::JointTrajectory> command_port_;

  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_joint_position_command_;

 private:
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

  void commandCB();
  void compleatCB();
  void bufferReadyCB();

  std::vector<std::string> jointNames_;
  unsigned int numberOfJoints_;

  std::vector<double> lowerLimits_;
  std::vector<double> upperLimits_;

  std::vector<int> remapTable_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd desired_joint_position_;

  ros::Time trajectory_finish_time_;

  // RTT action server
  rtt_actionlib::RTTActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  bool goal_active_, command_active_;
  GoalHandle activeGoal_;
  bool enable_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::JointTrajectoryControllerState control_state_;
  
  trajectory_msgs::JointTrajectory command_msg_;
};

#endif  // INTERNALSPACESPLINETRAJECTORYACTION_H_
