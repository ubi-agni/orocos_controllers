/*
 * Copyright (c) 2010, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * JointTrajectoryAction.cpp
 *
 *  Created on: 23-09-2010
 *      Author: Konrad Banachowicz
 */

#include <ocl/Component.hpp>

#include "JointTrajectoryAction.h"

JointTrajectoryAction::JointTrajectoryAction(const std::string& name) :
    RTT::TaskContext(name, PreOperational), trajectoryPoint_port(
      "trajectory_point"), bufferReady_port("buffer_ready"),
    numberOfJoints_prop("number_of_joints", "", 0), as(this,
        "JointTrajectoryAction", boost::bind(
          &JointTrajectoryAction::goalCB, this, _1),
        boost::bind(&JointTrajectoryAction::cancelCB, this, _1),
        true)
{

  this->addPort(trajectoryPoint_port);
  this->addEventPort(bufferReady_port);

  this->addProperty(numberOfJoints_prop);

}

JointTrajectoryAction::~JointTrajectoryAction()
{

}

bool JointTrajectoryAction::configureHook()
{
  if ((numberOfJoints = numberOfJoints_prop.get()) == 0)
  {
    return false;
  }

  jointNames.resize(numberOfJoints);

  for (unsigned int i = 0; i < numberOfJoints; i++)
  {
    jointNames[i] = ((RTT::Property<std::string>*) this->getProperty(
                       std::string("joint") + (char) (i + 48) + "_name"))->get();
  }

  return true;
}

bool JointTrajectoryAction::startHook()
{
  goal_active = false;
  return true;
}

void JointTrajectoryAction::updateHook()
{
  bool tmp;
  as.spinOnce();

  if (bufferReady_port.read(tmp) == RTT::NewData)
  {
    if (tmp && goal_active)
    {
      if (currentPoint < endPoint)
      {
        trajectoryPoint_port.write(trajectory[currentPoint]);
        ++currentPoint;
      }
      else
      {
        RTT::Logger::log(RTT::Logger::Debug) << "Trajectory complete" << RTT::endlog();
        goal_active = false;
        activeGoal.setSucceeded();
      }
    }
  }
}

void JointTrajectoryAction::goalCB(GoalHandle gh)
{
  if (!goal_active)
  {
    std::vector<int> remapTable;
    remapTable.resize(numberOfJoints);

    Goal g = gh.getGoal();

    RTT::Logger::log(RTT::Logger::Debug) << "Received trajectory contain " << g->trajectory.points.size() << " points" << RTT::endlog();

    // fill remap table
    for (unsigned int i = 0; i < numberOfJoints; i++)
    {
      int jointId = -1;
      for (unsigned int j = 0; j < g->trajectory.joint_names.size(); j++)
      {
        if (g->trajectory.joint_names[j] == jointNames[i])
        {
          jointId = j;
          break;
        }
      }
      if (jointId < 0)
      {
        RTT::Logger::log(RTT::Logger::Error) << "Trajectory contains invalid joint" << RTT::endlog();
        gh.setRejected();
        return;
      }
      else
      {
        remapTable[i] = jointId;
      }

    }

    //remap joints

    trajectory.resize(g->trajectory.points.size());

    for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
    {
      trajectory[i].positions.resize(g->trajectory.points[i].positions.size());
      for (unsigned int j = 0; j < g->trajectory.points[i].positions.size(); j++)
      {
        trajectory[i].positions[j]
        = g->trajectory.points[i].positions[remapTable[j]];
      }

      trajectory[i].velocities.resize(g->trajectory.points[i].velocities.size());
      for (unsigned int j = 0; j < g->trajectory.points[i].velocities.size(); j++)
      {
        trajectory[i].velocities[j]
        = g->trajectory.points[i].velocities[remapTable[j]];
      }

      trajectory[i].accelerations.resize(g->trajectory.points[i].accelerations.size());
      for (unsigned int j = 0; j < g->trajectory.points[i].accelerations.size(); j++)
      {
        trajectory[i].accelerations[j]
        = g->trajectory.points[i].accelerations[remapTable[j]];
      }

      if(i == 0)
      {
        trajectory[i].time_from_start = g->trajectory.points[i].time_from_start;
      }
      else
      {
        trajectory[i].time_from_start = g->trajectory.points[i].time_from_start - g->trajectory.points[i-1].time_from_start;
      }
    }

    endPoint = g->trajectory.points.size();
    currentPoint = 0;

    activeGoal = gh;
    goal_active = true;
    gh.setAccepted();

  }
  else
  {
    gh.setRejected();
  }
}

void JointTrajectoryAction::cancelCB(GoalHandle gh)
{
  goal_active = false;
}

ORO_CREATE_COMPONENT( JointTrajectoryAction )
