/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2010  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * joint_movement_action_controller.cpp
 *
 *  Created on: 07.04.2011
 *      Author: Henning Deeken <hdeeken@uos.de>
 *
 */

#include "katana/joint_movement_action_controller.h"
#include <fstream>
#include <iostream>
#include <cstdio>

namespace katana
{

JointMovementActionController::JointMovementActionController(boost::shared_ptr<AbstractKatana> katana) :
  katana_(katana), action_server_(ros::NodeHandle(), "katana_arm_controller/joint_movement_action",
                                  boost::bind(&JointMovementActionController::executeCB, this, _1), false)
{
  joints_ = katana_->getJointNames();
  gripper_joints_ = katana_->getGripperJointNames();
  action_server_.start();
}

JointMovementActionController::~JointMovementActionController()
{
}

/**
 * Checks if all joints in the joint goal match a among joints of the katana
 */
bool JointMovementActionController::suitableJointGoal(const std::vector<std::string> &jointGoalNames)
{
  for (size_t i = 0; i < jointGoalNames.size(); i++)
  {
    bool exists = false;

    for (size_t j = 0; j < joints_.size(); j++)
    {
      if (jointGoalNames[i] == joints_[j])
        exists = true;
    }

    for (size_t k = 0; k < gripper_joints_.size(); k++)
    {
      if (jointGoalNames[i] == gripper_joints_[k])
        exists = true;
    }
    if (!exists)
    {
      ROS_ERROR("joint name %s is not one of our controlled joints", jointGoalNames[i].c_str());
      return false;
    }
  }

  return true;
}

void JointMovementActionController::adjustJointGoalPositionsToMotorLimits(const std::vector<std::string> &name, std::vector<double> &position, double &velocity)
{
  for (size_t i = 0; i < name.size(); i++)
  {
    ROS_DEBUG("%s - min: %f - max: %f - curr: % f - req: %f", name[i].c_str(), katana_->getMotorLimitMin(name[i]), katana_->getMotorLimitMax(name[i]), katana_->getMotorAngles()[katana_->getJointIndex(name[i])], position[i]);
  }

  for (size_t i = 0; i < name.size(); i++)
  {

    if (position[i] < katana_->getMotorLimitMin(name[i]))
    {
      position[i] = katana_->getMotorLimitMin(name[i]);

      ROS_INFO("%s - requested JointGoalPosition: %f exceeded MotorLimit: %f  - adjusted JointGoalPosition to MotorLimit", name[i].c_str(), position[i], katana_->getMotorLimitMin(name[i]));
    }

    if (position[i] > katana_->getMotorLimitMax(name[i]))
    {
      position[i] = katana_->getMotorLimitMax(name[i]);
      ROS_INFO("%s - requested JointGoalPosition: %f exceeded MotorLimit: %f  - adjusted JointGoalPosition to MotorLimit", name[i].c_str(), position[i], katana_->getMotorLimitMax(name[i]));
    }
  }

  // standing still is no movement
  if (velocity <= minVelocity){
    ROS_INFO("requested movement velocity %f too slow - adjusted to %f", velocity, minVelocity);
    velocity= minVelocity;
  }
  // fix some (reasonable) maximum speed
  else if (velocity > maxVelocity){
    ROS_INFO("requested movement velocity %f exceeds maximum %f - adjusted", velocity, maxVelocity);
    velocity= maxVelocity;
  }
}

void JointMovementActionController::executeCB(const JMAS::GoalConstPtr &goal)
{
  // note: the SimpleActionServer guarantees that we enter this function only when
  // there is no other active goal. in other words, only one instance of executeCB()
  // is ever running at the same time.

  if (!suitableJointGoal(goal->name))
  {
    ROS_ERROR("Joints on incoming goal don't match our joints/gripper_joints");

    for (size_t i = 0; i < goal->name.size(); i++)
    {
      ROS_INFO("  incoming joint %d: %s", (int)i, goal->name[i].c_str());
    }

    for (size_t i = 0; i < joints_.size(); i++)
    {
      ROS_INFO("  our joint      %d: %s", (int)i, joints_[i].c_str());
    }

    for (size_t i = 0; i < gripper_joints_.size(); i++)
    {
      ROS_INFO("  our gripper_joint      %d: %s", (int)i, gripper_joints_[i].c_str());
    }
    action_server_.setAborted();
    return;
  }

  if (action_server_.isPreemptRequested())
  {
    ROS_WARN("New action goal already seems to have been cancelled!");
    action_server_.setPreempted();
    return;
  }

  // adjust all goal positions to match the given motor limits
  std::vector<std::string> name(goal->name);
  std::vector<double> position(goal->position);
  double velocity(goal->speed);
  adjustJointGoalPositionsToMotorLimits(name, position, velocity);

  ROS_INFO("Sending %d joint movements to Katana arm...", name.size());

  // make movement linear in configuration space
  std::vector<double> angles= katana_->getMotorAngles();
  std::vector<double> angleDiffs;
  double maxDiff= 0;
  for (size_t i = 0; i < name.size(); ++i)
  {
    angleDiffs.push_back( std::abs( angles[katana_->getJointIndex(name[i])] - position[i] ) );
    maxDiff = std::max(maxDiff, angleDiffs.at(i));
  }

  for (size_t i = 0; i < name.size(); i++)
  {
    ROS_DEBUG("(motor %d) jid: %d / angle: %f->%f /  diff: %f(max %f) / requested velocity: %f->%f", i, katana_->getJointIndex(name[i]), angles[katana_->getJointIndex(name[i])], position[i], angleDiffs.at(i), maxDiff, velocity, (angleDiffs.at(i)/maxDiff)*velocity);
    if (!katana_->moveJoint(katana_->getJointIndex(name[i]), position[i], (angleDiffs.at(i)/maxDiff)*velocity))
    {
      ROS_ERROR("Problem while transferring movement to Katana arm. Aborting...");
      action_server_.setAborted();
      return;
    }
  }

  ros::Rate goalWait(10.0);

  while (ros::ok())
  {
    // always have to call this before calling someMotorCrashed() or allJointsReady()
    katana_->refreshMotorStatus();

    if (katana_->someMotorCrashed())
    {
      ROS_ERROR("Some motor has crashed! Aborting...");
      action_server_.setAborted();
      return;
    }

    if (katana_->allJointsReady())
    {
      ROS_INFO("...movement successfully executed.");
      action_server_.setSucceeded();
      return;
    }

    if (action_server_.isPreemptRequested())
    {
      ROS_WARN("Goal canceled by client while waiting for movement to finish, aborting!");
      action_server_.setPreempted();
      return;
    }

    goalWait.sleep();
  }
}

}

