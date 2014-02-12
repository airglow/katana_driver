/*
 * follow_joint_trajectory_client.h
 *
 *  Created on: 06.11.2011
 *      Author: martin
 */

#ifndef FOLLOW_JOINT_TRAJECTORY_CLIENT_H_
#define FOLLOW_JOINT_TRAJECTORY_CLIENT_H_

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>

#define GRASP 1
#define RELEASE 2

namespace katana_tutorials
{

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
typedef control_msgs::GripperCommandGoal GCG;

class FollowJointTrajectoryClient
{
public:
  FollowJointTrajectoryClient();
  virtual ~FollowJointTrajectoryClient();

  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
  control_msgs::FollowJointTrajectoryGoal makeArmUpTrajectory();
  actionlib::SimpleClientGoalState getState();
  bool send_gripper_action(int32_t goal_type);

private:
  ros::NodeHandle nh_;
  TrajClient traj_client_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
  ros::Subscriber joint_state_sub_;
  std::vector<std::string> joint_names_;
  bool got_joint_state_;
  std::vector<double> current_joint_state_;
  ros::AsyncSpinner spinner_;

  void jointStateCB(const sensor_msgs::JointState::ConstPtr &msg);
  trajectory_msgs::JointTrajectory filterJointTrajectory(const trajectory_msgs::JointTrajectory &input);
};

} /* namespace katana_tutorials */
#endif /* FOLLOW_JOINT_TRAJECTORY_CLIENT_H_ */
