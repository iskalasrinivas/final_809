/**
 * @file      include/robot_controller.h
 * @brief     Header file Robot contoller
 * @author    Saurav Kumar
 * @author    Raja Srinivas
 * @author    Sanket Acharya
 * @author    Dinesh Kadirimangalam
 * @author    Preyash Parikh
 * @copyright 2020
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2020
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GROUP6_RWA4_ROBOT_CONTROLLER_H_
#define GROUP6_RWA4_ROBOT_CONTROLLER_H_

#include <iostream>
#include <string>
#include <vector>
#include <initializer_list>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stdarg.h>
#include <tf/transform_listener.h>



#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <order_part.h>


class RobotController {
private:
	std::string arm_id_;
	double interval;
	ros::NodeHandle robot_controller_nh_;
	ros::AsyncSpinner async_spinner;
	moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
	ros::ServiceClient gripper_client_;
	ros::NodeHandle gripper_nh_;
	ros::Subscriber gripper_subscriber_;

	tf::TransformListener robot_tf_listener_;
	tf::StampedTransform robot_tf_transform_;
	tf::TransformListener agv_tf_listener_;
	tf::StampedTransform agv_tf_transform_;


	geometry_msgs::Pose target_pose_;
	geometry_msgs::Pose flip_intermediate_pose_;
	geometry_msgs::Pose home_cart_pose_;
	geometry_msgs::Pose static_bin_pose;
	geometry_msgs::Pose quality_static_pose;
	geometry_msgs::Pose current_pose_;
	geometry_msgs::Pose thrash_pose;

	std::vector<double> belt_joint_pose_;
	std::vector<double> home_joint_pose_;
	std::vector<double> quality_cam_joint_position_;
	std::vector<double> trash_bin_joint_position_;
	std::vector<double> flip_test_joint_position;
	std::vector<double> other_side_joint_pose;


	std::vector<double> home_joint_fr_arm;
	std::vector<double> home_joint_fl_arm;
	std::vector<double> home_joint_ff_arm;

	// moveit_msgs::CollisionObject collision_object;
	// std::vector<moveit_msgs::CollisionObject> collision_objects;


	moveit::planning_interface::MoveGroupInterface robot_move_group_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

	osrf_gear::VacuumGripperControl gripper_service_;
	osrf_gear::VacuumGripperState gripper_status_;

	std::string object;
	bool plan_success_;


	geometry_msgs::Quaternion face_down_orientation_;
	geometry_msgs::Quaternion face_left_orientation_;
	geometry_msgs::Quaternion face_right_orientation_;
	geometry_msgs::Quaternion face_test_orientation_;
	geometry_msgs::Pose agv_position_;

	// geometry_msgs::Pose end_pose_;
	double offset_;
	double roll_def_, pitch_def_, yaw_def_;
	tf::Quaternion q;
	int counter_;
	bool gripper_state_, drop_flag_;
	bool is_at_qualitySensor;
	bool is_faulty;

public:
	explicit RobotController(std::string); // TODO - CHEKC THIS
	~RobotController();
	bool Planner();
	void chooseArm();
	void initialSequence();

	void postInitialisation();
	void lookupTransform();
	void Execute();
	void moveToTarget(geometry_msgs::Pose);
	void GoToTarget(std::vector<geometry_msgs::Pose>);
	void GoToTarget(std::initializer_list<geometry_msgs::Pose>);
	void GoToTarget(const geometry_msgs::Pose&);
	void GoToAGV(const geometry_msgs::Pose&);
	void flipPart(OrderPart *order_);
	void dropPartinAGV(const geometry_msgs::Pose &);
	void GripperToggle(const bool &);
	void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr&);
	void GripperStateCheck(geometry_msgs::Pose);
	bool isPartAttached();
	bool isAtQualitySensor();
	void setAtQualitySensor();

	std::string getArmName();
	void GoToJointState(const std::vector<double> &);
	geometry_msgs::Pose getHomeCartPose();
	geometry_msgs::Pose getTrashBinPose();
	void GotoTarget(const geometry_msgs::Pose&);

	void SendRobotHome();
	void dropInTrash();
	void GoToQualityCamera();
	void GoToBinStaticPosition();
	// void GoToQualityCameraFromBin();
	void moveToTargetinParabolicPath(geometry_msgs::Pose);
	void collisionAvoidance();
	geometry_msgs::Pose getCurrentPose();

	void pickPart(const geometry_msgs::Pose&, double, double);
	void pickPartFromAGV(const geometry_msgs::Pose&);
	void pickPartFromBin(const geometry_msgs::Pose&);
	bool pickPartFromBelt(geometry_msgs::Pose*);
	void GoToTargetForFlip(const geometry_msgs::Pose &pose);
	void dropPart(const geometry_msgs::Pose &);
	void dropInAGV();
	void GoToOtherSideliveryPose();
	void GoToBeltHome();

	// void deliverThePartinBin(OrderPart * oPart);

	// bool isPartfaulty();
};
#endif // GROUP6_RWA4_ROBOT_CONTROLLER_H_
