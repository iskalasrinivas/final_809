/**
 * @file      src/robot_controller.cpp
 * @brief     Source file for Robot Controller
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
#include <math.h>
#include <robot_controller.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id)
: async_spinner(0),
  interval(0.15),
  arm_id_(arm_id),
  robot_controller_nh_("/ariac/" + arm_id_),
  robot_controller_options("manipulator", "/ariac/" + arm_id_ + "/robot_description", robot_controller_nh_),
  robot_move_group_(robot_controller_options) {
	ROS_WARN_STREAM(">>>>> RobotController : " << arm_id_);
	async_spinner.start();

	// setting parameters of planner
	robot_move_group_.setPlanningTime(20);
	robot_move_group_.setNumPlanningAttempts(10);
	robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
	robot_move_group_.setMaxVelocityScalingFactor(0.9);
	robot_move_group_.setMaxAccelerationScalingFactor(0.9);

	robot_move_group_.allowReplanning(true);
	// collisionAvoidance();

	//--topic used to get the status of the gripper
	gripper_subscriber_ =
			gripper_nh_.subscribe("/ariac/" + arm_id_ + "/gripper/state", 10, &RobotController::GripperCallback, this);
	gripper_client_ =
			robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/" + arm_id_ + "/gripper/control");

	// home_joint_pose_ =  {1.0, 3.14,  -2.0,  2.14, 4.6, -1.51, 0.0};
	chooseArm();
	counter_ = 0;
	drop_flag_ = false;
}

RobotController::~RobotController(){}

void RobotController::initialSequence(){
	SendRobotHome();  // @TODO @Srinivas find the joint state near the home joint pose faced down
	lookupTransform();
	face_down_orientation_.x = robot_tf_transform_.getRotation().x();
	face_down_orientation_.y = robot_tf_transform_.getRotation().y();
	face_down_orientation_.z = robot_tf_transform_.getRotation().z();
	face_down_orientation_.w = robot_tf_transform_.getRotation().w();

	home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
	home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
	home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
	home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
	home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
	home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
	home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

    
	GoToJointState(home_joint_fl_arm); // @TODO @Srinivas find the joint state near the home joint pose faced left
	lookupTransform();
	face_left_orientation_.x = robot_tf_transform_.getRotation().x();
	face_left_orientation_.y = robot_tf_transform_.getRotation().y();
	face_left_orientation_.z = robot_tf_transform_.getRotation().z();
	face_left_orientation_.w = robot_tf_transform_.getRotation().w();

    
	GoToJointState(home_joint_fr_arm); // @TODO @Srinivas find the joint state near the home joint pose faced right
	lookupTransform();
	face_right_orientation_.x = robot_tf_transform_.getRotation().x();
	face_right_orientation_.y = robot_tf_transform_.getRotation().y();
	face_right_orientation_.z = robot_tf_transform_.getRotation().z();
	face_right_orientation_.w = robot_tf_transform_.getRotation().w();

	SendRobotHome();
	postInitialisation();
	ros::Duration(0.1).sleep();
	// GoToQualityCameraFromBin();
}

void RobotController::postInitialisation() {
	static_bin_pose.orientation = face_down_orientation_;
	if(arm_id_ == "arm1"){
       face_front_orientation_ = face_right_orientation_;
	} else if(arm_id_ == "arm1" ) {
		face_front_orientation_ = face_down_orientation_;
	}
	quality_static_pose.orientation = face_front_orientation_;

}

void RobotController::lookupTransform() {
	robot_tf_listener_.waitForTransform(arm_id_ + "_linear_arm_actuator", arm_id_ + "_ee_link", ros::Time(0),
			ros::Duration(10));
	robot_tf_listener_.lookupTransform("/" + arm_id_ + "_linear_arm_actuator", "/" + arm_id_ + "_ee_link", ros::Time(0),
			robot_tf_transform_);
}

bool RobotController::Planner() {
	ROS_INFO_STREAM("Planning started...");
	if (robot_move_group_.plan(robot_planner_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		plan_success_ = true;
		ROS_INFO_STREAM("Planner succeeded!");
	}
	else {
		plan_success_ = false;
		ROS_WARN_STREAM("Planner failed!");
	}

	return plan_success_;
}

void RobotController::chooseArm() {
	if (arm_id_ == "arm1") {
		home_joint_pose_ = {1.0, 3.14, -2.0, 2.14, -1.7, -1.59, 0.126}; 
		home_joint_fl_arm = {1.0, 3.14, -2.0, 2.14, -1.7, -3.14, 0.126};
		home_joint_fr_arm = {1.0, 3.14, -2.0, 2.14, -1.7, 0, 0.126};
		belt_joint_pose_ = { 0.1, 3.14, -2.7, -1.0, 2.1, -1.59, 0.126 };

		// home_joint_ff_arm = home_joint_fr_arm;
		static_bin_pose.position.x = -0.13;
		static_bin_pose.position.y = 0.75;
		static_bin_pose.position.z = 1.69;
		

		quality_static_pose.position.x = 0.19;
		quality_static_pose.position.y = 3.25;
		quality_static_pose.position.z = 1.15;
		

		agv_tf_listener_.waitForTransform("world", "kit_tray_1", ros::Time(0), ros::Duration(10));
		agv_tf_listener_.lookupTransform("/world", "/kit_tray_1", ros::Time(0), agv_tf_transform_);

		agv_position_.position.x = agv_tf_transform_.getOrigin().x();
		agv_position_.position.y = agv_tf_transform_.getOrigin().y();
		agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

		quality_cam_joint_position_ = { 1.18, 1.4, -0.38, 1.13, 2.26, -1.51, 0.0 };

		trash_bin_joint_position_ = { 1.18, 2.01, -1.38, 2.26, 3.27, -1.51, 0.0 };
	} else if (arm_id_ == "arm2") {
		home_joint_pose_ = {-0.9, 3.14, -2.0, 2.14, -1.7, -1.59, 0.126};
	    home_joint_fl_arm = {-0.9, 3.14, -2.0, 2.14, -1.7, -3.14, 0.126};
	    home_joint_fr_arm = {-0.9, 3.14, -2.0, 2.14, -1.7, 0, 0.126}; 
		// home_joint_ff_arm = home_joint_fl_arm;

		belt_joint_pose_ = { 0.1, 3.14, -2.7, -1.0, 2.1, -1.59, 0.126 };
		static_bin_pose.position.x = -0.04;
		static_bin_pose.position.y = -1.07;
		static_bin_pose.position.z = 1.41;

		quality_static_pose.position.x = 0.19;
		quality_static_pose.position.y = -3.25;
		quality_static_pose.position.z = 1.15;

		agv_tf_listener_.waitForTransform("world", "kit_tray_2", ros::Time(0), ros::Duration(10));
		agv_tf_listener_.lookupTransform("/world", "/kit_tray_2", ros::Time(0), agv_tf_transform_);
		agv_position_.position.x = agv_tf_transform_.getOrigin().x();
		agv_position_.position.y = agv_tf_transform_.getOrigin().y();
		agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

		quality_cam_joint_position_ = { -1.2, 4.50, -0.38, 1.01, 2.5, -1.51, 0.0 };

		trash_bin_joint_position_ = { -1.18, 3.52, -2.08, 2.71, 3.29, -1.51, 0.0 };
	}
}

void RobotController::Execute() {
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}
}

void RobotController::GoToBeltHome() {
	robot_move_group_.setJointValueTarget(belt_joint_pose_);
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}

	ros::Duration(interval).sleep();
}

void RobotController::GoToBinStaticPosition() {
	GoToTarget(static_bin_pose);
	ros::Duration(interval).sleep();
	ROS_INFO_STREAM("At Bin safe Home position");
}

geometry_msgs::Pose RobotController::getCurrentPose() {
	std::string armeelink_ = arm_id_ + "_ee_link";
	std::string larmeelink_ = "/" + arm_id_ + "_ee_link";
	robot_tf_listener_.waitForTransform("world", armeelink_, ros::Time(0), ros::Duration(10));
	robot_tf_listener_.lookupTransform("/world", larmeelink_, ros::Time(0), robot_tf_transform_);
	current_pose_.position.x = robot_tf_transform_.getOrigin().x();
	current_pose_.position.y = robot_tf_transform_.getOrigin().y();
	current_pose_.position.z = robot_tf_transform_.getOrigin().z();
	current_pose_.orientation.x = robot_tf_transform_.getRotation().x();
	current_pose_.orientation.y = robot_tf_transform_.getRotation().y();
	current_pose_.orientation.z = robot_tf_transform_.getRotation().z();
	current_pose_.orientation.w = robot_tf_transform_.getRotation().w();
	return current_pose_;
}

void RobotController::moveToTarget(geometry_msgs::Pose final_pose) {
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose dt_pose;

	dt_pose = getCurrentPose();

	dt_pose.position.x = (final_pose.position.x - current_pose_.position.x) / 10;
	dt_pose.position.y = (final_pose.position.y - current_pose_.position.y) / 10;
	dt_pose.position.z = (final_pose.position.z - current_pose_.position.z) / 10;
	dt_pose.orientation.x = 0;
	dt_pose.orientation.y = 0;
	dt_pose.orientation.z = 0;
	dt_pose.orientation.w = 0;

	geometry_msgs::Pose next_pose;
	for (int i = 1; i <= 10; i++)
	{
		next_pose.position.x = current_pose_.position.x + i * dt_pose.position.x;
		next_pose.position.y = current_pose_.position.y + i * dt_pose.position.y;
		next_pose.position.z = current_pose_.position.z + i * dt_pose.position.z;
		next_pose.orientation.x = current_pose_.orientation.x + i * dt_pose.orientation.x;
		next_pose.orientation.y = current_pose_.orientation.y + i * dt_pose.orientation.y;
		next_pose.orientation.z = current_pose_.orientation.z + i * dt_pose.orientation.z;
		next_pose.orientation.w = current_pose_.orientation.w + i * dt_pose.orientation.w;
		GoToTarget(next_pose);
		ros::Duration(interval).sleep();
	}
	GoToTarget(final_pose);
	ros::Duration(interval).sleep();
	// GoToTarget(waypoints);
}

void RobotController::moveToTargetinParabolicPath(geometry_msgs::Pose final_pose) {  // TODO @ Rachith Imporve this function
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose dt_pose;
	std::string armeelink_ = arm_id_ + "_ee_link";
	std::string larmeelink_ = "/" + arm_id_ + "_ee_link";
	robot_tf_listener_.waitForTransform("world", armeelink_, ros::Time(0), ros::Duration(10));
	robot_tf_listener_.lookupTransform("/world", larmeelink_, ros::Time(0), robot_tf_transform_);

	current_pose_.position.x = robot_tf_transform_.getOrigin().x();
	current_pose_.position.y = robot_tf_transform_.getOrigin().y();
	current_pose_.position.z = robot_tf_transform_.getOrigin().z();
	current_pose_.orientation.x = robot_tf_transform_.getRotation().x();
	current_pose_.orientation.y = robot_tf_transform_.getRotation().y();
	current_pose_.orientation.z = robot_tf_transform_.getRotation().z();
	current_pose_.orientation.w = robot_tf_transform_.getRotation().w();

	dt_pose.position.x = (final_pose.position.x - current_pose_.position.x) / 10;
	dt_pose.position.y = (final_pose.position.y - current_pose_.position.y) / 10;
	dt_pose.position.z = (final_pose.position.z - current_pose_.position.z) / 10;
	dt_pose.orientation.x = 0;
	dt_pose.orientation.y = 0;
	dt_pose.orientation.z = 0;
	dt_pose.orientation.w = 0;
	double dpara = -0.1;

	geometry_msgs::Pose next_pose;
	for (int i, j = 1; i <= 10; i++, j++) {
		if (i >= 6) {
			j = 10 - i;
		}
		//   dt_pose.position.x-=dpara;
		next_pose.position.x = current_pose_.position.x + i * dt_pose.position.x + j * dpara;
		next_pose.position.y = current_pose_.position.y + i * dt_pose.position.y;
		next_pose.position.z = current_pose_.position.z + i * dt_pose.position.z;
		//   next_pose.orientation.x = current_pose_.orientation.x +
		//     i * dt_pose.orientation.x;
		//   next_pose.orientation.y = current_pose_.orientation.y +
		//     i * dt_pose.orientation.y;
		//   next_pose.orientation.z = current_pose_.orientation.z +
		//     i * dt_pose.orientation.z;
		//   next_pose.orientation.w = current_pose_.orientation.w +
		//     i * dt_pose.orientation.w;
		//   waypoints.emplace_back(next_pose);
		GoToTarget(next_pose);
		ros::Duration(interval).sleep();
	}
	GoToTarget(final_pose);
	ros::Duration(interval).sleep();
}
void RobotController::GoToTarget(std::vector<geometry_msgs::Pose> waypoints) {
	ros::AsyncSpinner spinner(4);
	spinner.start();

	for (auto i : waypoints) {
		i.orientation.x = face_down_orientation_.x;
		i.orientation.y = face_down_orientation_.y;
		i.orientation.z = face_down_orientation_.z;
		i.orientation.w = face_down_orientation_.w;
	}

	moveit_msgs::RobotTrajectory traj;
	auto fraction = robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

	ROS_WARN_STREAM("Fraction: " << fraction * 100);
	ros::Duration(interval).sleep();

	robot_planner_.trajectory_ = traj;

	robot_move_group_.execute(robot_planner_);
	ros::Duration(interval).sleep();
}

void RobotController::GoToTarget(std::initializer_list<geometry_msgs::Pose> list) {
	ros::AsyncSpinner spinner(4);
	spinner.start();

	std::vector<geometry_msgs::Pose> waypoints;
	for (auto i : list) {
		i.orientation.x = face_down_orientation_.x;
		i.orientation.y = face_down_orientation_.y;
		i.orientation.z = face_down_orientation_.z;
		i.orientation.w = face_down_orientation_.w;
		waypoints.emplace_back(i);
	}

	moveit_msgs::RobotTrajectory traj;
	auto fraction = robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

	ROS_WARN_STREAM("Fraction: " << fraction * 100);
	ros::Duration(interval).sleep();

	robot_planner_.trajectory_ = traj;

	// if (fraction >= 0.3) {
	robot_move_group_.execute(robot_planner_);
	ros::Duration(interval).sleep();
	//    } else {
	//        ROS_ERROR_STREAM("Safe Trajectory not found!");
	//    }
}

void RobotController::GoToTarget(const geometry_msgs::Pose &pose) {
	target_pose_.orientation = face_down_orientation_;
	target_pose_.position = pose.position;
	ros::AsyncSpinner spinner(4);
	robot_move_group_.setPoseTarget(target_pose_);
	spinner.start();
	ros::Duration(interval).sleep();
	if (this->Planner()) {
		ros::Duration(interval).sleep();
		ROS_INFO_STREAM("Point success");
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}
	ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToAGV(const geometry_msgs::Pose &pose) {
	ROS_WARN_STREAM("placing in AGV HAHAHAHA ");
	ros::AsyncSpinner spinner(4);
	// pose.position.z += 0.1;
	robot_move_group_.setPoseTarget(pose);
	spinner.start();
	if (this->Planner()) {
		ROS_INFO_STREAM("Point success");
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}
	ROS_INFO_STREAM("Point reached...");
}

void RobotController::GripperToggle(const bool &state) {
	gripper_service_.request.enable = state;
	gripper_client_.call(gripper_service_);
	ros::Duration(interval).sleep();
	// if (gripper_client_.call(gripper_service_)) {
	if (gripper_service_.response.success) {
		ROS_INFO_STREAM("Gripper activated!");
	} else {
		ROS_WARN_STREAM("Gripper activation failed!");
	}
}

void RobotController::GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr &grip) {
	gripper_state_ = grip->attached;
}

bool RobotController::isPartAttached() {
	return gripper_state_;
}

bool RobotController::isAtQualitySensor() {
	return is_at_qualitySensor;
}

void RobotController::setAtQualitySensor() {
	is_at_qualitySensor = true;
}

void RobotController::GoToJointState(const std::vector<double> &pose) {
	robot_move_group_.setJointValueTarget(pose);
	// this->execute();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}
	ros::Duration(interval).sleep();
}

geometry_msgs::Pose RobotController::getHomeCartPose() {
	return home_cart_pose_;
}

void RobotController::SendRobotHome() {
	// ros::Duration(2.0).sleep();
	robot_move_group_.setJointValueTarget(home_joint_pose_);
	// this->execute();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}

	ros::Duration(interval).sleep();
}

void RobotController::dropInTrash() {
	// ros::Duration(interval).sleep();
	robot_move_group_.setJointValueTarget(trash_bin_joint_position_);
	// this->execute();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}
	GripperToggle(false);
	ros::Duration(interval).sleep();
}

// void RobotController::GoToQualityCamera() {
// 	// ros::Duration(interval).sleep();
// 	robot_move_group_.setJointValueTarget(quality_cam_joint_position_);
// 	// this->execute();
// 	//  ros::AsyncSpinner spinner(4);
// 	//  spinner.start();
// 	if (this->Planner()) {
// 		robot_move_group_.move();
// 		ros::Duration(interval).sleep();
// 		is_at_qualitySensor = true;
// 	}

// 	ros::Duration(interval).sleep();
// }

void RobotController::GoToQualityCamera() {
	// ros::Duration(interval).sleep();
	// is_at_qualitySensor = true;
	// this->execute();
	//  ros::AsyncSpinner spinner(4);
	//  spinner.start();
	ros::AsyncSpinner spinner(4);
	robot_move_group_.setPoseTarget(quality_static_pose);
	spinner.start();
	ROS_INFO_STREAM(" Going to Qual Camera");
	ros::Duration(interval).sleep();
	if (this->Planner()) {
		ros::Duration(interval).sleep();
		ROS_INFO_STREAM(" Reached Quality Cam Position");
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}
	ROS_INFO_STREAM("At quality camera check position");
}

// TODO @ Sanket Test Flip
void RobotController::pickFlipPart(const geometry_msgs::Pose &part_pose) {
	ROS_INFO_STREAM("Picking Flip Part");
	ros::Duration(interval).sleep();
	auto target_right_pose_1 = part_pose;
	// target_right_pose_1.position.z -= 0.02;
	target_right_pose_1.position.y -= 0.2;
	GoToTarget(target_right_pose_1);
	target_right_pose_1.position.z -= 0.2;
	GoToTarget(target_right_pose_1);
	ros::Duration(interval).sleep();
	auto target_pose = part_pose;
	target_pose.position.y -= 0.07;
	GoToTarget(target_pose);
	GripperToggle(true);
	if (!isPartAttached()) {
		while (!isPartAttached()){
			target_pose.position.y += 0.002;
			GoToTarget(target_pose);
			ros::Duration(interval).sleep();
		}
	}
	GoToTarget(target_right_pose_1);
	ros::Duration(interval).sleep();
	GoToBinStaticPosition();
	ros::Duration(interval).sleep();
}

// void RobotController::pickPart(const geometry_msgs::Pose &part_pose) {
// 	ROS_INFO_STREAM("Picking Part");
// 	ros::Duration(interval).sleep();

// 	GoToBinStaticPosition();
// 	ROS_INFO_STREAM("Going To BinStaticPosition");
	
// 	auto target_top_pose_1 = part_pose;
// 	target_top_pose_1.orientation = face_down_orientation_;
// 	target_top_pose_1.position.z += 0.2;
// 	GoToTarget(target_top_pose_1);
// 	ros::Duration(interval).sleep();
// 	auto target_pose = part_pose;
// 	target_pose.position.z += 0.07;
// 	GoToTarget(target_pose);
// 	GripperToggle(true);
// 	if (!isPartAttached()) {
// 		while (!isPartAttached()) {
// 			target_pose.position.z -= 0.002;
// 			GoToTarget(target_pose);
// 			ros::Duration(interval).sleep();
// 		}
// 	}
// 	GoToTarget(target_top_pose_1);
// 	ros::Duration(interval).sleep();
// 	GoToBinStaticPosition();
// 	ROS_INFO_STREAM("Coming back To BinStaticPosition");
// 	ros::Duration(interval).sleep();
// }

void RobotController::pickPart(const geometry_msgs::Pose &part_pose) {
	double bin_x_dis = 0.1; // Trial error
	double bin_height = 0.725;
	double object_thickness = std::fabs(part_pose.position.z - bin_height); //half odf the object thickness

	geometry_msgs::Pose inter_pose = part_pose;
	inter_pose.position.z = part_pose.position.z + 4 * object_thickness + 0.2;
	inter_pose.position.x += bin_x_dis;
	GoToTarget(inter_pose);
	inter_pose.position.x -= bin_x_dis;
	GoToTarget(inter_pose);
	geometry_msgs::Pose arrival_pose = part_pose;
	
	arrival_pose.position.z = part_pose.position.z + 1.1 * object_thickness + 0.15;
	
	GoToTarget(arrival_pose);
	GripperToggle(true);
	ROS_WARN_STREAM("Gripper toggled");
	while (!isPartAttached()) {
			ROS_WARN_STREAM("Part not attached");
			arrival_pose.position.z -= std::max(0.02*object_thickness, 0.02);
			GoToTarget(arrival_pose);
		}
		ROS_INFO_STREAM("Part attached");
		arrival_pose.position.z = part_pose.position.z + 1.1 * object_thickness + 0.05;
		GoToTarget(arrival_pose);
		ros::Duration(interval).sleep();
		GoToTarget(inter_pose);
		ros::Duration(interval).sleep();
		inter_pose.position.x += bin_x_dis;
		GoToTarget(inter_pose);
		ros::Duration(interval).sleep();
		// GoToBinStaticPosition();
		ROS_INFO_STREAM("Coming back To BinStaticPosition");		
}

void RobotController::deliverPart(const geometry_msgs::Pose &part_pose) {
	ROS_INFO_STREAM("Droping Part");
	ros::Duration(interval).sleep();

	auto target_top_pose_1 = part_pose;
	target_top_pose_1.position.z += 0.2;
	GoToTarget(target_top_pose_1);
	ros::Duration(interval).sleep();
	auto target_pose = part_pose;
	target_pose.position.z += 0.1;
	GoToTarget(target_pose);
	GripperToggle(false);

	GoToTarget(target_top_pose_1);
	ros::Duration(interval).sleep();
}
// sanket
void RobotController::flipPart(OrderPart *order_)
{
	if (order_->getFlipPart())
	{
		// logic to flip the part
		// define pose
		// move close to the bin surface?

		// roll the object
		tf2::Quaternion myQuaternion;
		auto flip_intermediate_pose_ = getCurrentPose();  // get the current pose of the robot ee_link
		myQuaternion.setRPY(1.558344, 0, 0);              // rotate about roll axis by 90deg
		flip_intermediate_pose_.orientation.x = myQuaternion.x();
		flip_intermediate_pose_.orientation.y = myQuaternion.y();
		flip_intermediate_pose_.orientation.z = myQuaternion.z();
		flip_intermediate_pose_.orientation.w = myQuaternion.w();

		GoToTarget(flip_intermediate_pose_);  // go to this pose
		GripperToggle(false);
		ros::Duration(0.2).sleep();

		flip_intermediate_pose_.position.z += 0.2;
		flip_intermediate_pose_.position.y -= 0.4;
		GoToTarget(flip_intermediate_pose_);

		flip_intermediate_pose_ = getCurrentPose();
		myQuaternion.setRPY(3.14, 0, 0);  // rotate about roll axis by 180deg
		flip_intermediate_pose_.orientation.x = myQuaternion.x();
		flip_intermediate_pose_.orientation.y = myQuaternion.y();
		flip_intermediate_pose_.orientation.z = myQuaternion.z();
		flip_intermediate_pose_.orientation.w = myQuaternion.w();
		GoToTarget(flip_intermediate_pose_);
		ros::Duration(0.2).sleep();

		// after flipping the part set flip part = true
		// order_->setFlipPart();
		// get pose from logical camera
		pickFlipPart(flip_intermediate_pose_);
	}
}

void RobotController::collisionAvoidance()
{
	// namespace rvt = rviz_visual_tools;
	// moveit_visual_tools::MoveItVisualTools visual_tools(arm_id_);
	// visual_tools.deleteAllMarkers();
	// visual_tools.loadRemoteControl();
	// //    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	// //    text_pose.translation().z() = 1.75;
	//
	// collision_object.header.frame_id = robot_move_group_.getPlanningFrame();
	//
	// // The id of the object is used to identify it.
	// collision_object.id = "box1";
	//
	// // Define a box to add to the world.
	// shape_msgs::SolidPrimitive primitive;
	// primitive.type = primitive.BOX;
	// primitive.dimensions.resize(3);
	// primitive.dimensions[0] = 0.1;
	// primitive.dimensions[1] = 0.1;
	// primitive.dimensions[2] = 0.1;
	//
	// // Define a pose for the box (specified relative to frame_id)
	// geometry_msgs::Pose box_pose;
	// box_pose.orientation.w = 1.0;
	// box_pose.position.x = -0.30;
	// box_pose.position.y = -0.38;
	// box_pose.position.z = 1.2;
	//
	// collision_object.primitives.push_back(primitive);
	// collision_object.primitive_poses.push_back(box_pose);
	// collision_object.operation = collision_object.ADD;
	//
	// //  std::vector<moveit_msgs::CollisionObject> collision_objects;
	// collision_objects.push_back(collision_object);
	//
	// // Now, let's add the collision object into the world
	// ros::Duration(5).sleep();
	// ROS_INFO_NAMED("tutorial", "Add an object into the world");
	// planning_scene_interface.addCollisionObjects(collision_objects);
	// ROS_INFO_STREAM("Added sensor");
	// //   visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
	// visual_tools.trigger();
}

void RobotController::pickPartFromBelt(geometry_msgs::Pose* part_pose)
{
	// TODO passing current pose as making a local copy of the part pose
	double belt_speed = 0.05; // Trial error
	double belt_height = 0.90;
	double object_thickness = std::fabs(part_pose->position.z - belt_height); //half odf the object thickness
	geometry_msgs::Pose arrival_pose = *part_pose;
	arrival_pose.position.z = part_pose->position.z + 1.1 * object_thickness + 0.05;
	arrival_pose.position.y -= belt_speed;
	GoToTarget(arrival_pose);
	GripperToggle(true);
	ROS_WARN_STREAM("Gripper toggled");
	geometry_msgs::Pose picking_pose = *part_pose;
	if (!isPartAttached())
	{
		int count = 0;
		picking_pose.position.z = part_pose->position.z + object_thickness + 0.02;
		picking_pose.position.y = part_pose->position.y - belt_speed;
		GoToTarget(picking_pose);
		picking_pose.position.z = part_pose->position.z + 1.1 * object_thickness + 0.05;
		picking_pose.position.y = part_pose->position.y-belt_speed;
		GoToTarget(picking_pose);
		++count;
		while (!isPartAttached() or count <= 3)
		{
			ROS_WARN_STREAM("Part not attached");
			picking_pose.position.z = part_pose->position.z + object_thickness + 0.004;
			picking_pose.position.y =  part_pose->position.y - belt_speed;
			GoToTarget(picking_pose);
			picking_pose.position.z = part_pose->position.z + 1.1 * object_thickness + 0.05;
			picking_pose.position.y =  part_pose->position.y - belt_speed;
			GoToTarget(picking_pose);
			++count;
		}
		ROS_INFO_STREAM("Part attached");
		picking_pose.position.z += 0.2;
		GoToTarget(picking_pose);
	}
}

bool RobotController::completeSinglePartOrder(OrderPart *order)
{
	bool flag = false;
	if(!isPartAttached()){
		auto curr_pose = order->getCurrentPose();
		pickPart(curr_pose);
	}

	GoToQualityCamera();
	if (is_faulty)
	{
		dropInTrash();
		flag = false;
	}
	else
	{
		auto end_pose = order->getEndPose();
		deliverPart(end_pose);
		flag = true;
	}
	// logic to return bool upon failure or success
	return flag;
}
