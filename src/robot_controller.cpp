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
  interval(0.12),
  arm_id_(arm_id),
  robot_controller_nh_("/ariac/" + arm_id_),
  robot_controller_options("manipulator", "/ariac/" + arm_id_ + "/robot_description", robot_controller_nh_),
  robot_move_group_(robot_controller_options) {
	ROS_WARN_STREAM(">>>>> RobotController : " << arm_id_);
	async_spinner.start();
	robot_move_group_.setPlanningTime(20);
	robot_move_group_.setNumPlanningAttempts(10);
	robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
	robot_move_group_.setMaxVelocityScalingFactor(0.9);
	robot_move_group_.setMaxAccelerationScalingFactor(0.9);

	robot_move_group_.allowReplanning(true);
	// collisionAvoidance();

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

std::string RobotController::getArmName() {
	return arm_id_;
}
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

	GoToJointState(flip_test_joint_position);
	lookupTransform();
	face_test_orientation_.x = robot_tf_transform_.getRotation().x();
	face_test_orientation_.y = robot_tf_transform_.getRotation().y();
	face_test_orientation_.z = robot_tf_transform_.getRotation().z();
	face_test_orientation_.w = robot_tf_transform_.getRotation().w();

	SendRobotHome();
	postInitialisation();
	ros::Duration(0.1).sleep();
	// GoToQualityCameraFromBin();
}

geometry_msgs::Pose RobotController::getTrashBinPose() {

	return thrash_pose;
}

void RobotController::postInitialisation() {
	thrash_pose.orientation = face_down_orientation_;
	static_bin_pose.orientation = face_down_orientation_;
	quality_static_pose.orientation = face_down_orientation_;

}

void RobotController::lookupTransform() {
	robot_tf_listener_.waitForTransform(arm_id_ + "_linear_arm_actuator", arm_id_ + "_ee_link", ros::Time(0),
			ros::Duration(10));
	robot_tf_listener_.lookupTransform("/" + arm_id_ + "_linear_arm_actuator", "/" + arm_id_ + "_ee_link", ros::Time(0),
			robot_tf_transform_);
}

bool RobotController::Planner() {
	//	ROS_INFO_STREAM("Planning started...");
	if (robot_move_group_.plan(robot_planner_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		plan_success_ = true;
		//		ROS_INFO_STREAM("Planner succeeded!");
	}
	else {
		plan_success_ = false;
		ROS_WARN_STREAM("Planner failed!");
	}

	return plan_success_;
}

void RobotController::chooseArm() {
	if (arm_id_ == "arm1") {
		home_joint_pose_ = {1.0, 3.14, -2.0, 2.14, -1.7, -1.59, 0.0};
		home_joint_fl_arm = {1.0, 3.14, -2.0, 2.14, -1.7, -3.14, 0.0};
		home_joint_fr_arm = {1.0, 3.14, -2.0, 2.14, -1.7, 0, 0.0};
		//		belt_joint_pose_ = { -0.3, 0, -0.7, 1.65, -2.5, -1.59, 0.0};
		belt_joint_pose_ = { 0.7,-0.9, -0.05,0.2 , -1.65, -1.59, 0.0};
		quality_cam_joint_position_ = { 0.8, 1.4, -0.7, 1.2, -2.2, -1.51, 0.0  };
		flip_test_joint_position = {0.5, 0, -2.3, -2.1, -2.0, 0, 0.0};
		other_side_joint_pose = {0.8, 3.98,-0.7, 0.5, -1.7, -1.59, 0.0};

		trash_bin_joint_position_ = {1.18, 2.01, -1.38, 2.26, -2.3, -1.59, 0.0};
		// home_joint_ff_arm = home_joint_fr_arm;
		static_bin_pose.position.x = -0.13;
		static_bin_pose.position.y = 0.75;
		static_bin_pose.position.z = 1.69;


		quality_static_pose.position.x = 0.19;
		quality_static_pose.position.y = 3.25;
		quality_static_pose.position.z = 1.15;

		thrash_pose.position.x = -0.1;
		thrash_pose.position.y = 2.17;
		thrash_pose.position.z = 1.15;
		thrash_pose.orientation.x = 0.0;
		thrash_pose.orientation.y = 0.0;
		thrash_pose.orientation.z = 0.0;
		thrash_pose.orientation.w = 0.0;

		agv_tf_listener_.waitForTransform("world", "kit_tray_1", ros::Time(0), ros::Duration(10));
		agv_tf_listener_.lookupTransform("/world", "/kit_tray_1", ros::Time(0), agv_tf_transform_);

		agv_position_.position.x = agv_tf_transform_.getOrigin().x();
		agv_position_.position.y = agv_tf_transform_.getOrigin().y();
		agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;


	} else if (arm_id_ == "arm2") {
		home_joint_pose_ = {-0.9, -3.14, -2.0, 2.14, -1.7, -1.59, 0.0};
		//		home_joint_pose_ = {-0.9, -3.14, -0.80, 2.4, -3.14, -1.59, 0.0};
		//		home_joint_fl_arm = {	};
		home_joint_fr_arm = {-0.9, -3.14, -2.0, 2.14, -1.7, 0, 0.0};
		//		quality_cam_joint_position_ = { -1.2, -1.68, -0.38, 1.01,-2.2, -1.51, 0.0  };
		quality_cam_joint_position_ = { -0.75, -1.68, -0.65, 1.01,-2.2, -1.51, 0.0 };
		flip_test_joint_position = {-0.9, 0, -2.3, -2.1, -2.0, 0, 0.0};
		other_side_joint_pose = {0.0, -3.914,-0.7, 0.5, -1.7, -1.59, 0.0};

		trash_bin_joint_position_ = { -1.18, -2.76, -2.08, 2.71, -1.7, -1.51, 0.0 };
		// home_joint_ff_arm = home_joint_fl_arm;

		//		belt_joint_pose_ = {-0.6, 0, -0.7, 1.65, -2.5, -1.59, 0.0 };
		belt_joint_pose_ = { 0.4,-0.9, -0.05,0.2 , -1.65, -1.59, 0.0 };
		static_bin_pose.position.x = -0.04;
		static_bin_pose.position.y = -1.07;
		static_bin_pose.position.z = 1.41;

		quality_static_pose.position.x = 0.19;
		quality_static_pose.position.y = -3.25;
		quality_static_pose.position.z = 1.15;

		thrash_pose.position.x = -0.10;
		thrash_pose.position.y = -2.17;
		thrash_pose.position.z = 1.15;
		thrash_pose.orientation.x = 0.0;
		thrash_pose.orientation.y = 0.0;
		thrash_pose.orientation.z = 0.0;
		thrash_pose.orientation.w = 0.0;

		agv_tf_listener_.waitForTransform("world", "kit_tray_2", ros::Time(0), ros::Duration(10));
		agv_tf_listener_.lookupTransform("/world", "/kit_tray_2", ros::Time(0), agv_tf_transform_);
		agv_position_.position.x = agv_tf_transform_.getOrigin().x();
		agv_position_.position.y = agv_tf_transform_.getOrigin().y();
		agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;


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


void RobotController::GoToTarget(const geometry_msgs::Pose &pose) {
	target_pose_.orientation = face_down_orientation_;
	target_pose_.position = pose.position;
	ros::AsyncSpinner spinner(4);
	robot_move_group_.setPoseTarget(target_pose_);
	spinner.start();
	ros::Duration(interval).sleep();
	if (this->Planner()) {
		ros::Duration(interval).sleep();
		//		ROS_INFO_STREAM("Point success");
		robot_move_group_.move();
		ros::Duration(interval).sleep();
	}
	//	ROS_INFO_STREAM("Point reached...");
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

void RobotController::GoToJointState(const std::vector<double> &joint_val) {
	robot_move_group_.setJointValueTarget(joint_val);
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



void RobotController::GoToQualityCamera() {
	GoToJointState(quality_cam_joint_position_);
	setAtQualitySensor();
}

void RobotController::pickPartFromAGV(const geometry_msgs::Pose &part_pose) {
	pickPart(part_pose, 0.750, 0.25);
}

void RobotController::pickPartFromBin(const geometry_msgs::Pose &part_pose) {
	pickPart(part_pose, 0.725, 0.7);
}

void RobotController::pickPart(const geometry_msgs::Pose &part_pose, double bin_height, double pickupheight) {
	double bin_x_dis = 0.0; // Trial error
	if(part_pose.position.x < -0.40 ) {
		bin_x_dis = 0.1;
	}
	if(part_pose.position.x < -0.45 ) {
		bin_x_dis = 0.17;
	}
	if(part_pose.position.x > -0.20 ) {
		bin_x_dis = -0.1;
	}
	double object_thickness = std::fabs(part_pose.position.z - bin_height); //half odf the object thickness

	geometry_msgs::Pose inter_pose = part_pose;
	inter_pose.position.z = part_pose.position.z + 4 * object_thickness + pickupheight;
	inter_pose.position.x += bin_x_dis;
	GoToTarget(inter_pose);
	inter_pose.position.z = part_pose.position.z + 4 * object_thickness + 0.2;
	inter_pose.position.x -= bin_x_dis;
	GoToTarget(inter_pose);
	geometry_msgs::Pose arrival_pose = part_pose;

	arrival_pose.position.z = part_pose.position.z + 1.1 * object_thickness + 0.15;

	GoToTarget(arrival_pose);
	GripperToggle(true);
	ROS_WARN_STREAM("Gripper toggled");
	while (!isPartAttached()) {
		ROS_WARN_STREAM("Part not attached");
		if(std::fabs(arrival_pose.position.z - bin_height) > 0.02) {
			arrival_pose.position.z -=std::fabs(arrival_pose.position.z - bin_height)/2;
		} else {
			arrival_pose.position.z -= std::min(std::max(1.5*object_thickness, 0.02), 0.1);
		}
		if(arrival_pose.position.z <= bin_height ) {
			arrival_pose.position.z = bin_height;
		}
		GoToTarget(arrival_pose);
		//		ros::Duration(interval).sleep();
	}
	ROS_INFO_STREAM("Part attached");
	arrival_pose.position.z = part_pose.position.z + 1.1 * object_thickness + 0.05;
	GoToTarget(arrival_pose);
	inter_pose.position.z = part_pose.position.z + 4 * object_thickness + pickupheight;
	ros::Duration(interval).sleep();
	GoToTarget(inter_pose);
	ros::Duration(interval).sleep();
	inter_pose.position.x += bin_x_dis;
	GoToTarget(inter_pose);
	ros::Duration(interval).sleep();
	// GoToBinStaticPosition();
	//	ROS_INFO_STREAM("Coming back To BinStaticPosition");
}

void RobotController::dropPart(const geometry_msgs::Pose &part_pose) {
	ROS_INFO_STREAM("Droping Part");
	ros::Duration(interval).sleep();

	geometry_msgs::Pose target_top_pose_1 = part_pose;
	target_top_pose_1.position.z += 0.5;
	GoToTarget(target_top_pose_1);
	ros::Duration(interval).sleep();
	geometry_msgs::Pose target_pose = part_pose;
	target_pose.position.z += 0.1;
	GoToTarget(target_pose);
	GripperToggle(false);

	GoToTarget(target_top_pose_1);
	ros::Duration(interval).sleep();
}

void RobotController::dropPartinAGV(const geometry_msgs::Pose &part_pose) {
	ROS_INFO_STREAM("Droping Part");
	ros::Duration(interval).sleep();

	geometry_msgs::Pose target_top_pose_1 = part_pose;
	target_top_pose_1.position.z += 0.2;
	GoToTarget(target_top_pose_1);
	ros::Duration(interval).sleep();
	geometry_msgs::Pose target_pose = part_pose;
	target_pose.position.z += 0.1;
	GoToTarget(target_pose);
	GripperToggle(false);

	GoToTarget(target_top_pose_1);
	ros::Duration(interval).sleep();
}

void RobotController::GoToTargetForFlip(const geometry_msgs::Pose &pose){
	ros::AsyncSpinner spinner(4);
	robot_move_group_.setPoseTarget(pose);
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


void RobotController::flipPart(OrderPart *order_)
{
	if (order_->isFlipRequired())
	{
		ROS_INFO_STREAM("in flip part function.");
		// logic to flip the part
		// define pose
		// move close to the bin surface?
		// roll the object
		//tf2::Quaternion myQuaternion;
		auto flip_intermediate_pose_ = getCurrentPose();  // get the current pose of the robot ee_link
		//myQuaternion.setRPY(1.558344, 0, 0);              // rotate about roll axis by 90deg
		flip_intermediate_pose_.position.x += 0.2;
		flip_intermediate_pose_.position.z += 0.2;


		GoToTargetForFlip(flip_intermediate_pose_);  // go to this pose

		flip_intermediate_pose_.orientation = face_right_orientation_;
		GoToTargetForFlip(flip_intermediate_pose_);  // go to this pose

		flip_intermediate_pose_.position.z -= 0.3;
		GoToTargetForFlip(flip_intermediate_pose_);  // go to this pose
		ros::Duration(0.4).sleep();



		ROS_INFO_STREAM("gripper deactivating.");
		GripperToggle(false);
		ros::Duration(1.0).sleep();

		flip_intermediate_pose_.position.z += 0.4;
		flip_intermediate_pose_.position.y += 0.1;
		GoToTargetForFlip(flip_intermediate_pose_);
		flip_intermediate_pose_.position.y -= 0.4;
		GoToTargetForFlip(flip_intermediate_pose_);

		ROS_INFO_STREAM("going to left orientation.");
		// flip_intermediate_pose_.position.y += 0.35;
		// flip_intermediate_pose_.position.z -= 0.3;
		//flip_intermediate_pose_.orientation = face_test_orientation_;
		//GoToTargetForFlip(flip_intermediate_pose_);

		ROS_INFO_STREAM(flip_intermediate_pose_.position.y);
		flip_test_joint_position = {flip_intermediate_pose_.position.y + 0.7, 0, -2.3, -2.1, -2.0, 0, 0.0};
		GoToJointState(flip_test_joint_position);
		flip_intermediate_pose_.orientation = face_test_orientation_;

		for(auto x = 0.0; x < 0.2; x += 0.1){
			flip_intermediate_pose_.position.y += 0.1;
			flip_intermediate_pose_.position.z -= 0.2;
			GoToTargetForFlip(flip_intermediate_pose_);
			ROS_INFO_STREAM("inside for loop: " << x);
		}

		ROS_INFO_STREAM("outside for loop! ");
		flip_intermediate_pose_.position.y += 0.005;
		GoToTargetForFlip(flip_intermediate_pose_);
		ros::Duration(0.1).sleep();
		ROS_INFO_STREAM("attaching object after flipping.");
		GripperToggle(true);
		ros::Duration(0.5).sleep();

		// after flipping the part set flip part = true
		// order_->setFlipPart();
		// get pose from logical camera
		//pickFlipPart(flip_intermediate_pose_);
	}
}

void RobotController::GoToOtherSideliveryPose(){
	GoToJointState(other_side_joint_pose);
}

//void RobotController::flipPart(OrderPart *order_)
//{
//	if (order_->isFlipRequired())
//	{
//		// logic to flip the part
//		// define pose
//		// move close to the bin surface?
//
//		// roll the object
//		tf2::Quaternion myQuaternion;
//		auto flip_intermediate_pose_ = getCurrentPose();  // get the current pose of the robot ee_link
//		myQuaternion.setRPY(1.558344, 0, 0);              // rotate about roll axis by 90deg
//		flip_intermediate_pose_.orientation.x = myQuaternion.x();
//		flip_intermediate_pose_.orientation.y = myQuaternion.y();
//		flip_intermediate_pose_.orientation.z = myQuaternion.z();
//		flip_intermediate_pose_.orientation.w = myQuaternion.w();
//
//		GoToTarget(flip_intermediate_pose_);  // go to this pose
//		GripperToggle(false);
//		ros::Duration(0.2).sleep();
//
//		flip_intermediate_pose_.position.z += 0.2;
//		flip_intermediate_pose_.position.y -= 0.4;
//		GoToTarget(flip_intermediate_pose_);
//
//		flip_intermediate_pose_ = getCurrentPose();
//		myQuaternion.setRPY(3.14, 0, 0);  // rotate about roll axis by 180deg
//		flip_intermediate_pose_.orientation.x = myQuaternion.x();
//		flip_intermediate_pose_.orientation.y = myQuaternion.y();
//		flip_intermediate_pose_.orientation.z = myQuaternion.z();
//		flip_intermediate_pose_.orientation.w = myQuaternion.w();
//		GoToTarget(flip_intermediate_pose_);
//		ros::Duration(0.2).sleep();
//
//		// after flipping the part set flip part = true
//		// order_->setFlipPart();
//		// get pose from logical camera
//		pickFlipPart(flip_intermediate_pose_);
//	}
//}

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

bool RobotController::pickPartFromBelt(geometry_msgs::Pose* part_pose)
{
	//	while(part_pose == nullptr) {
	////		ROS_WARN_STREAM("Waiting for part come under belt camera for pickup...");
	//	}

	//	ROS_INFO_STREAM("RC ACTION =>" << *part_pose);
	// TODO passing current pose as making a local copy of the part pose
	GripperToggle(true);
	double belt_speed = 0.3; // Trial error
	double belt_height = 0.90;
	double object_thickness = std::fabs(part_pose->position.z - belt_height)+0.015; //half odf the object thickness
	geometry_msgs::Pose arrival_pose = *part_pose;
	arrival_pose.position.z = part_pose->position.z + 1.1 * object_thickness;
	arrival_pose.position.y -= 1.5*belt_speed;
	GoToTarget(arrival_pose);

	ROS_WARN_STREAM("Gripper toggled");
	geometry_msgs::Pose picking_pose = *part_pose;
	if (!isPartAttached())
	{
		int count = 0;
		picking_pose.position.z = part_pose->position.z + object_thickness;
		picking_pose.position.y = part_pose->position.y - 1.5*belt_speed;
		GoToTarget(picking_pose);
		picking_pose.position.z = part_pose->position.z + 1.1 * object_thickness + 0.05;
		picking_pose.position.y = part_pose->position.y- 1.5*belt_speed;
		GoToTarget(picking_pose);
		++count;
		while (!isPartAttached() and count <= 2) {
			ROS_WARN_STREAM("Part not attached");
			picking_pose.position.z = part_pose->position.z + object_thickness + 0.02;
			picking_pose.position.y =  part_pose->position.y - belt_speed;
			GoToTarget(picking_pose);
			picking_pose.position.z = part_pose->position.z + 1.1 * object_thickness + 0.05;
			picking_pose.position.y =  part_pose->position.y - belt_speed;
			GoToTarget(picking_pose);
			++count;
		}
	}
	if (isPartAttached()) {
		ROS_INFO_STREAM("Part attached");
		picking_pose.position.z += 0.15;
		GoToTarget(picking_pose);
		return true;
	} else {return false;}
}
