/**
 * @file      src/executer.cpp
 * @brief     Header file for building map
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

#include <executor.h>
#include <order_part.h>
#include <osrf_gear/AGVControl.h>
#include <std_srvs/Trigger.h>
#include <environment.h>
#include <robot_controller.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Executor::Executor(Environment* env):  arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false), arm1_("arm1"), arm2_("arm2") {
    
 arm_1_joint_trajectory_publisher_ = executer_nh_.advertise<trajectory_msgs::JointTrajectory>( "/ariac/arm1/arm/command", 10);

 arm_2_joint_trajectory_publisher_ = executer_nh_.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm2/arm/command", 10);

arm1_thread = std::thread(&RobotController::initialSequence, &arm1_);
arm2_thread = std::thread(&RobotController::initialSequence, &arm2_);

arm1_thread.join();
arm2_thread.join();

}

Executor::~Executor()
{
}

// Called when a new JointState message is received.

void Executor::arm_1_joint_state_callback(
  const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
 ROS_INFO_STREAM_THROTTLE(10,
   "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
 // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
 arm_1_current_joint_states_ = *joint_state_msg;
 if (!arm_1_has_been_zeroed_) {
  arm_1_has_been_zeroed_ = true;
  ROS_INFO("Sending arm to zero joint positions...");
  send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
 }
}

void Executor::arm_2_joint_state_callback(
  const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
 ROS_INFO_STREAM_THROTTLE(10,
   "Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
 // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
 arm_2_current_joint_states_ = *joint_state_msg;
 if (!arm_2_has_been_zeroed_) {
  arm_2_has_been_zeroed_ = true;
  ROS_INFO("Sending arm 2 to zero joint positions...");
  send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
 }
}


/// Create a JointTrajectory with all positions set to zero, and command the arm.
void Executor::send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
 // Create a message to send.
 trajectory_msgs::JointTrajectory msg;

 // Fill the names of the joints to be controlled.
 // Note that the vacuum_gripper_joint is not controllable.
 msg.joint_names.clear();
 msg.joint_names.push_back("shoulder_pan_joint");
 msg.joint_names.push_back("shoulder_lift_joint");
 msg.joint_names.push_back("elbow_joint");
 msg.joint_names.push_back("wrist_1_joint");
 msg.joint_names.push_back("wrist_2_joint");
 msg.joint_names.push_back("wrist_3_joint");
 msg.joint_names.push_back("linear_arm_actuator_joint");
 // Create one point in the trajectory.
 msg.points.resize(1);
 // Resize the vector to the same length as the joint names.
 // Values are initialized to 0.
 msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
 // How long to take getting to the point (floating point seconds).
 msg.points[0].time_from_start = ros::Duration(0.001);
 ROS_INFO_STREAM("Sending command:\n" << msg);
 joint_trajectory_publisher.publish(msg);
}

RobotController* Executor::getArm1Object(){
    return &arm1_;
}

RobotController* Executor::getArm2Object(){
    return &arm2_;
}

void Executor::SendAGV1(){
    ros::ServiceClient start_client =
      executer_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else {
  ROS_INFO("Service succeeded.");
 }
}

void Executor::SendAGV2() {
 ros::ServiceClient start_client =
   executer_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else {
  ROS_INFO("Service succeeded.");
 }

}
