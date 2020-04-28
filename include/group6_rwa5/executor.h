/**
 * @file      include/competition.h
 * @brief     Header file for competition
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


#ifndef GROUP6_RWA5_EXECUTOR_H
#define GROUP6_RWA5_EXECUTOR_H

#include <order_part.h>
#include <environment.h>
#include <osrf_gear/Order.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <robot_controller.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <transformer.h>
#include <robot_controller.h>
#include <thread>


class Executor
{
private:
std::thread arm1_thread;
 std::thread arm2_thread;
ros::Publisher arm_1_joint_trajectory_publisher_;
 ros::Publisher arm_2_joint_trajectory_publisher_;

 sensor_msgs::JointState arm_1_current_joint_states_;
 sensor_msgs::JointState arm_2_current_joint_states_;

 bool arm_1_has_been_zeroed_;
 bool arm_2_has_been_zeroed_;

 RobotController arm1_; 
 RobotController arm2_;

 ros::NodeHandle executer_nh_;

public:
    Executor(Environment*);
    ~Executor();

    /// Called when a new JointState message is received.
 void arm_1_joint_state_callback(const sensor_msgs::JointState::ConstPtr & );

 void arm_2_joint_state_callback(const sensor_msgs::JointState::ConstPtr &);

 /// Create a JointTrajectory with all positions set to zero, and command the arm.
 void send_arm_to_zero_state(ros::Publisher &);

    RobotController* getArm1Object();

    RobotController* getArm2Object();

 void SendAGV1();

 void SendAGV2();




};

#endif //GROUP6_RWA5_EXECUToR_H
