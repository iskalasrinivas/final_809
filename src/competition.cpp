/**
 * @file      src/competition.cpp
 * @brief     Source file for Competition
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

#include <algorithm>
#include <vector>
#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <competition.h>

Competition::Competition(Environment* env)
: async_spinner(0),
  current_score_(0),
  env_(env),
  sensor_(env),
  order_manager_(env),
  dplan_(env) {

 // manager_(&comp_nh_);
 async_spinner.start();
 env_ = new Environment();
 current_score_subscriber = comp_nh_.subscribe("/ariac/current_score", 10,&Competition::current_score_callback, this);

 // Subscribe to the '/ariac/competition_state' topic.
 competition_state_subscriber = comp_nh_.subscribe("/ariac/competition_state", 10, &Competition::competition_state_callback, this);

 StartCompetition();
}

Competition::~Competition(){

}

/// Called when a new message is received.
void Competition::current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
 if (msg->data != current_score_)
 {
  ROS_INFO_STREAM("Score: " << msg->data);
 }
 current_score_ = msg->data;
}

/// Called when a new message is received.
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
 if (msg->data == "done" && competition_state_ != "done")
 {
  ROS_INFO("Competition ended.");
 }
 competition_state_ = msg->data;
}

void Competition::StartCompetition() {
 ROS_INFO("Competition function.");
 // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
 ros::ServiceClient start_client =
   comp_nh_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
 // If it's not already ready, wait for it to be ready.
 // Calling the Service using the client before the server is ready would fail.
 if (!start_client.exists()) {
  ROS_INFO("Waiting for the competition to be ready...");
  start_client.waitForExistence();
  ROS_INFO("Competition is now ready.");
 }
 ROS_INFO("Requesting competition start...");
 std_srvs::Trigger srv;  // Combination of the "request" and the "response".
 start_client.call(srv);  // Call the start Service.
 if (!srv.response.success) {  // If not successful, print out why.
  ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
 } else {
  ROS_INFO("Competition started!");
 }
}

void Competition::EndCompetition() {
 ros::ServiceClient start_client =
   comp_nh_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

 if (!start_client.exists()) {
  ROS_INFO("Waiting for the competition to be End...");
  start_client.waitForExistence();
  ROS_INFO("Competition  now Ended.");
 }
 ROS_INFO("Requesting competition End...");
 std_srvs::Trigger srv;
 start_client.call(srv);
 if (!srv.response.success) {
  ROS_ERROR_STREAM(
    "Failed to End the competition: " << srv.response.message);
 } else {
  ROS_INFO("Competition Ended!");
 }
}
