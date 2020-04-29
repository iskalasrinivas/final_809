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

#ifndef GROUP6_RWA4_COMPETITION_H
#define GROUP6_RWA4_COMPETITION_H

#include <dynamic_planner.h>
#include <executor.h>
#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <environment.h>
#include <sensor_manager.h>
#include <order_manager.h>
#include <dynamic_planner.h>


//class AriacOrderManager;

class Competition {

  private:

    ros::NodeHandle comp_nh_;
    ros::AsyncSpinner async_spinner;
    Environment* env_;
    OrderManager order_manager_;
    SensorManager sensor_;
    DynamicPlanner dplan_;

    std::string competition_state_;
    double current_score_;

    // osrf_gear::Order order_;
    ros::Subscriber current_score_subscriber;

    // Subscribe to the '/ariac/competition_state' topic.
    ros::Subscriber competition_state_subscriber;

  public:
    Competition(Environment*);
    ~Competition();
    /// Called when a new message is received.
    void current_score_callback(const std_msgs::Float32::ConstPtr & );

    /// Called when a new message is received.
    void competition_state_callback(const std_msgs::String::ConstPtr & );

    void StartCompetition();

    void EndCompetition();

};


#endif //GROUP6_RWA4_COMPETITION_H
