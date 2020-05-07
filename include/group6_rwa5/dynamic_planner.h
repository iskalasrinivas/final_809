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


#ifndef GROUP6_RWA5_DYNAMICPLANNER_H
#define GROUP6_RWA5_DYNAMICPLANNER_H

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
#include <priority_queue.h>
#include <executor.h>
#include <thread>



class DynamicPlanner
{
private:
    ros::NodeHandle dplanner_nh_;
 ros::AsyncSpinner async_spinner;
 std::thread arm1_thread;
 std::thread arm2_thread;
    Executor exe_;
    Environment* env_;
 ros::Subscriber dplanner_sub_;
 std::vector<std::vector<OrderPart*>>::iterator current_shipment_it;

public:
    DynamicPlanner(Environment*);
    ~DynamicPlanner();

 void dynamicPlannerCallBack(const std_msgs::Bool::ConstPtr& );

 void deliverThePartinBin(OrderPart * oPart);

 void updatePickupCoordinate(OrderPart * oPart);

 void updateDeliveryCoordinate(OrderPart * oPart);
 
 void updatePickPoseFromBin(OrderPart* );

 void pickPartFromBelt(geometry_msgs::Pose*);

 bool isPoseSame(geometry_msgs::Pose, geometry_msgs::Pose);

    bool isShipmentofTray1Checked();
 
 bool isShipmentofTray2Checked();

 void dynamicPlanning();

 void dynamicPlanningforArm1();
 
 int updatePickupLocation(std::string, OrderPart*);

 void dynamicPlanningforArm2();

// bool inVicinity(const geometry_msgs::Pose& , RobotController);

 void p();

 bool completeSinglePartOrder(RobotController*, OrderPart *, bool);

 bool isShipmentofTrayChecked(std::string agv_id);

 bool isShipmentComplete(std::string, std::vector<std::vector<OrderPart*>>::iterator);

 bool isitCurrentShipment();

 void removeUnwantedPartfromTray(std::string, std::vector<std::vector<OrderPart*>>::iterator);

 void clearTray(RobotController*, std::string);

 void checkBeforeDispatch(RobotController*, std::vector<std::vector<OrderPart*>>::iterator, std::string);

 bool arePoseSame(geometry_msgs::Pose, geometry_msgs::Pose);
 // void flipPart(OrderPart * oPart);
};

#endif //GROUP6_RWA5_DYNAMICPLANNER_H
