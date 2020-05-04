/**
 * @file      include/order_manager.h
 * @brief     Header file for order manager
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


#ifndef GROUP6_RWA5_ORDER_MANAGER_H_
#define GROUP6_RWA5_ORDER_MANAGER_H_

#include <list>
#include <map>
#include <string>
#include <iostream>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <order_part.h>
#include <environment.h>

using std::vector;

class OrderManager
{
private:
    ros::NodeHandle order_manager_nh_;
    ros::AsyncSpinner async_spinner;
    ros::Subscriber order_subscriber_;
    osrf_gear::Order* order_;
    std::vector<OrderPart*> current_order_;
    Environment *environment;
    ros::Publisher dynamic_planner;
    std::vector<std::vector<std::map<std::string, std::vector<OrderPart* > > >::iterator>agv1_any;
    std::vector<std::vector<std::map<std::string, std::vector<OrderPart* > > >::iterator>agv2_any;

public:
    explicit OrderManager(Environment *);
    ~OrderManager();
    void comparewithTrayandUpdateOrder();
    void OrderCallback(const osrf_gear::Order::ConstPtr&);
    void setOrderParts(const osrf_gear::Order::ConstPtr& order_msg);
    std::map<std::string, std::vector<OrderPart*>> getTrashParts(std::map<std::string, std::vector<geometry_msgs::Pose>>);
    void comparewithTrayandUpdate(std::string agv_id, std::map<std::string, std::vector<geometry_msgs::Pose>> );
    bool comparePose();
    void updatePickupLocation();
    void setArmForAnyParts();
    void UpdateUnavailableParts();
    void UpdateUnavailablePartsforEachAGV(std::string);
    void executeDynamicPlanner();
    bool arePoseSame(geometry_msgs::Pose, geometry_msgs::Pose);
};

#endif //  GROUP6_RWA5_ORDER_MANAGER_H_
