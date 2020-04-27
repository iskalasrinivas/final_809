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

#include <planner.h>
#include <limits>
#include <cmath>

Planner::Planner(Environment *env) : async_spinner(0), env_(env), ordermanager_(env) {  
 async_spinner.start();
 common_pose_ind = 0; // TODO @ Rachith Confirm
 planner_sub_ = planner_nh_.subscribe<std_msgs::Bool>("/ariac/execute_planner", 10 ,&Planner::plancallback, this);
 
 execute_executer = planner_nh_.advertise<std_msgs::Bool>("/ariac/execute_executer", 1000);
 
 // while( not env_.getOrderManagerStatus() ) {
 //  ROS_INFO_STREAM("Waiting for order manager to process...");
 //  ros::Duration(5.0).sleep();
 // }
 // ROS_INFO_STREAM("Planning now...");
 // planner_.plan();

}

Planner::~Planner()
{}

void Planner::plancallback(const std_msgs::Bool::ConstPtr& msg) {
 if(msg->data) {
  plan();
  setPriorityQueue();
  

  std_msgs::Bool msg;
  msg.data = true;
  for(size_t i=0; i<1; ++i){
   execute_executer.publish(msg);
  }
  printOrders();
 }
}

void Planner::plan() {

 ROS_INFO_STREAM("In planning modules's plan()");
 // auto sorted_BinParts = environment->getSortedBinParts();  std::vector<std::map<std::string, std::vector<OrderPart*>>>
 std::vector<std::map<std::string, std::vector<OrderPart*>>>* agv1_OrderParts = env_->getArm1OrderParts(); //  remove from tray, displace, bin to tray p1(mp X -> ep)
 auto agv2_OrderParts = env_->getArm2OrderParts(); // p1(sp -> mp)

 /// Checking if anything needs to be added to arm2 to support arm1

 std::map<std::string, std::vector<OrderPart *>> new_shipment_to_agv2;

 for (auto map_it = agv1_OrderParts->begin(); map_it != agv1_OrderParts->end(); ++map_it) {
  // ROS_INFO_STREAM("plan agv1: first loop");
  for (auto ship_it = map_it->begin(); ship_it != map_it->end(); ++ship_it) {
   // ROS_INFO_STREAM("plan agv1: second loop");
   auto part_type = (*ship_it).first;

   for (auto ord_it = (*ship_it).second.begin(); ord_it != (*ship_it).second.end(); ++ord_it) {

    // TODO add isnan() comparator before doing the rest for part without pickup location
    // Remove try, catch if getcurrentpose works as expected i.e has nan values whern not assigned.
    try {
     auto pose = (*ord_it)->getCurrentPose().position.y;
    } catch (std::exception& e) {
     ROS_ERROR_STREAM(e.what() << std::endl);
     ROS_ERROR_STREAM("Error while checking current pose value when not assigned,  position-> " << (*ord_it)->getCurrentPose().position << "\n");
    }
    
    if( ! std::isnan(std::fabs((*ord_it)->getCurrentPose().position.y))) {
     if ((*ord_it)->getCurrentPose().position.y < 0)
     { // order part is not reachable To-DO ----- make sure value is right
      // add a copy of this part to agv2 order parts
      ROS_INFO_STREAM("Part is not reachable by arm-1, Part: " << (*ord_it)->getPartType() <<" "<< (*ord_it)->getCurrentPose());
      OrderPart* part = new OrderPart();
      part->setShipmentType((*ord_it)->getShipmentType());
      part->addPriority(1);
      part->setPartType((*ord_it)->getPartType());
      part->setCurrentPose((*ord_it)->getCurrentPose());
      // TO-DO ----> set common bin pose so that arm1 can pick it from there.
      // add it to the beginning of agv2 as a new shipment
      // if( common_pose_ind <= 3) {
      //  part->setEndPose(common_pose_[common_pose_ind]);
      //  (*ord_it)->setCurrentPose(common_pose_[common_pose_ind]);
      //  // ROS_INFO_STREAM("PO1 :" << (*ord_it)->getPartType() << " " << (*ord_it)-> getEndPose());
      //  if(common_pose_ind==3) common_pose_ind=0;
      //  else ++common_pose_ind;
      // }

      // part->setEndPose(available_poses_->getAvailableBinPoseArm2());
      geometry_msgs::Pose pose = geometry_msgs::Pose();
      pose.position.x = std::numeric_limits<double>::quiet_NaN();
      pose.position.y = std::numeric_limits<double>::quiet_NaN();
      pose.position.z = std::numeric_limits<double>::quiet_NaN();
      pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
      pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
      pose.orientation.y = std::numeric_limits<double>::quiet_NaN();;
      pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
      part->setEndPose(pose); // set empty end pose to recognise that it needs to call getpose from available_bin_poses

      if(new_shipment_to_agv2.count(part_type)) {
       new_shipment_to_agv2[part_type].emplace_back(part);
      }
      else {
       new_shipment_to_agv2[part_type] = std::vector<OrderPart*>({part});
      }
     }
    }
   }
  }
 }
 // ROS_INFO_STREAM("plan: out of first loop");
 if(new_shipment_to_agv2.size()) {
  env_->getArm2PreOrderParts()->clear();
  env_->getArm2PreOrderParts()->emplace_back(new_shipment_to_agv2);
 }

 /// Checking if anything needs to be added to arm1 to support arm2
 std::map<std::string, std::vector<OrderPart *>> new_shipment_to_agv1;

 for (auto map_it = agv2_OrderParts->begin(); map_it != agv2_OrderParts->end(); ++map_it) {
//  ROS_INFO_STREAM("plan agv2: second loop");
  for (auto ship_it = map_it->begin(); ship_it != map_it->end(); ++ship_it) {

   auto part_type = ship_it->first;

   for (auto ord_it = ship_it->second.begin(); ord_it != ship_it->second.end(); ++ord_it)
   {
    // TODO add isnan() comparator before doing the rest for part without pickup location
    if (!std::isnan(std::fabs((*ord_it)->getCurrentPose().position.y))) {
     if ((*ord_it)->getCurrentPose().position.y > 0) { // order part is not reachable  To-DO ----- make sure value is right
      // add a copy of this part to agv2 order parts
      OrderPart *part = new OrderPart();
      part->setShipmentType((*ord_it)->getShipmentType());
      part->addPriority(1);
      part->setPartType((*ord_it)->getPartType());
      part->setCurrentPose((*ord_it)->getCurrentPose());
      ROS_INFO_STREAM("Part is not reachable by arm-2, part: " << (*ord_it)->getPartType() <<" "<< (*ord_it)->getCurrentPose());
      // TO-DO ---- set ommon bin pose so that arm1 can pick it from there.
      // add it to the beginning of agv2 as a new shipment
      // if( common_pose_ind <= 3) {
      //  part->setEndPose(common_pose_[common_pose_ind]);
      //  (*ord_it)->setCurrentPose(common_pose_[common_pose_ind]);
      //  // ROS_INFO_STREAM("PO2 :" << (*ord_it)->getPartType() << " " << (*ord_it)-> getEndPose());
      //  if(common_pose_ind==3) common_pose_ind=0;
      //  else ++common_pose_ind;
      // }
      // part->setEndPose(available_poses_->getAvailableBinPoseArm1());
      geometry_msgs::Pose pose = geometry_msgs::Pose();
      pose.position.x = std::numeric_limits<double>::quiet_NaN();
      pose.position.y = std::numeric_limits<double>::quiet_NaN();
      pose.position.z = std::numeric_limits<double>::quiet_NaN();
      pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
      pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
      pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
      pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
      part->setEndPose(pose);
      if (new_shipment_to_agv1.count(part_type)) {
       new_shipment_to_agv1[part_type].emplace_back(part);
      }
      else {
       new_shipment_to_agv1[part_type] = std::vector<OrderPart *>({part});
      }
     }
    }
   }
  }
 }

 if (new_shipment_to_agv1.size()) {
  env_->getArm1PreOrderParts()->clear();
  env_->getArm1PreOrderParts()->emplace_back(new_shipment_to_agv1);
 }
}