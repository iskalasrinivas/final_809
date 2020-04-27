/**
 * @file      src/dynamic_planner.cpp
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

#include <dynamic_planner.h>
#include <environment.h>
#include <order_part.h>
#include <robot_controller.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

DynamicPlanner::DynamicPlanner(Environment* env) : async_spinner(0), env_(env), exe_(env)
{
	// ros::AsyncSpinner async_spinner(0);
	async_spinner.start();

	dplanner_sub_ = dplanner_nh_.subscribe<std_msgs::Bool>("/ariac/dynamic_planner", 10,
			&DynamicPlanner::dynamicPlannerCallBack, this);
}
DynamicPlanner::~DynamicPlanner()
{
}

void DynamicPlanner::dynamicPlannerCallBack(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_WARN_STREAM("<<<<<In Dynamic Planner>>>>>");
	if (msg->data)
	{
		arm1_thread = std::thread(&DynamicPlanner::dynamicPlanningforArm1, this);
		arm2_thread = std::thread(&DynamicPlanner::dynamicPlanningforArm2, this);

		arm1_thread.join();
		arm2_thread.join();
	}
}

void DynamicPlanner::updatePickPoseFromBin(OrderPart* order_)
{
	// we need to search for the part type in bin parts
	// if found update pickpose of parts
	env_->setBinCameraRequired(true);

	while (!env_->isAllBinCameraCalled())
	{
		ROS_INFO_STREAM("ALL BIN CAMERA IS NOT CALLED...Waiting...");
		ros::Duration(0.1).sleep();
	}
	auto part_type = order_->getPartType();

	if (env_->isAllBinCameraCalled())
	{
		auto allbinpart = env_->getSortedBinParts();  // std::map<std::string, std::vector<geometry_msgs::Pose> >*
		if (allbinpart->count(part_type))
		{
			auto new_pose = (*allbinpart)[part_type].back();
			order_->setCurrentPose(new_pose);
			env_->setBinCameraRequired(false);
			env_->setAllBinCameraCalled(false);
		}
	}
}

// void DynamicPlanner::flipPart(OrderPart *order_)
// {
//  if (!order_->getFlipPart())
//  {
//   // logic to flip the part
//   auto target_pose = ; //define pose
//   arm1_.GoToTarget(target_pose);
//   arm1_.GripperToggle(false);
//   ros::Duration(0.2).sleep();

//   // after flipping the part set flip part = true
//   order_->setFlipPart();
//  }
// }

// void DynamicPlanner::dynamicPlanning()
// {
//    ("<<<<<<<In Execute modules");
//   // Execute Pre-order tasks of arm1
//   auto arm1_ = exe_.getArm1Object();
//   auto arm2_ = exe_.getArm2Object();

//   if (env_->isConveyor1Triggered())
//   {
//  // call function to handle cb pickup
//  arm1_->GoToBeltHome();
//  arm1_->pickPartFromBelt();
//   }
//   if (env_->isConveyor2Triggered())
//   {
//  // call function to handle cb pickup
//  arm2_->GoToBeltHome();
//  arm2_->pickPartFromBelt();
//   }

//   std::vector<std::map<std::string, std::vector<OrderPart*>>>* arm1preOrderParts =
//    env_->getArm1PreOrderParts();  // std::vector<std::map<std::string, std::vector<OrderPart*>>>*
//   for (auto po1_vec_it = arm1preOrderParts->begin(); po1_vec_it != arm1preOrderParts->end(); ++po1_vec_it)
//   {
//  for (auto po1_map_it = po1_vec_it->begin(); po1_map_it != po1_vec_it->end(); ++po1_map_it)
//  {
//    for (auto po1_it = po1_map_it->second.begin(); po1_it != po1_map_it->second.end(); ++po1_it)
//    {  // pol_it is basically iterator to std::vector<OrderPart*>
//   ROS_INFO_STREAM("<<< Pre Order Arm1 >>>");
//   // TODO
//   if (env_->isConveyor1Triggered())
//   {
//     // call function to handle cb pickup
//     arm1_->GoToBeltHome();
//     arm1_->startBeltOperation();
//   }
//   arm1_->pickPart((*po1_it)->getCurrentPose());
//   ROS_INFO_STREAM("<<<<<<<arm1 going to quality bin");
//   arm1_->GoToQualityCameraFromBin();
//   env_->setSeeQualityCamera1(true);
//   while (not env_->isQuality1Called())
//   {
//     ros::Duration(0.1).sleep();
//     ROS_WARN_STREAM("Waiting for Quality Camera 1 to be called");
//   }
//   if (env_->isQualityCamera1Partfaulty())
//   {
//     ROS_WARN_STREAM("Part is faulty");
//     arm1_->dropInTrash();
//     updatePickPoseFromBin((*po1_it));
//     --po1_it;
//   }
//   else
//   {
//     arm1_->deliverPart((*po1_it)->getEndPose());
//   }
//   env_->setSeeQualityCamera2(true);
//    }
//  }
//   }
//   arm1_->SendRobotHome();

//   // Execute Pre-order tasks of arm2

//   std::vector<std::map<std::string, std::vector<OrderPart*>>>* arm2preOrderParts = env_->getArm2PreOrderParts();
//   for (auto po2_vec_it = arm2preOrderParts->begin(); po2_vec_it != arm2preOrderParts->end(); ++po2_vec_it)
//   {
//  for (auto po2_map_it = po2_vec_it->begin(); po2_map_it != po2_vec_it->end(); ++po2_map_it)
//  {
//    for (auto po2_it = po2_map_it->second.begin(); po2_it != po2_map_it->second.end(); ++po2_it)
//    {
//   ROS_INFO_STREAM("<<< Pre Order Arm2 >>>");
//   arm2_->pickPart((*po2_it)->getCurrentPose());
//   ROS_INFO_STREAM("<<<<<<<arm2 going to quality bin");
//   arm2_->GoToQualityCameraFromBin();
//   env_->setSeeQualityCamera2(true);
//   while (not env_->isQuality2Called())
//   {
//     ros::Duration(0.1).sleep();
//     ROS_WARN_STREAM("Waiting for Quality Camera 2 to be called");
//   }
//   if (env_->isQualityCamera2Partfaulty())
//   {
//     ROS_WARN_STREAM("Part is faulty");
//     arm2_->dropInTrash();
//     updatePickPoseFromBin((*po2_it));
//     --po2_it;
//   }
//   else
//   {
//     arm2_->deliverPart((*po2_it)->getEndPose());
//   }
//   env_->setSeeQualityCamera2(false);
//    }
//  }
//   }
//   arm2_->SendRobotHome();

//   auto arm1OrderParts = env_->getArm1OrderParts();  // std::vector<std::map<std::string, std::vector<OrderPart* > >
//   >* for (auto o1_vec_it = arm1OrderParts->begin(); o1_vec_it != arm1OrderParts->end(); ++o1_vec_it)
//   {
//  for (auto o1_map_it = o1_vec_it->begin(); o1_map_it != o1_vec_it->end(); ++o1_map_it)
//  {
//    for (auto o1_it = o1_map_it->second.begin(); o1_it != o1_map_it->second.end(); ++o1_it)
//    {
//   ROS_INFO_STREAM("<<< Order Arm1 >>>");
//   arm1_->pickPart((*o1_it)->getCurrentPose());
//   // arm1.flipPart((*o1_it));
//   arm1_->GoToQualityCameraFromBin();
//   env_->setSeeQualityCamera1(true);
//   while (not env_->isQuality1Called())
//   {
//     ros::Duration(0.1).sleep();
//     ROS_WARN_STREAM("Waiting for Quality Camera 1 to be called");
//   }
//   if (env_->isQualityCamera1Partfaulty())
//   {
//     ROS_WARN_STREAM("Part is faulty");
//     arm1_->dropInTrash();
//     updatePickPoseFromBin((*o1_it));
//     --o1_it;
//   }
//   else
//   {
//     ROS_INFO_STREAM("Part is not faulty");
//     ROS_INFO_STREAM("Dropping in AGV");
//     arm1_->deliverPart((*o1_it)->getEndPose());
//     // if()
//     // removeItemFromOrderPart((*o1_it));
//     // deleteTheOrderPart((*o1_it));
//   }
//    }
//  }
//   }

//   auto arm2OrderParts = env_->getArm2OrderParts();  // std::vector<std::map<std::string, std::vector<OrderPart* > >
//   >*

//   for (auto o2_vec_it = arm2OrderParts->begin(); o2_vec_it != arm2OrderParts->end(); ++o2_vec_it)
//   {
//  for (auto o2_map_it = o2_vec_it->begin(); o2_map_it != o2_vec_it->end(); ++o2_map_it)
//  {
//    for (auto o2_it = o2_map_it->second.begin(); o2_it != o2_map_it->second.end(); ++o2_it)
//    {
//   ROS_INFO_STREAM("<<< Order Arm2 >>>");
//   arm2_->pickPart((*o2_it)->getCurrentPose());
//   // arm1.flipPart((*o1_it));
//   arm2_->GoToQualityCameraFromBin();
//   env_->setSeeQualityCamera2(true);
//   while (not env_->isQuality2Called())
//   {
//     ros::Duration(0.1).sleep();
//     ROS_WARN_STREAM("Waiting for Quality Camera 2 to be called");
//   }
//   if (env_->isQualityCamera1Partfaulty())
//   {
//     ROS_WARN_STREAM("Part is faulty");
//     arm2_->dropInTrash();
//     updatePickPoseFromBin((*o2_it));
//     --o2_it;
//   }
//   else
//   {
//     ROS_INFO_STREAM("Part is not faulty");
//     ROS_INFO_STREAM("Dropping in AGV");
//     arm2_->deliverPart((*o2_it)->getEndPose());
//     // if()
//     // removeItemFromOrderPart((*o1_it));
//     // deleteTheOrderPart((*o1_it));
//   }
//    }
//  }
//   }
// }

bool DynamicPlanner::isPoseSame(geometry_msgs::Pose tray_pose, geometry_msgs::Pose order_pose)
{
	double px = pow(tray_pose.position.x - order_pose.position.x, 2);
	double py = pow(tray_pose.position.y - order_pose.position.y, 2);
	double pz = pow(tray_pose.position.z - order_pose.position.z, 2);

	double ox = tray_pose.orientation.x - order_pose.orientation.x;
	double oy = tray_pose.orientation.y - order_pose.orientation.y;
	double oz = tray_pose.orientation.z - order_pose.orientation.z;
	double ow = tray_pose.orientation.w - order_pose.orientation.w;

	bool pose_same = true;  // TODO

	double dist = sqrt(px + py + pz);

	if (dist < 0.01 and pose_same)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//@saurav: To be reviewed from here

bool DynamicPlanner::isShipmentofTrayChecked(std::string agv_id)
{
	std::map<std::string, std::vector<geometry_msgs::Pose>>* tray1_parts = env_->getTray1Parts();

	auto shipment = (*(env_->getCompletedShipment()))[agv_id];

	int shipment_id = shipment.front()->getShipmentId();

	auto ground_truth_shipment = (*(env_->getShipments()))[shipment_id];

	for (auto traypart_map : *(tray1_parts))
	{
		auto part_type = traypart_map.first;
		if(traypart_map.second.size() != ground_truth_shipment[part_type]) {
			return false;
		}
	}
	return true;
}

// function to process orderparts from priortity queue  for arm1
void DynamicPlanner::dynamicPlanningforArm1()
{
	// make sure this runs only after planner is completed
	// ie. when new order comes, we are clearing the priortiy queue. So we will have to wait till we re-populate it.
	// planner does that. so wait!!
	ROS_WARN_STREAM("<<<<<In dynamicPlanningforArm1: 1>>>>>");
    auto arm1_pq = (*(env_->getPriorityQueue()))["agv1"];
	auto arm1_ = exe_.getArm1Object();
	int previousShipmentId = arm1_pq.top()->getShipmentId();
	ROS_WARN_STREAM("<<<<<In dynamicPlanningforArm1:  2 >>>>>");
	while (!arm1_pq.empty())
	{
		auto order_part = arm1_pq.top();
		arm1_pq.pop();

		// check pick up location for part
		int retVal = updatePickupLocation(order_part);

		bool delivered = false;

		if (retVal == 0)
		{  // pick up from bin
			bool delivered = arm1_->completeSinglePartOrder(order_part);
		}
		else if (retVal == 1)
		{  // no action
			continue;
		}
		else if (retVal == 2)
		{  // no action
			continue;
		}
		else if (retVal == 3)
		{  // action for belt
			arm1_->GoToBeltHome();
			// TODO set the sequence of action here it self using pointer since the location is updating continuously
			if ((*(env_->getPickupLocations())).count("agv1") and (*(env_->getPickupLocations()))["agv1"].count(order_part->getPartType())) {
				auto arm1_pck_lctn = (*(env_->getPickupLocations()))["agv1"][order_part->getPartType()];

				arm1_->pickPartFromBelt(arm1_pck_lctn);
				bool delivered = arm1_->completeSinglePartOrder(order_part);
			}
		}

		if (!delivered)
		{
			arm1_pq.push(order_part);
		}
		else
		{
			(*(env_->getCompletedShipment()))[order_part->getAgvId()].push_back(order_part);
		}

		if (isShipmentofTrayChecked("arm1"))
		{
			exe_.SendAGV1();
		}
	}
}

// function to process orderparts from priortity queue  for arm2
void DynamicPlanner::dynamicPlanningforArm2()
{
	// make sure this runs only after planner is completed
	// ie. when new order comes, we are clearing the priortiy queue. So we will have to wait till we re-populate it.
	// planner does that. so wait!!
	ROS_WARN_STREAM("<<<<<In dynamicPlanningforArm2 :1 >>>>>");
	auto arm2_pq = (*(env_->getPriorityQueue()))["agv2"];
	auto arm2_ = exe_.getArm2Object();
	int previousShipmentId = arm2_pq.top()->getShipmentId();
	ROS_WARN_STREAM("<<<<<In dynamicPlanningforArm2 :2 >>>>>");

	while (!arm2_pq.empty())
	{
		auto order_part = arm2_pq.top();
		arm2_pq.pop();

		bool delivered = false;

		// check pick up location for part
		int retVal = updatePickupLocation(order_part);

		if (retVal == 0)
		{ // pick up from bin
			bool delivered = arm2_->completeSinglePartOrder(order_part);
		}
		else if (retVal == 1)
		{ // no action
			continue;
		}
		else if (retVal == 2)
		{ // no action
			continue;
		}
		else if (retVal == 3)
		{ // action for belt
			arm2_->GoToBeltHome();
			// TODO set the sequence of action here it self using pointer since the location is updating continuously
			if ((*(env_->getPickupLocations())).count("agv2") and (*(env_->getPickupLocations()))["agv2"].count(order_part->getPartType())) {
				auto arm2_pck_lctn = (*(env_->getPickupLocations()))["agv2"][order_part->getPartType()];

				arm2_->pickPartFromBelt(arm2_pck_lctn);
				bool delivered = arm2_->completeSinglePartOrder(order_part);
			}
		}

		if (!delivered)
		{
			arm2_pq.push(order_part);
		}
		else
		{
			(*(env_->getCompletedShipment()))[order_part->getAgvId()].push_back(order_part);
		}

		if (isShipmentofTrayChecked("arm2"))
		{
			exe_.SendAGV2();
		}
	}
}

// Once we have order, update currentpose based on poses from binparts, if it is part of BIN OFCOURSE!!
/*
 return types:
 0 - avaiable in bin and is reachable : Action for bin
 1 - avaialbe in bin and is not reachable : No Action
 2 - not availabnle in bin : No Action
 3 - Highest priortity part waiting for part in belt : Action for belt
 */
int DynamicPlanner::updatePickupLocation(OrderPart* part)
{
	// given order part

	// if avaiable and reachable  update pick location
	// if only available but not reachable : reduce priority in your que and push it to the other queue are with higher
	// prioiry if not available  : reduce the priority to minimum  in your queue : push it to list of unavailable parts

	std::map<std::string, std::vector<geometry_msgs::Pose>>* binParts = env_->getSortedBinParts();

	ROS_INFO_STREAM("<<<Updating Pick up Location>>>");
	ROS_INFO_STREAM("<<<Reducing priority of the Conveyer parts with unknown Pick up Location");

	if (!part->isOfHighestPriority())
	{  // if higest priority belt trigger will change it
		std::string part_type = part->getPartType();

		// first check if part_type is in any of the bins (NA NR)
		if (!binParts->count(part_type))
		{
			ROS_INFO_STREAM("Part is not avaialble in any bins....so picking up from conveyor belt!!!");
			part->setLowestPriority();           // set lowest priority :INT_MAX
			part->setStatic(false);              // make it non-static for as part of conveyor part
			env_->pushToUnavailableParts(part);  // set
			(*(env_->getPriorityQueue()))[part->getAgvId()].push(part);
			return 2;
		}
		else
		{
			// iterate and see if any binpart is reachable
			for (auto pose_it = (*binParts)[part_type].begin(); pose_it != (*binParts)[part_type].end(); ++pose_it)
			{
				// check if any part is reachable
				if (part->getAgvId() == "agv1")
				{
					if (pose_it->position.y <= 0)
					{
						part->setCurrentPose(*pose_it);
						return 0;
					}
				}
				else
				{
					if (pose_it->position.y >= 0)
					{
						part->setCurrentPose(*pose_it);
						return 0;
					}
				}
			}

			// if part is available in bin but not reachable, do this similar to pre-order setup A  NR
			geometry_msgs::Pose current_pose = (*binParts)[part_type].back();
			(*binParts)[part_type].pop_back();

			OrderPart* part_copy = new OrderPart();
			part_copy->setPartType(part->getPartType());
			part_copy->setShipmentId(part->getShipmentId());
			if (part->getAgvId() == "agv1")
			{
				part_copy->setAgvId("agv2");
				part_copy->setCurrentPose(current_pose);
				geometry_msgs::Pose end_pose = env_->getAvailableBinPosesObject()->getAvailableBinPoseArm1();
				part_copy->setEndPose(end_pose);
				part_copy->addPriority(-4);
				part->setCurrentPose(end_pose);
				part->addPriority(4);
			}
			else
			{
				part_copy->setAgvId("agv1");
				part_copy->setCurrentPose(current_pose);
				geometry_msgs::Pose end_pose = env_->getAvailableBinPosesObject()->getAvailableBinPoseArm2();
				part_copy->setEndPose(end_pose);
				part_copy->addPriority(-4);
				part->setCurrentPose(end_pose);
				part->addPriority(4);
			}
			// add them back to respective pqs
			(*(env_->getPriorityQueue()))[part->getAgvId()].push(part);
			(*(env_->getPriorityQueue()))[part_copy->getAgvId()].push(part_copy);
			return 1;  // pick later
		}
	}

	return 3;
}
