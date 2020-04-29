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
	//	env_->setBinCameraRequired(true);

	env_->ensureAllPartsinAllBinsareUpdated();
	auto part_type = order_->getPartType();


	auto allbinpart = env_->getSortedBinParts();  // std::map<std::string, std::vector<geometry_msgs::Pose> >*
	if (allbinpart->count(part_type))
	{
		auto new_pose = (*allbinpart)[part_type].back();
		order_->setCurrentPose(new_pose);
		//			env_->setBinCameraRequired(false);
		//			env_->setAllBinCameraCalled(false);
	}
}

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

	auto arm1_pq = (*(env_->getPriorityQueue()))["agv1"];
	auto arm1_ = exe_.getArm1Object();
	int previousShipmentId = arm1_pq->top()->getShipmentId();
	while (!arm1_pq->empty())
	{	

		//		arm1_pq.printPq();
		ROS_INFO_STREAM("Arm1 : " << arm1_pq->top()->getPartType() << "  Priority Queue size : "<<arm1_pq->getpq()->size());
		auto order_part = arm1_pq->top();
		ros::Duration(0.1).sleep();

		arm1_pq->pop();


		// check pick up location for part
		int retVal = updatePickupLocation(order_part);

		bool delivered = false;

		if (retVal == 0)
		{  // pick up from bin
			//			ROS_INFO_STREAM("Arm1 : " << order_part->getPartType() <<"removing from PQ1");

			delivered = completeSinglePartOrder(arm1_, order_part);
			//			delivered =true;
		}
		else if (retVal == 1)
		{  // no action
			ROS_INFO_STREAM("Arm1 : " << order_part->getPartType() <<"removing from PQ1 and adding to PQ2");
			continue;
		}
		else if (retVal == 2)
		{  // no action
			continue;
		}
		else if (retVal == 3)
		{  // action for belt
			arm1_->GoToBeltHome();
			ROS_WARN_STREAM("Went to Belt home");
//			ROS_WARN_STREAM("Part type is " << order_part->getPartType());
			// TODO set the sequence of action here it self using pointer since the location is updating continuously
			//			while ((*(env_->getPickupLocations())).count("agv1") and (*(env_->getPickupLocations()))["agv1"].count(order_part->getPartType())) {
			//				ROS_WARN_STREAM("This part exists in get pickup locations()");

			ROS_INFO_STREAM("DP ARM1 : Wait" << env_->getPickupLocations()->count("agv1") );
			while(!env_->getPickupLocations()->count("agv1")) {
//				ros::Duration(0.01).sleep();
						}
			ROS_INFO_STREAM("DP ARM1: Waiting" << env_->getPickupLocations()->at("agv1").count(order_part->getPartType()));
			while(!env_->getPickupLocations()->at("agv1").count(order_part->getPartType())) {
							//		ROS_WARN_STREAM("Waiting for part come under belt camera for pickup...");
//				ros::Duration(0.01).sleep();
						}
//
//			ROS_INFO_STREAM("DP : agv1" << " "<< order_part->getPartType());
			//				geometry_msgs::Pose* arm1_pck_lctn = (*(env_->getPickupLocations()))["agv1"][order_part->getPartType()];
			while(env_->getPickupLocations()->at("agv1").at(order_part->getPartType()) == nullptr) {
				//		ROS_WARN_STREAM("Waiting for part come under belt camera for pickup...");
//				ros::Duration(0.01).sleep();
			}
			ROS_INFO_STREAM("DP ARM1: Action");
			geometry_msgs::Pose* arm1_pck_lctn = env_->getPickupLocations()->at("agv1").at(order_part->getPartType());
//			ROS_WARN_STREAM("DP : 222222222");
			ROS_WARN_STREAM("Value of pose is in DP =>" << *arm1_pck_lctn);
			arm1_->pickPartFromBelt(arm1_pck_lctn);
			delivered = completeSinglePartOrder(arm1_,order_part);
			//				delivered =true;

		}

		if (!delivered)
		{
			arm1_pq->push(order_part);
		}
		else
		{
			(*(env_->getCompletedShipment()))[order_part->getAgvId()].push_back(order_part);
		}
	}
}

// function to process orderparts from priortity queue  for arm2
void DynamicPlanner::dynamicPlanningforArm2()
{
	// make sure this runs only after planner is completed
	// ie. when new order comes, we are clearing the priortiy queue. So we will have to wait till we re-populate it.
	// planner does that. so wait!!
	auto arm2_pq = (*(env_->getPriorityQueue()))["agv2"];
	auto arm2_ = exe_.getArm2Object();
	int previousShipmentId = arm2_pq->top()->getShipmentId();

	while (!arm2_pq->empty())
	{
		//		arm2_pq.printPq();
		ROS_INFO_STREAM("Arm2 : " << arm2_pq->top()->getPartType() << "  Priority Queue size : "<<arm2_pq->getpq()->size());
		auto order_part = arm2_pq->top();

		arm2_pq->pop();

		bool delivered = false;

		// check pick up location for part
		int retVal = updatePickupLocation(order_part);

		if (retVal == 0)
		{ // pick up from bin
			delivered = completeSinglePartOrder(arm2_, order_part);
			ROS_INFO_STREAM("Arm2 : " << order_part->getPartType() <<"removing from PQ2");
			//			delivered =true;
		}
		else if (retVal == 1)
		{ // no action
			ROS_INFO_STREAM("Arm2 : " << order_part->getPartType() <<"removing from PQ2 and adding to PQ1");
			continue;
		}
		else if (retVal == 2)
		{ // no action
			continue;
		}
		else if (retVal == 3)
		{ // action for belt
			arm2_->GoToBeltHome();
			ROS_WARN_STREAM("Went to Belt home");
//			ROS_WARN_STREAM("Part type is " << order_part->getPartType());
			ROS_INFO_STREAM("DP ARM2 : Wait" << env_->getPickupLocations()->count("agv2") );
			while(!env_->getPickupLocations()->count("agv2")) {
	//				ros::Duration(0.01).sleep();
						}
			ROS_INFO_STREAM("DP ARM2: Waiting " << env_->getPickupLocations()->at("agv2").count(order_part->getPartType()));
			while(!env_->getPickupLocations()->at("agv2").count(order_part->getPartType())) {
							//		ROS_WARN_STREAM("Waiting for part come under belt camera for pickup...");
	//				ros::Duration(0.01).sleep();
						}

			while(env_->getPickupLocations()->at("agv2").at(order_part->getPartType()) == nullptr) {
				//		ROS_WARN_STREAM("Waiting for part come under belt camera for pickup...");
//				ros::Duration(0.01).sleep();
			}
			ROS_INFO_STREAM("DP ARM2 : Action");
			geometry_msgs::Pose* arm2_pck_lctn = env_->getPickupLocations()->at("agv2").at(order_part->getPartType());
//			ROS_WARN_STREAM("DP : 222222222");
			ROS_WARN_STREAM("Value of pose is in DP =>" << *arm2_pck_lctn);
			arm2_->pickPartFromBelt(arm2_pck_lctn);
			delivered = completeSinglePartOrder(arm2_,order_part);
			//				delivered =true;

					}


		if (!delivered)
		{
			arm2_pq->push(order_part);
		}
		else
		{
			(*(env_->getCompletedShipment()))[order_part->getAgvId()].push_back(order_part);
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
	//
	//	ROS_INFO_STREAM("<<<Updating Pick up Location>>>");
	//	ROS_INFO_STREAM("<<<Reducing priority of the Conveyer parts with unknown Pick up Location");

	if (!part->isOfHighestPriority())
	{  // if highest priority belt trigger will change it
		std::string part_type = part->getPartType();

		// first check if part_type is in any of the bins (NA NR)
		if (!binParts->count(part_type))
		{
			//			ROS_INFO_STREAM("Part is not available in any bins....so pushing back to prioroty queue with lowest prioiry!!!");
			part->setLowestPriority();           // set lowest priority :INT_MAX
			part->setStatic(false);              // make it non-static for as part of conveyor part
			env_->pushToUnavailableParts(part);  // set
			(*(env_->getPriorityQueue()))[part->getAgvId()]->push(part);
			return 2;
		}
		else
		{
			// iterate and see if any bin part is reachable
			for (auto pose_it = (*binParts)[part_type].begin(); pose_it != (*binParts)[part_type].end(); ++pose_it)
			{
				// check if any part is reachable
				if (part->getAgvId() == "agv1")
				{
					if (pose_it->position.y >= 0)
					{
						part->setCurrentPose(*pose_it);
						ROS_INFO_STREAM("Pick Up Location" <<pose_it->position.x <<", "<<pose_it->position.y<<", "<<pose_it->position.z);
						return 0;
					}
				}
				else
				{
					if (pose_it->position.y <= 0)
					{
						part->setCurrentPose(*pose_it);
						ROS_INFO_STREAM("Pick Up Location" <<pose_it->position.x <<", "<<pose_it->position.y<<", "<<pose_it->position.z);
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
			(*(env_->getPriorityQueue()))[part->getAgvId()]->push(part);
			(*(env_->getPriorityQueue()))[part_copy->getAgvId()]->push(part_copy);
			return 1;  // pick later
		}
	}

	return 3;
}


bool DynamicPlanner::completeSinglePartOrder(RobotController* arm, OrderPart *order) {
	bool flag = false;
	if(!arm->isPartAttached()){
		auto curr_pose = order->getCurrentPose();
		arm->pickPartFromBin(curr_pose);
	}
	if(order->isFlipRequired()) {
		// Drop part in left face Orientation // TODO check this function
		// Pick from right side // TODO check this function
		// Change Orientation to Down Side // TODO check this function
	}
	arm->GoToQualityCamera(); // TODO Perfect This function
	arm->dropPart(order->getEndPose()); // TODO Perfect This function

	env_->setQualityCameraRequired(order->getAgvId(), true); // TODO implement inverse method of continuos input

	while(!env_->isQualityCameraCalled(order->getAgvId())) {
		ros::Duration(0.02).sleep();
	}
	env_->setQualityCameraRequired(order->getAgvId(), false);
	if (env_->isPartFaulty(order->getAgvId()))
	{
		arm->pickPartFromAGV(order->getEndPose());
		arm->dropInTrash(); // TODO check this function
		flag = false;
	}
	else
	{
		flag = true;
	}
	// logic to return bool upon failure or success
	return flag;
}
