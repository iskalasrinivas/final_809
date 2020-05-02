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

DynamicPlanner::DynamicPlanner(Environment* env) : async_spinner(0), env_(env), exe_(env) {
	// ros::AsyncSpinner async_spinner(0);
	async_spinner.start();

	dplanner_sub_ = dplanner_nh_.subscribe<std_msgs::Bool>("/ariac/dynamic_planner", 10,
			&DynamicPlanner::dynamicPlannerCallBack, this);
}

DynamicPlanner::~DynamicPlanner(){}

void DynamicPlanner::dynamicPlannerCallBack(const std_msgs::Bool::ConstPtr& msg) {
	ROS_WARN_STREAM("<<<<<In Dynamic Planner>>>>>");
	if (msg->data) {
		current_shipment_it =  env_->getshipmentVector()->begin();
		arm1_thread = std::thread(&DynamicPlanner::dynamicPlanningforArm1, this);
		arm2_thread = std::thread(&DynamicPlanner::dynamicPlanningforArm2, this);

		arm1_thread.join();
		arm2_thread.join();
	}
}

void DynamicPlanner::updatePickPoseFromBin(OrderPart* order_) {

	env_->ensureAllPartsinAllBinsareUpdated();
	auto part_type = order_->getPartType();


	auto allbinpart = env_->getSortedBinParts();  // std::map<std::string, std::vector<geometry_msgs::Pose> >*
	if (allbinpart->count(part_type)) {
		auto new_pose = (*allbinpart)[part_type].back();
		order_->setCurrentPose(new_pose);
	}
}

bool DynamicPlanner::isPoseSame(geometry_msgs::Pose tray_pose, geometry_msgs::Pose order_pose) {
	double px = pow(tray_pose.position.x - order_pose.position.x, 2);
	double py = pow(tray_pose.position.y - order_pose.position.y, 2);
	double pz = pow(tray_pose.position.z - order_pose.position.z, 2);

	double ox = tray_pose.orientation.x - order_pose.orientation.x;
	double oy = tray_pose.orientation.y - order_pose.orientation.y;
	double oz = tray_pose.orientation.z - order_pose.orientation.z;
	double ow = tray_pose.orientation.w - order_pose.orientation.w;

	bool pose_same = true;  // TODO

	double dist = sqrt(px + py + pz);

	if (dist < 0.01 and pose_same) {return true;}
	else {return false;}
}

//@saurav: To be reviewed from here

//bool DynamicPlanner::isShipmentofTrayChecked(std::string agv_id) {
//	std::map<std::string, std::vector<geometry_msgs::Pose>>* tray1_parts = env_->getTray1Parts();
//
//	auto shipment = (*(env_->getCompletedShipment()))[agv_id];
//
//	int shipment_id = shipment.front()->getShipmentId();
//
//	auto ground_truth_shipment = (*(env_->getShipments()))[shipment_id];
//
//	for (auto traypart_map : *(tray1_parts)) {
//		auto part_type = traypart_map.first;
//		if(traypart_map.second.size() != ground_truth_shipment[part_type]) { return false;}
//	}
//	return true;
//}

bool DynamicPlanner::isShipmentComplete(std::string agv_id, std::vector<std::vector<OrderPart*>>::iterator shipment_it) {

	std::map<std::string, std::vector<geometry_msgs::Pose>>* tray_parts;

	if(agv_id == "agv1") {
		tray_parts = env_->getTray1Parts();
	} else if(agv_id == "agv2") {
		tray_parts = env_->getTray2Parts();
	}

	std::map<std::string, int> tray_size;
	for (auto t_it = tray_parts->begin(); t_it != tray_parts->end(); ++t_it){
		tray_size[t_it->first] = t_it->second.size();
	}

	int parts_intray = 0;

	for(auto map_it = tray_size.begin(); map_it != tray_size.end(); ++map_it ) {
		parts_intray +=  (*map_it).second;
	}

	if(shipment_it->size() !=  parts_intray) { return false; }
	if(shipment_it->size() ==  parts_intray) {
		for(auto o_it = shipment_it->begin(); o_it != shipment_it->end(); ++o_it){
			if(!tray_parts->count((*o_it)->getPartType())) {
				return false;
			} else {
				tray_size[(*o_it)->getPartType()] -= 1;
			}
		}
	}
	for(auto map_it = tray_size.begin(); map_it != tray_size.end(); ++map_it ){
		if(map_it->second != 0) {return false ;}
	}
	return true;
}

void DynamicPlanner::removeUnwantedPartfromTray(std::string agv_id, std::vector<std::vector<OrderPart*>>::iterator shipment_it) {

	std::map<std::string, std::vector<geometry_msgs::Pose>>* tray_parts;

	if(agv_id == "agv1") {
		tray_parts = env_->getTray1Parts();
	} else if(agv_id == "agv2") {
		tray_parts = env_->getTray2Parts();
	}

	std::map<std::string, int> tray_size;
	for (auto t_it = tray_parts->begin(); t_it != tray_parts->end(); ++t_it){
		tray_size[t_it->first] = t_it->second.size();
	}

	int parts_intray = 0;

	for(auto map_it = tray_size.begin(); map_it != tray_size.end(); ++map_it ) {
		parts_intray +=  (*map_it).second;
	}

	for(auto o_it = shipment_it->begin(); o_it != shipment_it->end(); ++o_it){
		if(!tray_parts->count((*o_it)->getPartType())) {
			tray_size[(*o_it)->getPartType()] -= 1;
		}
	}
	auto arm_pq = (*(env_->getPriorityQueue()))[agv_id];
	for(auto map_it = tray_size.begin(); map_it != tray_size.end(); ++map_it ){
		if(map_it->second > 0) {
			// TODO multiple parts not considered
			OrderPart *trash_part = new OrderPart();
			ROS_DEBUG_STREAM("WARN");
			trash_part->setHighestPriority();
			trash_part->setPartType(map_it->first);
			geometry_msgs::Pose pose_ = (*tray_parts)[map_it->first].back();
			trash_part->setCurrentPose(pose_);
			trash_part->setEndPose(env_->getTrashBinPose());
			env_->getPriorityQueue()->at(agv_id)->push(trash_part);
		}
	}
}



// function to process orderparts from priortity queue  for arm1
// make sure this runs only after planner is completed
// ie. when new order comes, we are clearing the priortiy queue. So we will have to wait till we re-populate it.
// planner does that. so wait!!
void DynamicPlanner::dynamicPlanningforArm1() {
	auto arm1_pq = (*(env_->getPriorityQueue()))["agv1"];

	if(arm1_pq->empty()) {
		ROS_INFO_STREAM("Arm1 not in action");
		return;
	}
	auto arm1_ = exe_.getArm1Object();
	//	int previousShipmentId = arm1_pq->top()->getShipmentId();
	std::vector<std::vector<OrderPart*>>::iterator shipmnet_it = env_->getshipmentVector()->begin();
	while(shipmnet_it->front()->getAgvId() != "agv1") {
		++shipmnet_it;
	}
	int previousShipmentId = shipmnet_it->front()->getShipmentId();
	while (!arm1_pq->empty()) {
		//		ROS_INFO_STREAM("Arm1 : " << arm1_pq->top()->getPartType() <<" "<< arm1_pq->top()->getPriority() << ",  Priority Queue size : "<<arm1_pq->getpq()->size());
		auto order_part = arm1_pq->top();
		ros::Duration(0.01).sleep();

		arm1_pq->pop();

		// check pick up location for part
		int retVal = updatePickupLocation("arm1" ,order_part);
		ROS_INFO_STREAM("Arm 1 : PQ size : "<< arm1_pq->size() << " Part Type : "<<
				order_part->getPartType()<<" , PQ :" << order_part->getPriority()<<" ,  A,R(0), A,NR(1), NA(2), B(3) : " << retVal);
		bool isSameAsPreviousShipment = previousShipmentId==order_part->getShipmentId();
		bool delivered = false;

		if (retVal == 0) {
			delivered = completeSinglePartOrder(arm1_, order_part, isSameAsPreviousShipment);
		}
		else if (retVal == 1) {
			continue;
		}
		else if (retVal == 2) {
			continue;
		}
		else if (retVal == 3) {
			arm1_->GoToBeltHome();
			ROS_INFO_STREAM("ARM1 : Went to Belt home");

			//			ROS_INFO_STREAM("DP ARM1 : Wait" << env_->getPickupLocations()->count("agv1") );
			while(!env_->getPickupLocations()->count("agv1")) {}
			//			ROS_WARN_STREAM("ARM1 : Waiting " << order_part->getPartType());
			std::map<std::string, geometry_msgs::Pose*>* armpickuplocation = &((*env_->getPickupLocations())["agv1"]);
			while(!armpickuplocation->count(order_part->getPartType())) {}
			//			ROS_WARN_STREAM("ARM1 : About To act");
			while(armpickuplocation->at(order_part->getPartType()) == nullptr) {
				ros::Duration(0.001).sleep();
			}

			ROS_INFO_STREAM("DP ARM1: Action");
			geometry_msgs::Pose* arm1_pck_lctn = armpickuplocation->at(order_part->getPartType());

			bool picked = arm1_->pickPartFromBelt(arm1_pck_lctn);
			if (!picked){
				ROS_INFO_STREAM("ARM1 : not Picked from belt Pushed to PQ1: " << arm1_pq->size() );
				order_part->setLowestPriority();
				arm1_pq->push(order_part);
				arm1_->SendRobotHome();
				continue;
			}
			delivered = completeSinglePartOrder(arm1_,order_part, isSameAsPreviousShipment);
		}

		if (!delivered) {
			if(retVal==0 and order_part->getInTransit()) {
				order_part->setInTransit(false);
			}
			ROS_INFO_STREAM("Pushing in Arm 1 PQ If Not delivered , PQ1 size " << arm1_pq->size());
			arm1_pq->push(order_part);
		}
		else {
			if(order_part->getAgvId() == "agv1"){
				(*(env_->getCompletedShipment()))[order_part->getAgvId()].push_back(order_part);
			}

			if(retVal==3) {
				// remove from unavailable parts as it bis processed
				if(env_->getUnavailableParts()->count("agv1")){// remove from unavailable part map
					if(env_->getUnavailableParts()->at("agv1").count(order_part)){
						env_->getUnavailableParts()->at("agv1").erase(order_part);
					}
				}
				// remove from pickuplocations map
				if(env_->getPickupLocations()->count(order_part->getAgvId())) {
					if(env_->getPickupLocations()->at(order_part->getAgvId()).count(order_part->getPartType())) {
						env_->getPickupLocations()->at(order_part->getAgvId()).erase(order_part->getPartType());
					}
				}
			}
		}

		if(isShipmentComplete("agv1", shipmnet_it)) {
			while (current_shipment_it != shipmnet_it) {
				ros::Duration(2.0).sleep();
			}

			// TODO Send AGV
			// Wait for AGV
			exe_.SendAGV1();
			ros::Duration(5.0).sleep();
			++current_shipment_it;

		} else{
			//			removeUnwantedPartfromTray("agv1", shipmnet_it);
		}
	}
}

// function to process orderparts from priortity queue  for arm2
// make sure this runs only after planner is completed
// ie. when new order comes, we are clearing the priortiy queue. So we will have to wait till we re-populate it.
// planner does that. so wait!!
void DynamicPlanner::dynamicPlanningforArm2() {

	auto arm2_pq = (*(env_->getPriorityQueue()))["agv2"];
	if(arm2_pq->empty()) {
		return;
	}
	auto arm2_ = exe_.getArm2Object();
	std::vector<std::vector<OrderPart*>>::iterator shipmnet_it = env_->getshipmentVector()->begin();
	while(shipmnet_it->front()->getAgvId() != "agv2") {
		++shipmnet_it;
	}
	int previousShipmentId = shipmnet_it->front()->getShipmentId();
	while (!arm2_pq->empty()) {
		ros::Duration(0.01).sleep();
		//		ROS_WARN_STREAM("Arm2 : " << arm2_pq->top()->getPartType() <<": "<< arm2_pq->top()->getPriority() << ", Priority Queue size : "<<arm2_pq->getpq()->size());
		auto order_part = arm2_pq->top();
		arm2_pq->pop();
		bool isSameAsPreviousShipment = previousShipmentId==order_part->getShipmentId();
		bool delivered = false;
		int test = 0;
		// check pick up location for part
		int retVal = updatePickupLocation("arm2", order_part);
		//		ROS_WARN_STREAM("Arm 2 A,R(0), A,NR(1), NA(2), B(3) : " << retVal);
		ROS_WARN_STREAM("Arm 2 : PQ size : "<< arm2_pq->getpq()->size()<< " Part Type : "<<
				order_part->getPartType()<<" , PQ :" << order_part->getPriority()<<" ,  A,R(0), A,NR(1), NA(2), B(3) : " << retVal);
		if (retVal == 0) {
			delivered = completeSinglePartOrder(arm2_, order_part, isSameAsPreviousShipment);

		}
		else if (retVal == 1) {
			test = 1;
			continue; }
		else if (retVal == 2) { continue; }
		else if (retVal == 3) {
			arm2_->GoToBeltHome();
			ROS_WARN_STREAM("ARM2 : Went to Belt home");

			while(!env_->getPickupLocations()->count("agv2")) {}
			ROS_WARN_STREAM("ARM2 : Waiting");
			std::map<std::string, geometry_msgs::Pose*>* armpickuplocation = &((*env_->getPickupLocations())["agv2"]);
			while(!armpickuplocation->count(order_part->getPartType())) {}
			ROS_WARN_STREAM("ARM2 : About To act");
			while(armpickuplocation->at(order_part->getPartType()) == nullptr) {
				ros::Duration(0.01).sleep();
			}
			ROS_INFO_STREAM("DP ARM2 : Action");
			geometry_msgs::Pose* arm2_pck_lctn = armpickuplocation->at(order_part->getPartType());

			//			ROS_WARN_STREAM("DP Arm 2 Value of pose is in DP =>" << *arm2_pck_lctn);
			bool picked = arm2_->pickPartFromBelt(arm2_pck_lctn);
			if (!picked){
				ROS_INFO_STREAM("ARM2 : not Picked from belt Pushed to PQ2: " << arm2_pq->size() );
				order_part->setLowestPriority();
				arm2_pq->push(order_part);
				arm2_->SendRobotHome();
				continue;
			}

			delivered = completeSinglePartOrder(arm2_,order_part, isSameAsPreviousShipment);

		}

		if (!delivered)	{
			if(retVal==0 and order_part->getInTransit()) {
				order_part->setInTransit(false);
			}
			ROS_INFO_STREAM("ARM2 : not delivered from belt Pushed to PQ1: " << arm2_pq->size() );
			arm2_pq->push(order_part);
		} else {
			// do this so that if part is for this agv
			if(order_part->getAgvId() == "agv2"){
				(*(env_->getCompletedShipment()))[order_part->getAgvId()].push_back(order_part);
			}

			if(retVal==3) {
				// remove from unavailable parts as it bis processed
				if(env_->getUnavailableParts()->count("agv1")){// remove from unavailable part map
					if(env_->getUnavailableParts()->at("agv1").count(order_part)){
						env_->getUnavailableParts()->at("agv1").erase(order_part);
					}
				}
				// remove from pickuplocations map
				if(env_->getPickupLocations()->count(order_part->getAgvId())) {
					if(env_->getPickupLocations()->at(order_part->getAgvId()).count(order_part->getPartType())) {
						env_->getPickupLocations()->at(order_part->getAgvId()).erase(order_part->getPartType());
					}
				}
			}
		}
		if(isShipmentComplete("agv2", shipmnet_it)) {
			while (current_shipment_it != shipmnet_it) {
				ros::Duration(2.0).sleep();
			}

			// TODO Send AGV
			// Wait for AGV
			exe_.SendAGV2();
			ros::Duration(5.0).sleep();
			++current_shipment_it;

		} else{
			//			removeUnwantedPartfromTray("agv2", shipmnet_it); //TODO
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
int DynamicPlanner::updatePickupLocation(std::string arm, OrderPart* part) {
	// given order part

	// if avaiable and reachable  update pick location
	// if only available but not reachable : reduce priority in your que and push it to the other queue are with higher
	// prioiry if not available  : reduce the priority to minimum  in your queue : push it to list of unavailable parts

	env_->ensureAllPartsinAllBinsareUpdated();
	std::map<std::string, std::vector<geometry_msgs::Pose>>* binParts = env_->getSortedBinParts();
//	for (auto it = binParts->begin(); it != binParts->end();++it) {
//		ROS_WARN_STREAM("Part Type : "<< it->first<< " : " <<it->second.size());
//	}
//	ros::Duration(1.0).sleep();
	if (!part->isOfHighestPriority()) {  // if highest priority belt trigger will change it
		std::string part_type = part->getPartType();

		// first check if part_type is in any of the bins (NA NR)
		if (!binParts->count(part_type)) {
			if(not part->getInTransit()){
				part->setLowestPriority();           // set lowest priority :INT_MAX
				part->setStatic(false);              // make it non-static for as part of conveyor part
				env_->pushToUnavailableParts(part);  // set
				(*(env_->getPriorityQueue()))[part->getAgvId()]->push(part); //  because we are popping in arm1dyunamic planner
				return 2;
			}
			return 1;
		} else {
			// iterate and see if any bin part is reachable
			for (auto pose_it = (*binParts)[part_type].begin(); pose_it != (*binParts)[part_type].end(); ++pose_it) {
				// check if any part is reachable
//				ROS_INFO_STREAM("I am here");
				if (part->getAgvId() == "agv1" or (part->getAgvId() == "agv2" and part->hasToBePickedbyOtherHand())) {
					if (pose_it->position.y >= 0) {
						part->setCurrentPose(*pose_it);
//						ros::Duration(0.01).sleep();
//						ROS_INFO_STREAM("Pick Up Location" <<pose_it->position.x <<", "<<pose_it->position.y<<", "<<pose_it->position.z);
						(*binParts)[part_type].erase(pose_it);
						return 0;
					}
				} else {
					if (pose_it->position.y <= 0 or (part->getAgvId() == "agv1" and part->hasToBePickedbyOtherHand())) {
						part->setCurrentPose(*pose_it);
//						ros::Duration(0.01).sleep();
//						ROS_INFO_STREAM("Pick Up Location" <<pose_it->position.x <<", "<<pose_it->position.y<<", "<<pose_it->position.z);
						(*binParts)[part_type].erase(pose_it);
						return 0;
					}
				}
			}
			OrderPart* part_copy;
			if(!part->getInTransit()) {
				// if part is available in bin but not reachable, do this similar to pre-order setup A  NR
				geometry_msgs::Pose current_pose = (*binParts)[part_type].back();
				(*binParts)[part_type].pop_back();

				part_copy = new OrderPart();
//				ros::Duration(0.1).sleep();
				part_copy->setPartType(part->getPartType());
				part_copy->setShipmentId(part->getShipmentId());
				if (arm == "arm1" and part->getAgvId() == "agv1") {
					part_copy->setAgvId("agv1");
//					part_copy->setCurrentPose(current_pose);
					geometry_msgs::Pose end_pose = env_->getAvailableBinPosesObject()->getAvailableBinPoseArm2();
					part_copy->setEndPose(end_pose);
					part_copy->setShipmentId(part->getShipmentId());
					part_copy->setPriority(-4);
					part_copy->setPickedbyOtherHand(true);
					part->setCurrentPose(end_pose);
					part->setInTransit(true);
					part->addPriority(4);

					env_->getPriorityQueue()->at(part->getAgvId())->push(part);
					env_->getPriorityQueue()->at("agv2")->push(part_copy);
//					(*(env_->getPriorityQueue()))[part->getAgvId()]->push(part);
					(*(env_->getPriorityQueue()))["agv2"]->push(part_copy);
//					ROS_ERROR_STREAM("1Pushing 1 in agv1 :" << (*(env_->getPriorityQueue()))[part->getAgvId()]->size()
//							<<" and 1 in agv2 : " <<(*(env_->getPriorityQueue()))["agv2"]->size()
//							<<" for not reachable part for arm 1 "<< part->getPartType());
				} else if (arm == "arm2" and part->getAgvId() == "agv2"){
					part_copy->setAgvId("agv2");
//					part_copy->setCurrentPose(current_pose);
					geometry_msgs::Pose end_pose = env_->getAvailableBinPosesObject()->getAvailableBinPoseArm1();
					part_copy->setEndPose(end_pose);
					part_copy->setShipmentId(part->getShipmentId());
					part_copy->setPriority(-4);
					part_copy->setPickedbyOtherHand(true);
					part->setCurrentPose(end_pose);
					part->addPriority(4);
					part->setInTransit(true);
					env_->getPriorityQueue()->at(part->getAgvId())->push(part);
					env_->getPriorityQueue()->at("agv1")->push(part_copy);
//					(*(env_->getPriorityQueue()))[part->getAgvId()]->push(part);
//					(*(env_->getPriorityQueue()))["agv1"]->push(part_copy);
//					ROS_ERROR_STREAM("2Pushing 1 in agv1 : " << (*(env_->getPriorityQueue()))["agv1"]->size()
//							<<" and 1 in agv2 : "<< (*(env_->getPriorityQueue()))[part->getAgvId()]->size()
//							<<" for not reachable part for arm 2 "<< part->getPartType());
				}
			} else {
				part->addPriority(1);

//				ROS_ERROR_STREAM("Duplicate Part Priority :" << part_copy->getPartType()<< " "<<part_copy->getPriority());
				part_copy->addPriority(-4);
				(*(env_->getPriorityQueue()))[part->getAgvId()]->push(part);
			}
			// add them back to respective pqs

			return 1;  // pick later
		}
	}

	return 3;
}


bool DynamicPlanner::completeSinglePartOrder(RobotController* arm, OrderPart *order, bool isSameAsPreviousShipment) {
	if(isSameAsPreviousShipment) {
		bool flag = false;
		if(!arm->isPartAttached()) {
			auto curr_pose = order->getCurrentPose();
			arm->pickPartFromBin(curr_pose);
		}
		if(order->isFlipRequired()) {
			// Drop part in left face Orientation // TODO check this function
			// Pick from right side // TODO check this function
			// Change Orientation to Down Side // TODO check this function
			ROS_INFO_STREAM("Fliping part ...");
			arm->flipPart(order);
			ROS_INFO_STREAM("part is flipped going to quality.");
		}
		if(order->getAgvId() ==  "agv1" and arm->getArmName() != "arm1") {
			arm->dropPart(order->getEndPose()); // TODO Perfect This function
			return true;

		}
		if(order->getAgvId() ==  "agv2" and arm->getArmName() != "arm2") {
			arm->dropPart(order->getEndPose()); // TODO Perfect This function
			return true;
		}

		arm->GoToQualityCamera(); // TODO Perfect This function

		arm->dropPart(order->getEndPose());

		env_->setQualityCameraRequired(order->getAgvId(), true); // TODO implement inverse method of continuos input

		while(!env_->isQualityCameraCalled(order->getAgvId())) {
			ros::Duration(0.5).sleep();
		}
		env_->setQualityCameraRequired(order->getAgvId(), false);
		ROS_INFO_STREAM("going into check if part faulty");
		ROS_INFO_STREAM("is part faulty: " << env_->isPartFaulty(order->getAgvId()));
		if (env_->isPartFaulty(order->getAgvId())) {
			arm->pickPartFromAGV(order->getEndPose());
			arm->dropInTrash(); // TODO check this function
			flag = false;
		} else {
			flag = true;
		}
		// logic to return bool upon failure or success
		return flag;
	} else {
		if(order->hasToBePickedbyOtherHand()) {
			arm->pickPartFromBin(order->getCurrentPose());
			arm->dropPart(order->getEndPose());
			return true;
		} else {
			if(arm->getArmName()=="arm1"){
				order->setEndPose(env_->getAvailableBinPosesObject()->getAvailableBinPoseArm1());
			} else {
			order->setEndPose(env_->getAvailableBinPosesObject()->getAvailableBinPoseArm2());
			}
			arm->dropPart(order->getEndPose()); // TODO Perfect This function
		}
		return false;
	}
}
