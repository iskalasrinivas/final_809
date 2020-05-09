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
#include <priority_queue.h>
#include <vector>
#include <string>
#include <thread>


DynamicPlanner::DynamicPlanner(Environment* env) : async_spinner(0), env_(env), exe_(env) {
	// ros::AsyncSpinner async_spinner(0);
	async_spinner.start();
	env_->setTrashBinPose(exe_.getArm1Object()->getTrashBinPose(),exe_.getArm2Object()->getTrashBinPose());
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
	env_->ensureAllPartsinBothTraysareUpdated();
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
			trash_part->setEndPose(env_->getTrashBinPose(agv_id));
			env_->getPriorityQueue()->at(agv_id)->push(trash_part);
		}
	}
}


void DynamicPlanner::clearTray(RobotController* arm , std::string agv_id){
	ROS_INFO_STREAM("PreOrder Started");
	PriorityQueue* preorder;
	if(agv_id == "agv1") {
		preorder= env_->getPreOrderForArm1();
	} else if (agv_id == "agv2"){
		preorder= env_->getPreOrderForArm2();
	}
	ROS_INFO_STREAM("PreOrder Order Size : " << preorder->size() );
	while (!preorder->empty()) {

		auto order_part = preorder->top();
		ros::Duration(0.01).sleep();
		preorder->pop();
		arm->GoToQualityCamera();
		arm->pickPartFromAGV(order_part->getCurrentPose());
		if (order_part->isTrashPart()) {
			arm->dropInTrash();
		} else {
			arm->dropPart(order_part->getEndPose());
		}
	}
	ROS_INFO_STREAM("PreOrder Completed");
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
		//		ROS_INFO_STREAM("Pre-Order Processing");
		if(env_->getPreOrderForArm1()->size() != 0) {
			clearTray(arm1_, "agv1");
		}
		auto order_part = arm1_pq->top();
		ros::Duration(0.01).sleep();

		arm1_pq->pop();
		ROS_INFO_STREAM("Pre-Order Processing");
		ROS_INFO_STREAM("Arm1 : " << order_part->getPartType());
		// check pick up location for part


		int retVal = updatePickupLocation("arm1" ,order_part);
		ROS_INFO_STREAM("Arm 1 : PQ size : "<< arm1_pq->size()+1 << " Part Type : "<<
				order_part->getPartType()<<" , PQ :" << order_part->getPriority()<<" ,  A,R(0), A,NR(1), NA(2), B(3) : " << retVal);
		bool isSameAsPreviousShipment = previousShipmentId==order_part->getShipmentId();
		bool delivered = false;
		ROS_INFO_STREAM("Before complete Single ");
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
				std::map<std::string, geometry_msgs::Pose*>* armpickuplocation = &((*env_->getPickupLocations())["agv1"]);
				armpickuplocation->erase(order_part->getPartType());
				arm1_->SendRobotHome();
				continue;
			}
			delivered = completeSinglePartOrder(arm1_,order_part, isSameAsPreviousShipment);
		}
		ROS_INFO_STREAM("After complete Single ");
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


		ROS_INFO_STREAM(arm1_->getArmName() << " : Shipment not completed");

		if(isShipmentComplete("agv1", shipmnet_it)) {
			ROS_INFO_STREAM(arm1_->getArmName() << " : Shipment completed");
			checkBeforeDispatch(arm1_, shipmnet_it, "agv1");
			arm1_->SendRobotHome();
			ROS_INFO_STREAM("Send AGV");
			while (current_shipment_it != shipmnet_it) {
				ROS_INFO_STREAM("Wait");
				ros::Duration(2.0).sleep();

			}

			// TODO Send AGV
			// Wait for AGV

			ROS_INFO_STREAM("Sending AGV 1");
			exe_.SendAGV1();
			ros::Duration(5.0).sleep();
			++current_shipment_it;

		} else{
			//			removeUnwantedPartfromTray("agv1", shipmnet_it);
		}
		ROS_INFO_STREAM("Next Loop");
	}
	arm1_->SendRobotHome();
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
	ROS_INFO_STREAM("Pre-Order Processing");


	std::vector<std::vector<OrderPart*>>::iterator shipmnet_it = env_->getshipmentVector()->begin();
	while(shipmnet_it->front()->getAgvId() != "agv2") {
		++shipmnet_it;
	}
	int previousShipmentId = shipmnet_it->front()->getShipmentId();
	while (!arm2_pq->empty()) {
		ros::Duration(3.0).sleep();
		//		ROS_WARN_STREAM("Arm2 : " << arm2_pq->top()->getPartType() <<": "<< arm2_pq->top()->getPriority() << ", Priority Queue size : "<<arm2_pq->getpq()->size());
		if(env_->getPreOrderForArm2()->size() != 0) {
			clearTray(arm2_, "agv2");
		}
		auto order_part = arm2_pq->top();
		arm2_pq->pop();
		bool isSameAsPreviousShipment = previousShipmentId==order_part->getShipmentId();
		bool delivered = false;
		ROS_INFO_STREAM("Arm2 : " << order_part->getPartType());
		// check pick up location for part
		int retVal = updatePickupLocation("arm2", order_part);
		//		ROS_WARN_STREAM("Arm 2 A,R(0), A,NR(1), NA(2), B(3) : " << retVal);
		ROS_WARN_STREAM("Arm 2 : PQ size : "<< arm2_pq->getpq()->size()+1 << " Part Type : "<<
				order_part->getPartType()<<" , PQ :" << order_part->getPriority()<<" ,  A,R(0), A,NR(1), NA(2), B(3) : " << retVal);
		if (retVal == 0) {
			delivered = completeSinglePartOrder(arm2_, order_part, isSameAsPreviousShipment);

		}
		else if (retVal == 1) {
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
				std::map<std::string, geometry_msgs::Pose*>* armpickuplocation = &((*env_->getPickupLocations())["agv2"]);
				armpickuplocation->erase(order_part->getPartType());
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
		ROS_INFO_STREAM(arm2_->getArmName() << " : Shipment not completed !!");
		if(isShipmentComplete("agv2", shipmnet_it)) {
			ROS_INFO_STREAM(arm2_->getArmName() << " : Shipment completed !!");
			checkBeforeDispatch(arm2_,shipmnet_it, "agv2");
			arm2_->SendRobotHome();
			while (current_shipment_it != shipmnet_it) {
				ros::Duration(2.0).sleep();
			}

			// TODO Send AGV
			// Wait for AGV
			ROS_INFO_STREAM("Sending AGV 2");
			exe_.SendAGV2();
			ros::Duration(5.0).sleep();
			++current_shipment_it;

		} else{
			//			removeUnwantedPartfromTray("agv2", shipmnet_it); //TODO
		}
		ROS_INFO_STREAM("Next Loop Arm 2");
	}
	arm2_->SendRobotHome();
}

void DynamicPlanner::checkBeforeDispatch(RobotController* arm_, std::vector<std::vector<OrderPart*>>::iterator current_shipment_it, std::string agv_id){
	ROS_INFO_STREAM("Making the Order correct for " << agv_id);
	env_->ensureAllPartsinBothTraysareUpdated();
	std::map<std::string, std::vector<geometry_msgs::Pose>> tray_parts;

	if (agv_id == "agv1") {
		tray_parts = *(env_->getTray1Parts());
	}
	if(agv_id == "agv2") {
		tray_parts = *(env_->getTray2Parts());
	}

	for(auto map_it = tray_parts.begin(); map_it != tray_parts.end(); ++map_it) {
		for(auto part = map_it->second.begin(); part != map_it->second.end(); ++part) {
			ROS_INFO_STREAM(" Tray Part : "<< map_it->first<< " : " << (*part).position.x << " , "<<(*part).position.y);
		}
	}
	std::vector<OrderPart*> current_shipment = *current_shipment_it;

	for(auto part_it = current_shipment.begin(); part_it != current_shipment.end();++part_it) {
		ROS_INFO_STREAM(" Order Part : "<< (*part_it)->getPartType()<< " : " << (*part_it)->getEndPose().position.x << " , "<<(*part_it)->getEndPose().position.y);
	}

	std::vector<std::string> clear_map;
	//	std::vector < std::vector<OrderPart*>::iterator > current_shipment_remove;
	//	std::map<std::string, std::vector<geometry_msgs::Pose>>
	for(std::vector<OrderPart*>::iterator part_it = current_shipment.begin(); part_it != current_shipment.end();++part_it) {
		if(tray_parts.count((*part_it)->getPartType())) {
			auto part_type = (*part_it)->getPartType();
			for (auto tray_part =  tray_parts.at(part_type).begin(); tray_part !=  tray_parts.at(part_type).end(); tray_part++) {
				ROS_INFO_STREAM(" 0: 0");
				if(arePoseSame(*tray_part, (*part_it)->getEndPose())) {
					ROS_INFO_STREAM(" 0: 1");
					if(tray_parts.at(part_type).size() >= 1) {
						ROS_INFO_STREAM("0: 2");
						current_shipment.erase(part_it--);

						if(tray_parts.at(part_type).size() > 1) {
							tray_parts.at(part_type).erase(tray_part--);
						} else if(tray_parts.at(part_type).size() == 1) {
							clear_map.push_back(part_type);
						}
						break;
					}
				}
			}
		}
	}
	for (auto&  it : clear_map) {
		tray_parts.erase(it);
	}
	PriorityQueue delivery;
	std::vector<std::string> clear_map2;
	//	for(auto part_it = current_shipment.begin(); part_it != current_shipment.end();++part_it) {
	//		for (auto&  traypart_map : tray_parts) {
	//			if(traypart_map.first == (*part_it)->getPartType()) {
	//				if(traypart_map.second.size() > 0) {
	//					geometry_msgs::Pose t_part = traypart_map.second.back();
	//					(*part_it)->setPriority(-7);
	//					(*part_it)->setCurrentPose(t_part);
	//					delivery.push((*part_it));
	//					current_shipment.erase(part_it--);
	//					traypart_map.second.pop_back();
	//
	//				} else {
	//					clear_map2.push_back(traypart_map.first);
	//				}
	//			}
	//			break;
	//		}
	//	}

	for(auto map_it = tray_parts.begin(); map_it != tray_parts.end(); ++map_it) {
		for(auto part = map_it->second.begin(); part != map_it->second.end(); ++part) {
			ROS_INFO_STREAM(" Tray Part : "<< map_it->first<< " : " << (*part).position.x << " , "<<(*part).position.y);
		}
	}

	for(auto part_it = current_shipment.begin(); part_it != current_shipment.end();++part_it) {
		ROS_INFO_STREAM(" Order Part : "<< (*part_it)->getPartType()<< " : " << (*part_it)->getEndPose().position.x << " , "<<(*part_it)->getEndPose().position.y);
	}

	std::map<std::string, std::vector<OrderPart*>>order_map;
	for(auto part_it = current_shipment.begin(); part_it != current_shipment.end();++part_it) {
		order_map[(*part_it)->getPartType()].push_back((*part_it));
	}

	for(auto part_type_it = order_map.begin(); part_type_it != order_map.end();++part_type_it) {
		auto part_type = part_type_it->first;
		auto tray_it = tray_parts.at(part_type).begin();
		for(auto order_it = part_type_it->second.begin();
				(order_it != part_type_it->second.end()) or ( tray_it !=  tray_parts.at(part_type).end());++tray_it,++order_it) {
			(*order_it)->setCurrentPose(*tray_it);
			(*order_it)->setPriority(-7);
			delivery.push((*order_it));
			ros::Duration(0.1).sleep();
		}
	}


	//	for(auto part_it = current_shipment.begin(); part_it != current_shipment.end();++part_it) {
	//		//		bool should_break = false;
	//		ros::Duration(0.1).sleep();
	//		ROS_INFO_STREAM("0");
	//
	//		auto traypart_map = tray_parts.at((*part_it)->getPartType());
	//		ROS_INFO_STREAM("Tray Parts : " <<(*part_it)->getPartType()<<" , "<<traypart_map.size());
	//		if(traypart_map.size() > 0) {
	//			ROS_INFO_STREAM("Displace Order Part : "<< (*part_it)->getPartType()<< " : " << (*part_it)->getEndPose().position.x << " , "<<(*part_it)->getEndPose().position.y);
	//			geometry_msgs::Pose t_part = traypart_map.back();
	//			ros::Duration(0.1).sleep();
	//			ROS_INFO_STREAM("1");
	//			(*part_it)->setPriority(-7);
	//			(*part_it)->setCurrentPose(t_part);
	//			delivery.push((*part_it));
	//			current_shipment.erase(part_it--);
	//			ros::Duration(0.1).sleep();
	//			ROS_INFO_STREAM("2");
	//			traypart_map.pop_back();
	//			ros::Duration(0.1).sleep();
	//			ROS_INFO_STREAM("3");
	//			if(tray_parts.count((*part_it)->getPartType())) {
	//				ROS_INFO_STREAM("4");
	//				if(tray_parts.at((*part_it)->getPartType()).size() == 0) {
	//					ROS_INFO_STREAM("5");
	//					clear_map2.push_back((*part_it)->getPartType());
	//					ROS_INFO_STREAM("5:1");
	//					//				should_break = true;
	//				}
	//			}
	//		}
	//	}
//	ROS_INFO_STREAM("6");
//	ros::Duration(0.01).sleep();
//	for (auto&  it : clear_map2) {
//		tray_parts.erase(it);
//	}

	delivery.printPq();

	while(!delivery.empty()) {
		auto order_part = delivery.top();
		ros::Duration(0.01).sleep();

		delivery.pop();
		arm_->pickPartFromAGV(order_part->getCurrentPose());
		arm_->dropPart(order_part->getEndPose());
	}


}


bool DynamicPlanner::arePoseSame(geometry_msgs::Pose tray_pose, geometry_msgs::Pose order_pose) {
	double px = pow(tray_pose.position.x - order_pose.position.x, 2);
	double py = pow(tray_pose.position.y - order_pose.position.y, 2);
	double pz = pow(tray_pose.position.z - order_pose.position.z, 2);

	//	double ox = tray_pose.orientation.x - order_pose.orientation.x;
	//	double oy = tray_pose.orientation.y - order_pose.orientation.y;
	//	double oz = tray_pose.orientation.z - order_pose.orientation.z;
	//	double ow = tray_pose.orientation.w - order_pose.orientation.w;

	bool pose_same = true;  // TODO

	double dist = sqrt(px + py);
	ROS_INFO_STREAM(dist);
	if (dist < 0.1 and pose_same) {return true;}
	else {return false;}
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
				//								ROS_INFO_STREAM("I am here");
				if (part->getAgvId() == "agv1" or (part->getAgvId() == "agv2" and part->hasToBePickedbyOtherHand())) {
					if (pose_it->position.y >= -1.525*0) {
						part->setCurrentPose(*pose_it);
						//						ros::Duration(0.01).sleep();
						//						ROS_INFO_STREAM("Pick Up Location" <<pose_it->position.x <<", "<<pose_it->position.y<<", "<<pose_it->position.z);
						(*binParts)[part_type].erase(pose_it--);
						return 0;
					}
				}
				if (part->getAgvId() == "agv2" or (part->getAgvId() == "agv1" and part->hasToBePickedbyOtherHand())) {
					if (pose_it->position.y <= 1.525*0 ) {
						part->setCurrentPose(*pose_it);
						//						ros::Duration(0.01).sleep();
						//						ROS_INFO_STREAM("Pick Up Location" <<pose_it->position.x <<", "<<pose_it->position.y<<", "<<pose_it->position.z);
						(*binParts)[part_type].erase(pose_it--);
						return 0;
					}
				}
			}

			if(!part->getInTransit()) {
				// if part is available in bin but not reachable, do this similar to pre-order setup A  NR
				geometry_msgs::Pose current_pose = (*binParts)[part_type].back();
				(*binParts)[part_type].pop_back();

				OrderPart* part_copy = new OrderPart();
				//				ros::Duration(0.1).sleep();
				part_copy->setPartType(part->getPartType());
				part_copy->setShipmentId(part->getShipmentId());
				if (arm == "arm1" and part->getAgvId() == "agv1") {
					part_copy->setAgvId("agv1");
					//					part_copy->setCurrentPose(current_pose);
					ROS_INFO_STREAM("Setting End Pose for unavailable part for arm 1 ");
					//					env_->ensureAllPartsinAllBinsareUpdated();
					geometry_msgs::Pose end_pose = env_->getAvailableBinPosesObject()->getAvailableBinPoseArm1();
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
					//					(*(env_->getPriorityQueue()))["agv2"]->push(part_copy);
					//					ROS_ERROR_STREAM("1Pushing 1 in agv1 :" << (*(env_->getPriorityQueue()))[part->getAgvId()]->size()
					//							<<" and 1 in agv2 : " <<(*(env_->getPriorityQueue()))["agv2"]->size()
					//							<<" for not reachable part for arm 1 "<< part->getPartType());
				} else if (arm == "arm2" and part->getAgvId() == "agv2"){
					part_copy->setAgvId("agv2");
					//					part_copy->setCurrentPose(current_pose);
					ROS_INFO_STREAM("Setting End Pose for unavailable part for arm 2 ");
					//					env_->ensureAllPartsinAllBinsareUpdated();
					geometry_msgs::Pose end_pose = env_->getAvailableBinPosesObject()->getAvailableBinPoseArm2();
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

				ROS_ERROR_STREAM(arm <<" Part not available yet " << part->getPartType());
				//				part->addPriority(-4);
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
			arm->SendRobotHome();
			ROS_INFO_STREAM("Picking part ...");
			auto curr_pose = order->getCurrentPose();

			arm->pickPartFromBin(curr_pose);
			//			arm->SendRobotHome();
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
		ros::Duration(0.3).sleep();
		if(!arm->isPartAttached()) {
			ROS_INFO_STREAM("Part fell off from arm");
			ros::Duration(2.0).sleep();

			std::map<std::string, std::vector<geometry_msgs::Pose>> tray_parts;
			env_->ensureAllPartsinBothTraysareUpdated();
			if (arm->getArmName()== "arm1" and order->getAgvId() == "agv1") {
				tray_parts = *(env_->getTray1Parts());
			} else if(arm->getArmName()== "arm2" and order->getAgvId() == "agv2") {
				tray_parts = *(env_->getTray2Parts());
			}
			if(tray_parts.count(order->getPartType())) {
				if(tray_parts.at(order->getPartType()).size() == 1) {
					order->setCurrentPose(tray_parts.at(order->getPartType()).front());
					arm->pickPartFromAGV(order->getCurrentPose());
					//				arm->dropPart(order->getEndPose());
				}
			} else {
				ROS_INFO_STREAM("Part Not Found in Tray !!");
			}
		}
		arm->dropPartinAGV(order->getEndPose());

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
		//		arm->SendRobotHome();
		// logic to return bool upon failure or success
		return flag;
	} else {
		if(order->hasToBePickedbyOtherHand()) {
			arm->GoToOtherSideliveryPose();
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
