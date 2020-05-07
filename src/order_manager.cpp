/**
 * @file      src/order_manager.cpp
 * @brief     Source file for order manager
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

#include <environment.h>
#include <order_manager.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <boost/optional.hpp>
#include <set>
#include <algorithm>
OrderManager::OrderManager(Environment *env) : async_spinner(4), environment(env)
{
	async_spinner.start();
	order_ = NULL;
	order_subscriber_ = order_manager_nh_.subscribe("/ariac/orders", 10, &OrderManager::OrderCallback, this);
	ROS_INFO_STREAM("order manager is called. ");

	dynamic_planner = order_manager_nh_.advertise<std_msgs::Bool>("/ariac/dynamic_planner", 1000);
}

OrderManager::~OrderManager()
{
}

void OrderManager::OrderCallback(const osrf_gear::Order::ConstPtr &order_msg)
{
	ROS_WARN(">>>>> OrderCallback");
	environment->clearPriorityQueue();
	environment->clearANYvector();
	environment->clearShipmentVector();

	setOrderParts(order_msg);         // create new order and assign agv_id and shipment type to parts
	setArmForAnyParts();              // if there are "any" parts assign them to either arm1 or arm2
	comparewithTrayandUpdateOrder();  // check whether tray camera is called and segregate into trash parts and tray parts
	// and update arm*orderparts
	UpdateUnavailableParts();
	executeDynamicPlanner();
}

void OrderManager::setOrderParts(const osrf_gear::Order::ConstPtr &order_msg)
{
	ROS_INFO_STREAM("<<<<<Reading order>>>>>" << std::endl);
	auto order_id = order_msg->order_id;
	auto shipments = order_msg->shipments;
	auto pq = environment->getPriorityQueue();
	auto vector_ofshipments_withANY_tag = environment->getShipmentsOfAnyTagId();
	std::vector<std::vector<OrderPart*>>* shipment_vector_ = environment->getshipmentVector();
	std::vector<std::vector<OrderPart*>>::iterator shipment_it = shipment_vector_->begin();
	for (const auto &shipment : shipments){
		bool is_shipment_any =false;
		std::map<std::string, std::vector<OrderPart *>> shipment_Parts;
		auto shipment_type = shipment.shipment_type;
		auto agv_id = shipment.agv_id;
		auto products = shipment.products;
		std::vector<OrderPart*> current_shipment;
		current_shipment.clear();
		std::vector <OrderPart*> one_shipment_with_any;
		one_shipment_with_any.clear();
		for (const auto &product : products) {
			std::string part_type = product.type;
			OrderPart *order_part = new OrderPart(shipment_type, agv_id, part_type, product.pose);
			tf::Quaternion q(
					product.pose.orientation.x,
					product.pose.orientation.y,
					product.pose.orientation.z,
					product.pose.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			ROS_INFO_STREAM("roll is: " << roll);
			if(roll + 3.14159 <= 0.01 && part_type == "pulley_part"){
				order_part->setFlipPart(true);
			}
			ROS_INFO_STREAM(order_part->getPartType() << " needs to be flipped: " << order_part->isFlipRequired());
			current_shipment.push_back(order_part);
			if(order_part->getAgvId() != "any") {
				(*pq)[order_part->getAgvId()]->push(order_part);  // agv1 , agv2, any
			} else {
				one_shipment_with_any.push_back(order_part);
				is_shipment_any = true;
			}
			// ground truth shipments
			auto shipments = environment->getShipments();
			if(shipments->count(order_part->getShipmentId())) {
				if ((*shipments)[order_part->getShipmentId()].count(part_type)) {
					(*shipments)[order_part->getShipmentId()][part_type]++;
				} else {
					(*shipments)[order_part->getShipmentId()][part_type] = 0;
				}
			} else {
				(*shipments)[order_part->getShipmentId()][part_type] = 0;
			}
		}
		if(is_shipment_any) {
			vector_ofshipments_withANY_tag->push_back(one_shipment_with_any);

		}
		shipment_vector_->push_back(current_shipment);

	}

	ROS_INFO_STREAM("<<<<<Finished setting order parts>>>>>" );
}

// TODO check rbegin works properly
void OrderManager::setArmForAnyParts()
{
	ROS_INFO_STREAM("<<<<< Smart Decision for ANY tagged shipment>>>>>" << std::endl);

	ros::Duration(0.1).sleep();
	std::map<std::string, PriorityQueue*>* pq = environment->getPriorityQueue();
	PriorityQueue* pq_agv1 = (*pq)["agv1"];
	PriorityQueue* pq_agv2 = (*pq)["agv2"];

	std::vector<std::vector <OrderPart*>> * shipments_ANY = environment->getShipmentsOfAnyTagId();
	std::vector<std::vector<OrderPart*>>* shipment_vector_ = environment->getshipmentVector();

	if (shipments_ANY->size() == 0) {
		return;
	}

	std::array<std::map<std::string, int>, 2> arr = environment->getCountOfavailablePartsArmWise();
	ROS_WARN_STREAM("Check1");
	std::map<std::string, int> parttype_count_agv1 = arr[0];
	std::map<std::string, int> parttype_count_agv2 = arr[1];

	for (auto any_ship_it = shipments_ANY->begin(); any_ship_it != shipments_ANY->end(); ++any_ship_it) {
		int agv1_score = 0;
		int agv2_score = 0;
		for(auto it = shipment_vector_->begin(); it != shipment_vector_->end();++it) {
			if(it->back()->getAgvId() == "agv1"){
				agv1_score -= it->size();
			} else if (it->back()->getAgvId() == "agv2"){
				agv2_score -= it->size();
			}
		}
		ROS_WARN_STREAM("AGV Score : "  << agv1_score << " " << agv2_score);
		for(auto part_it = any_ship_it->begin(); part_it != any_ship_it->end(); ++part_it) {
			auto part_type = (*part_it)->getPartType();
			if (parttype_count_agv1.count(part_type)) {
				if (parttype_count_agv1[part_type] > 0) {
					++agv1_score;
					parttype_count_agv1[part_type] -= 1;
				}
			}
			if (parttype_count_agv2.count(part_type)) {
				if (parttype_count_agv2[part_type] > 0) {
					++agv2_score;
					parttype_count_agv2[part_type] -= 1;
				}
			}
		}
		ros::Duration(0.1).sleep();

		ROS_WARN_STREAM("AGV Score : "  << agv1_score << " " << agv2_score);
		if (agv1_score >= agv2_score) {
			for(auto part_it = any_ship_it->begin(); part_it != any_ship_it->end(); ++part_it) {
				(*part_it)->setAgvId("agv1");
				ros::Duration(0.01).sleep();
				(*part_it)->worldTransformation();
				ros::Duration(0.01).sleep();
				pq_agv1->push(*part_it);
				parttype_count_agv2[(*part_it)->getPartType()] += 1;
			}
		} else {
			for(auto part_it = any_ship_it->begin(); part_it != any_ship_it->end(); ++part_it) {
				(*part_it)->setAgvId("agv2");
				ros::Duration(0.01).sleep();
				(*part_it)->worldTransformation();
				ros::Duration(0.01).sleep();
				pq_agv2->push(*part_it);
				parttype_count_agv2[(*part_it)->getPartType()] += 1;
			}
		}
	}
	environment->clearANYvector();
	ROS_INFO_STREAM("<<<<<Finished allocating arm for any Parts>>>>>");
}

// once the part are categorized in agv1 and agv2 we need to compare with parts
// available in agv1 and agv2 resp and then remove the unnecessary part from the tray
// displace the parts in tray itself and add the new parts from bin.
void OrderManager::comparewithTrayandUpdateOrder()
{
	environment->ensureAllPartsinBothTraysareUpdated();
	auto r_tray1_parts = *(environment->getTray1Parts());
	auto r_tray2_parts = *(environment->getTray2Parts());

	comparewithTrayandUpdate("agv1", r_tray1_parts);
	comparewithTrayandUpdate("agv2", r_tray2_parts);
}

bool OrderManager::arePoseSame(geometry_msgs::Pose tray_pose, geometry_msgs::Pose order_pose) {
	double px = pow(tray_pose.position.x - order_pose.position.x, 2);
	double py = pow(tray_pose.position.y - order_pose.position.y, 2);
	double pz = pow(tray_pose.position.z - order_pose.position.z, 2);

	//	double ox = tray_pose.orientation.x - order_pose.orientation.x;
	//	double oy = tray_pose.orientation.y - order_pose.orientation.y;
	//	double oz = tray_pose.orientation.z - order_pose.orientation.z;
	//	double ow = tray_pose.orientation.w - order_pose.orientation.w;

	bool pose_same = true;  // TODO

	double dist = sqrt(px + py);

	if (dist < 0.1 and pose_same) {return true;}
	else {return false;}
}

void OrderManager::comparewithTrayandUpdate(std::string agv_id,
		std::map<std::string, std::vector<geometry_msgs::Pose>> r_tray_parts_) {
	std::map<std::string, PriorityQueue*>* pq = environment->getPriorityQueue();
	std::map<std::string, std::vector<geometry_msgs::Pose>> r_tray_parts = r_tray_parts_;


	if (!pq->count(agv_id))
	{
		return;
	}

	auto pq1 = pq->at(agv_id);

	std::vector<std::string> clear_map;
	for (auto pq1_it = pq1->getpq()->begin(); pq1_it != pq1->getpq()->end(); pq1_it++) {

		if ((*pq1_it)->getShipmentId() == 0) {
			if(r_tray_parts.count((*pq1_it)->getPartType())) {
				for (auto  part =  r_tray_parts.at((*pq1_it)->getPartType()).begin();
						part !=  r_tray_parts.at((*pq1_it)->getPartType()).end(); part++) {
					if(arePoseSame(*part, (*pq1_it)->getEndPose())) {
						if(r_tray_parts.at((*pq1_it)->getPartType()).size() > 0) {
							pq1->getpq()->erase(pq1_it--);
							r_tray_parts.at((*pq1_it)->getPartType()).erase(part--);
							if(r_tray_parts.at((*pq1_it)->getPartType()).size() == 0) {
								clear_map.push_back((*pq1_it)->getPartType());
							}
							break;
						}
					}
				}
			}
		}
	}

	for (auto&  it : clear_map) {
			r_tray_parts.erase(it);
		}


	for (const auto&  traypart_map : r_tray_parts) {
		ROS_INFO_STREAM("TEST2:" <<traypart_map.first << traypart_map.second.size() );
	}

	PriorityQueue* preorder;
	if(agv_id == "agv1") {
		preorder= environment->getPreOrderForArm1();
	} else if(agv_id == "agv2") {{
		preorder= environment->getPreOrderForArm2();
	}


	ROS_INFO_STREAM("One Stage Crossed");

	// displace parts update - setting priority and changhinf piuckup pose
	vector<std::string> clear_map2;
	for (auto pq1_it = pq1->getpq()->begin(); pq1_it != pq1->getpq()->end(); ++pq1_it) {
		//		ROS_INFO_STREAM("Loop 2 :" << count);

		if ((*pq1_it)->getShipmentId() == 0) {
			for (auto&  traypart_map : r_tray_parts) {
				if(traypart_map.first == (*pq1_it)->getPartType()) {
					if(traypart_map.second.size() > 0) {
						geometry_msgs::Pose t_part = traypart_map.second.back();
//						ROS_INFO_STREAM("There is "<< (*pq1_it)->getPartType()  << " part in tray with different Pose Start : "
//								<< t_part.position.x<< " , " << t_part.position.y <<
//								" End: "<< (*pq1_it)->getEndPose().position.x << " , " << (*pq1_it)->getEndPose().position.y);
						(*pq1_it)->addPriority(-7);
						(*pq1_it)->setCurrentPose(t_part);
						preorder->push((*pq1_it));
						pq1->getpq()->erase(pq1_it--);
						traypart_map.second.pop_back();

					} else {
						clear_map2.push_back(traypart_map.first);
					}
					break;
				}

			}
		}
	}

	for (auto&  it : clear_map2) {
		r_tray_parts.erase(it);
	}


	ROS_INFO_STREAM("Two Stage Crossed");

	for (const auto&  traypart_map : r_tray_parts) {
		ROS_INFO_STREAM("TEST3:" <<traypart_map.first << traypart_map.second.size() );
	}
	//		for (const auto&  traypart_map : r_tray_parts) {
	//			ROS_INFO_STREAM("TEST:" <<traypart_map.first << it.position.x << " , "<< it.position.y);
	//		}
	//	}
	// trash parts - remove from tray and put to trash bin // TODO check this part
	//	for (std::map<std::string, std::vector<geometry_msgs::Pose>>::const_iterator traypart_map =
	//			r_tray_parts.begin(); traypart_map != r_tray_parts.end(); traypart_map++ ) {
	for (const auto&  traypart_map : r_tray_parts) {
		std::string part_type = traypart_map.first;
		std::vector<geometry_msgs::Pose> part_vec = traypart_map.second;
		ROS_INFO_STREAM("Thrash Part " << part_type <<", Size : "<< part_vec.size());

		for(int i = 0; i < part_vec.size(); ++i) {
			OrderPart *trash_part = new OrderPart();
			ROS_WARN_STREAM("WARN Throw :" << part_type << part_vec[i].position.x << ", "<< part_vec[i].position.y);
			trash_part->setPriority(-10);
			trash_part->setPartType(part_type);
			trash_part->setCurrentPose(part_vec[i]);
			//			trash_part->setEndPose(environment->getTrashBinPose(agv_id));
			//			pq1->push(trash_part);
			trash_part->setTrashPart(true);
			preorder->push(trash_part);
		}
	}
	}
	ros::Duration(0.1).sleep();
	ROS_INFO_STREAM("Third Stage Crossed");
}


void OrderManager::UpdateUnavailableParts()
{
	ROS_INFO_STREAM("Stuck");
	std::map<std::string, std::vector<geometry_msgs::Pose>> *binParts = environment->getSortedBinParts();
	std::map<std::string, std::set<OrderPart *>> *unavailableParts = environment->getUnavailableParts();
	auto pq_map = environment->getPriorityQueue();
	for (auto pq_it = pq_map->begin(); pq_it != pq_map->end(); ++pq_it)
	{
		auto agv_id = pq_it->first;
		for (auto o_it = pq_it->second->getpq()->begin(); o_it != pq_it->second->getpq()->end(); ++o_it)
		{
			if (!binParts->count((*o_it)->getPartType()))
			{
				// we are not changing to lowerst prioty here..maybe in dymic planner
				(*unavailableParts)[agv_id].insert((*o_it));
			}
		}
	}
	for (auto pq_it = pq_map->begin(); pq_it != pq_map->end(); ++pq_it) {
		pq_it->second->printPq();
	}
	//	ROS_INFO_STREAM("Stuck2");

}

void OrderManager::executeDynamicPlanner() {
	ros::Duration(0.1).sleep();
	std_msgs::Bool msg;
	msg.data = true;
	for(size_t i=0; i<1; ++i){
		dynamic_planner.publish(msg);
	}
	ROS_INFO_STREAM("!!! Order Manager has completed it's processing !!!");

}
