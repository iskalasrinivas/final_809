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
	for (const auto &shipment : shipments)
	{
		std::map<std::string, std::vector<OrderPart *>> shipment_Parts;
		auto shipment_type = shipment.shipment_type;
		auto agv_id = shipment.agv_id;
		auto products = shipment.products;
		for (const auto &product : products)
		{
			std::string part_type = product.type;
			OrderPart *order_part = new OrderPart(shipment_type, agv_id, part_type, product.pose);
			(*pq)[order_part->getAgvId()].push(order_part);  // agv1 , agv2, any
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
	}

	ROS_INFO_STREAM("<<<<<Finished setting order parts>>>>>");
}

// TODO check rbegin works properly
void OrderManager::setArmForAnyParts()
{
	ROS_INFO_STREAM("<<<<< Smart Decision for ANY tagged shipment>>>>>" << std::endl);
	int agv1_score = 0;
	int agv2_score = 0;

	auto pq = environment->getPriorityQueue();

	if (!pq->count("any")) {
		return;
	}

	PriorityQueue &pq_any = pq->at("any");

	std::array<std::map<std::string, int>, 2> arr = environment->getCountOfavailablePartsArmWise();
	std::map<std::string, int> parttype_count_agv1 = arr[0];
	std::map<std::string, int> parttype_count_agv2 = arr[1];

	int previousShipmentID = (*(pq_any.getpq()->rbegin()))->getShipmentId();

	auto start_it = pq_any.getpq()->rbegin();
	// 222222 111111 000000
	for (auto pq_any_it = pq_any.getpq()->rbegin(); pq_any_it != pq_any.getpq()->rend(); ++pq_any_it) {
		if ((*pq_any_it)->getShipmentId() == previousShipmentID) {
			auto part_type = (*pq_any_it)->getPartType();

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

		if ((*pq_any_it)->getShipmentId() != previousShipmentID or pq_any_it == pq_any.getpq()->rend() - 1) {
			if (agv1_score >= agv2_score) {
				while (start_it != pq_any_it) {
					(*start_it)->setAgvId("agv1");
					(*start_it)->worldTransformation();
					(*pq)["agv1"].push(*start_it);
					start_it++;
					parttype_count_agv2[(*start_it)->getPartType()] += 1;
				}
			} else {
				while (start_it != pq_any_it) {
					(*start_it)->setAgvId("agv2");
					(*start_it)->worldTransformation();
					(*pq)["agv2"].push(*start_it);
					start_it++;
					parttype_count_agv1[(*start_it)->getPartType()] += 1;
				}
			}

			agv1_score = 0;
			agv2_score = 0;
			previousShipmentID = (*pq_any_it)->getShipmentId();
		}
	}
	// clear any since it is dealt with
	pq_any.clear();
	ROS_INFO_STREAM("<<<<<Finished allocating arm for any Parts>>>>>");
}

// once the part are categorized in agv1 and agv2 we need to compare with parts
// available in agv1 and agv2 resp and then remove the unnecessary part from the tray
// displace the parts in tray itself and add the new parts from bin.
void OrderManager::comparewithTrayandUpdateOrder()
{
	auto r_tray1_parts = *(environment->getTray1Parts());
	auto r_tray2_parts = *(environment->getTray1Parts());

	comparewithTrayandUpdate("agv1", r_tray1_parts);
	comparewithTrayandUpdate("agv2", r_tray2_parts);
}

void OrderManager::comparewithTrayandUpdate(std::string agv_id,
		std::map<std::string, std::vector<geometry_msgs::Pose>> &r_tray_parts)
{
	auto pq = environment->getPriorityQueue();

	if (!pq->count(agv_id))
	{
		return;
	}

	auto pq1 = pq->at(agv_id);

	// removing parts that are opn tray and of required pose
	for (auto traypart_map : r_tray_parts)
	{  // for part on tray

		auto part_type = traypart_map.first;
		auto part_vec = traypart_map.second;

		for (auto pq1_it = pq1.getpq()->begin(); pq1_it != pq1.getpq()->end(); ++pq1_it)
		{
			if ((*pq1_it)->getShipmentId() == 0) // only for first shipment
			{  // under assumption that first shipment has shipment id == 0
				for (auto part_it = part_vec.begin(); part_it != part_vec.end(); ++part_it)
				{
					if ((*pq1_it)->getPartType() == part_type and (*pq1_it)->getEndPose() == (*part_it))
					{
						ROS_INFO_STREAM("There are parts in Tray" << agv_id << "similar to current Order");
						pq1.getpq()->erase(pq1_it);
						part_vec.erase(part_it);
					}
				}
			}
		}
	}

	// displace parts update - setting priority and changhinf piuckup pose
	for (auto traypart_map : r_tray_parts)
	{
		auto part_type = traypart_map.first;
		auto part_vec = traypart_map.second;

		for (auto pq1_it = pq1.getpq()->begin(); pq1_it != pq1.getpq()->end(); ++pq1_it)
		{
			if ((*pq1_it)->getShipmentId() == 0) // only for first shipment
			{  // under assumption that first shipment has shipment id == 0
				for (auto part_it = part_vec.begin(); part_it != part_vec.end(); ++part_it)
				{
					if ((*pq1_it)->getPartType() == part_type)
					{
						(*pq1_it)->addPriority(-2);
						(*pq1_it)->setCurrentPose((*part_it));
						part_vec.erase(part_it);
					}
				}
			}
		}
	}

	// trash parts - remove from tray and put to trash bin
	for (auto traypart_map : r_tray_parts)
	{
		auto part_type = traypart_map.first;
		auto part_vec = traypart_map.second;

		for (auto part_it = part_vec.begin(); part_it != part_vec.end(); ++part_it)
		{
			OrderPart *trash_part = new OrderPart();
			ROS_DEBUG_STREAM("WARN");
			trash_part->setPriority(-3);
			trash_part->setPartType(part_type);
			trash_part->setCurrentPose(*part_it);
			trash_part->setEndPose(environment->getTrashBinPose());
			pq1.push(trash_part);
		}
	}
}

void OrderManager::UpdateUnavailableParts()
{
	std::map<std::string, std::vector<geometry_msgs::Pose>> *binParts = environment->getSortedBinParts();
	std::map<std::string, std::set<OrderPart *>> *unavailableParts = environment->getUnavailableParts();
	auto pq_map = environment->getPriorityQueue();
	for (auto pq_it = pq_map->begin(); pq_it != pq_map->end(); ++pq_it)
	{
		auto agv_id = pq_it->first;
		for (auto o_it = pq_it->second.getpq()->begin(); o_it != pq_it->second.getpq()->end(); ++o_it)
		{
			if (!binParts->count((*o_it)->getPartType()))
			{
				(*unavailableParts)[agv_id].insert((*o_it));
			}
		}
	}
}

void OrderManager::executeDynamicPlanner() {

	std_msgs::Bool msg;
	msg.data = true;
	for(size_t i=0; i<1; ++i){
		dynamic_planner.publish(msg);
	}
	ROS_INFO_STREAM("!!! Order Manager has completed it's processing !!!");

}
