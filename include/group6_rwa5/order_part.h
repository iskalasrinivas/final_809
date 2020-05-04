/**
 * @file      include/order_path.h
 * @brief     Header file for Order part
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
#ifndef GROUP6_RWA5_ORDER_PART_H_
#define GROUP6_RWA5_ORDER_PART_H_

#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class OrderPart
{

private:
 std::string part_type_;
 std::string shipment_type;
 std::string agv_id;
 bool in_transit_;
 bool tobepickedbyotherhand_;
 bool trashpart_;
 int ship_id;
 geometry_msgs::Pose tray_pose_;
 geometry_msgs::Pose end_pose_;
 geometry_msgs::Pose current_pose_;
 geometry_msgs::Pose middle_pose_;
 
 int priority;

 static int count;
 static tf2_ros::Buffer tfBuffer;

 tf2_ros::TransformListener tfListener;
 geometry_msgs::TransformStamped tS_w_b;
 bool flip_part;
 bool static_part;

public:
 OrderPart();
 OrderPart(std::string, std::string, std::string, geometry_msgs::Pose);
 ~OrderPart();

 // Setters
 void setPartType(std::string);
 void setCurrentPose(geometry_msgs::Pose);
 void setEndPose(geometry_msgs::Pose);
 void setFlipPart(const bool &);
 void setAgvId(std::string);
 void setShipmentType(std::string);
 void setShipmentId();
 void setShipmentId(int);
 void setLowestPriority();
 void setHighestPriority();
 void setPriority(int);
 void setStatic(bool);
 void addPriority(int);
 void setInTransit(bool);
 void setPickedbyOtherHand(bool);
 // Getters
 bool getInTransit();
 bool hasToBePickedbyOtherHand();
 std::string getPartType() const;
 geometry_msgs::Pose getEndPose() const;
 geometry_msgs::Pose getTrayPose() const;
 geometry_msgs::Pose getMiddlePose() const;
 geometry_msgs::Pose getCurrentPose() const;
 void worldTransformation();
 bool isFlipRequired() const;
 std::string getShipmentType() const;
 int getShipmentId() const;
 int getPriority();
 bool getStatic() const;
 std::string getAgvId();

 bool isOfHighestPriority();
 bool isOfLowestPriority();

 bool isTrashPart();
 void setTrashPart(bool);

};

#endif // GROUP6_RWA5_ORDER_PART_H_
