/**
 * @file      include/logical_camera_sensor.h
 * @brief     Header file for Sensor
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

#ifndef GROUP6_RWA5_TRANSFORMER_H_
#define GROUP6_RWA5_TRANSFORMER_H_

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <osrf_gear/LogicalCameraImage.h>


class Transformer {

    public:
        Transformer();
        Transformer(const std::string &);
        ~Transformer();
        // std::string getCameraName();
        // void setWorldTransform();
        // void setParentPose(const geometry_msgs::Pose &sensor_pose);
        // void setChildPose(const geometry_msgs::Pose &child_pose);
        // void setPose(const geometry_msgs::Pose, geometry_msgs::TransformStamped &);
        // void setPose(const geometry_msgs::TransformStamped &, geometry_msgs::Pose &);
        // geometry_msgs::Pose getChildWorldPose();
        // void fromCameraName(std::string camera_name);

        geometry_msgs::Pose getChildPose(geometry_msgs::Pose);
        geometry_msgs::Pose getChildPose(std::string , geometry_msgs::Pose );

            private : std::string topic_;
        // std::string parent_;
        // std::string child_;
        std::string frame_;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
        geometry_msgs::TransformStamped t_camera_world;
        geometry_msgs::Pose pose_wrt_world;

        // geometry_msgs::TransformStamped transformStamped1;
        // geometry_msgs::TransformStamped transformStamped2;
        // geometry_msgs::TransformStamped transformStamped3;
        
        // tf2_ros::TransformBroadcaster br_w_s;
        // tf2_ros::TransformBroadcaster br_s_c;

};


#endif // GROUP6_RWA5_TRANSFORMER_H