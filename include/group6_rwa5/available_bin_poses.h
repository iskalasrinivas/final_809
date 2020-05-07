/**
 * @file      include/available_bin_poses.h
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

#ifndef GROUP6_RWA5_AVAILABLEBINPOSE_H
#define GROUP6_RWA5_AVAILABLEBINPOSE_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <transformer.h>

class AvailableBinPoses {
  public:
  // AvailableBinPoses();
    explicit AvailableBinPoses(std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> *);
    ~AvailableBinPoses();

    void addToAvailableBinPoses(std::string, geometry_msgs::Pose);        // add a pose to a camera in arm1 reach i.e. 3 bins
//    void addToAvailableBinPosesArm2(std::string, geometry_msgs::Pose);        // add a pose to a camera in arm2 reach i.e. 3 bins

//    void clearBinFromArm1(std::string);
//    void clearBinFromArm2(std::string);
    void takesCareofAllCamera();
    bool isInProximity(geometry_msgs::Pose , geometry_msgs::Pose );

    geometry_msgs::Pose getAvailableBinPoseArm1(); // returns a pose available in one of the 3bins for arm2
    geometry_msgs::Pose getAvailableBinPoseArm2(); // returns a pose available in one of the 3bins for arm1
    // geometry_msgs::Pose getStaticBinPoseInWorld(std::string, geometry_msgs::Pose, geometry_msgs::Pose );

  private:
    bool emptyplace_ensured;
    Transformer transform_;
    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* all_bin_parts;
    std::vector<geometry_msgs::Pose> static_poses_;
//    std::map<std::string, std::vector<int>> available_ind_arm1_; // keep a track of the pose number
//    std::map<std::string, std::vector<int>> available_ind_arm2_; // keep a track of the pose number
    std::vector<geometry_msgs::Pose> available_poses_arm1_; // keep a track of the pose number
    std::vector<geometry_msgs::Pose> available_poses_arm2_; // keep a track of the pose number
    std::vector<geometry_msgs::Pose> temp_poses_arm1_; // keep a track of the pose number
    std::vector<geometry_msgs::Pose> temp_poses_arm2_;
};

#endif //GROUP6_RWA5_AVAILABLEBINPOSE_H
