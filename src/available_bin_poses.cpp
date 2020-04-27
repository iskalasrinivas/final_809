#include <available_bin_poses.h>
#include <cmath>
#include <algorithm>

AvailableBinPoses::AvailableBinPoses(std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* abp): all_bin_parts(abp){
    
    for (size_t i = 0; i < 4; ++i)
    {
        static_poses_.emplace_back();
    }

    static_poses_.at(0).position.x = 0.36722;
    static_poses_.at(0).position.y = 0.1512;
    static_poses_.at(0).position.z = -0.12;
    static_poses_.at(0).orientation.x = 0.0150;
    static_poses_.at(0).orientation.y = -0.523;
    static_poses_.at(0).orientation.z = 0.013;
    static_poses_.at(0).orientation.w = 0.8522;

    static_poses_.at(1).position.x = 0.36722;
    static_poses_.at(1).position.y = -0.1486;
    static_poses_.at(1).position.z = -0.12;
    static_poses_.at(1).orientation.x = 0.0150;
    static_poses_.at(1).orientation.y = -0.523;
    static_poses_.at(1).orientation.z = 0.013;
    static_poses_.at(1).orientation.w = 0.8522;

    static_poses_.at(2).position.x = 0.4893;
    static_poses_.at(2).position.y = 0.1512;
    static_poses_.at(2).position.z = -0.12;
    static_poses_.at(2).orientation.x = 0.0150;
    static_poses_.at(2).orientation.y = -0.523;
    static_poses_.at(2).orientation.z = 0.013;
    static_poses_.at(2).orientation.w = 0.8522;

    static_poses_.at(3).position.x = 0.4893;
    static_poses_.at(3).position.y = -0.1486;
    static_poses_.at(3).position.z = -0.12;
    static_poses_.at(3).orientation.x = 0.0150;
    static_poses_.at(3).orientation.y = -0.523;
    static_poses_.at(3).orientation.z = 0.013;
    static_poses_.at(3).orientation.w = 0.8522;

}


AvailableBinPoses::~AvailableBinPoses()
{}

geometry_msgs::Pose AvailableBinPoses::getStaticBinPoseInWorld(std::string cam_name, geometry_msgs::Pose cam_pose, geometry_msgs::Pose child_pose)
{
    transform_.fromCameraName(cam_name);
    transform_.setChildPose(child_pose);
    transform_.setParentPose(cam_pose);
    transform_.setWorldTransform();
    geometry_msgs::Pose world_pose = transform_.getChildWorldPose();
    return world_pose;
}

bool AvailableBinPoses::isInProximity(geometry_msgs::Pose bin_pose, geometry_msgs::Pose static_pose)
{

    double x = pow(bin_pose.position.x - static_pose.position.x, 2);
    double y = pow(bin_pose.position.y - static_pose.position.y, 2);

    double dist = sqrt(x + y);

    if (dist < 0.25)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void AvailableBinPoses::addToAvailableBinPosesArm1(std::string cam_name, geometry_msgs::Pose cam_pose)
{
    // auto all_bin_parts = env->getAllBinParts; //std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> *getAllBinParts();
    auto parts_in_bin = (*all_bin_parts)[cam_name];

    for (auto cam_elements_it = parts_in_bin.begin(); cam_elements_it != parts_in_bin.end(); ++cam_elements_it)
    {
        for (auto pose_it = cam_elements_it->second.begin(); pose_it != cam_elements_it->second.end(); ++pose_it)
        {
            for (int i = 0; i < static_poses_.size(); ++i)
            {
                auto static_world_pose = getStaticBinPoseInWorld(cam_name, cam_pose, static_poses_.at(i));
                if (!isInProximity(*pose_it, static_world_pose))
                {
                    if (available_ind_arm1_.count(cam_name) and available_poses_arm1_.count(cam_name))
                    {
                        // check for duplicacy
                        if(std::find(available_ind_arm1_[cam_name].begin(), available_ind_arm1_[cam_name].end(),i)==available_ind_arm1_[cam_name].end()) {
                            available_ind_arm1_[cam_name].push_back(i);
                            available_poses_arm1_[cam_name].push_back(static_world_pose);
                        }
                    }
                    else
                    {
                        available_ind_arm1_[cam_name] = std::vector<int>{i};
                        available_poses_arm1_[cam_name] = std::vector<geometry_msgs::Pose>{static_world_pose};
                    }
                }
                else
                {
                    if (available_ind_arm1_.count(cam_name) and available_poses_arm1_.count(cam_name))
                    {
                        auto it = std::find(available_ind_arm1_[cam_name].begin(), available_ind_arm1_[cam_name].end(), i);
                        if (it != available_ind_arm1_[cam_name].end()) {
                            available_ind_arm1_[cam_name].erase(it);
                            auto ind = std::distance(available_ind_arm1_[cam_name].begin(), it);
                            auto jt = available_poses_arm1_[cam_name].begin() + ind;
                            available_poses_arm1_[cam_name].erase(jt);
                            if (available_poses_arm1_[cam_name].size()==0 or available_ind_arm1_[cam_name].size()==0) {
                                available_poses_arm1_.erase(cam_name);
                                available_ind_arm1_.erase(cam_name);
                            }
                        }
                    }
                }
            }
        }
    }
} // add a pose to a camera in arm1 reach i.e. 3 bins

void AvailableBinPoses::addToAvailableBinPosesArm2(std::string cam_name, geometry_msgs::Pose cam_pose)
{
    auto parts_in_bin = (*all_bin_parts)[cam_name];

    for (auto cam_elements_it = parts_in_bin.begin(); cam_elements_it != parts_in_bin.end(); ++cam_elements_it)
    {
        for (auto pose_it = cam_elements_it->second.begin(); pose_it != cam_elements_it->second.end(); ++pose_it)
        {
            // for (int i = 0; i < static_poses_.size(); ++i)
            for (int i =0; i<static_poses_.size(); ++i)
            {
                auto static_world_pose = getStaticBinPoseInWorld(cam_name, cam_pose, static_poses_.at(i));
                if (!isInProximity(*pose_it, static_world_pose))
                {
                    if (available_ind_arm2_.count(cam_name) and available_poses_arm2_.count(cam_name))
                    {
                        // check for duplicacy
                        if (std::find(available_ind_arm2_[cam_name].begin(), available_ind_arm2_[cam_name].end(),i) == available_ind_arm2_[cam_name].end())
                        {
                            available_ind_arm2_[cam_name].push_back(i);
                            available_poses_arm2_[cam_name].push_back(static_world_pose);
                        }
                    }
                    else
                    {
                        available_ind_arm2_[cam_name] = std::vector<int>{i};
                        available_poses_arm2_[cam_name] = std::vector<geometry_msgs::Pose>{static_world_pose};
                    }
                }
                else
                {
                    if (available_ind_arm2_.count(cam_name) and available_poses_arm2_.count(cam_name))
                    {
                        auto it = std::find(available_ind_arm2_[cam_name].begin(), available_ind_arm2_[cam_name].end(), i);
                        if (it != available_ind_arm2_[cam_name].end())
                        {
                            available_ind_arm2_[cam_name].erase(it);
                            auto ind = std::distance(available_ind_arm2_[cam_name].begin(), it);
                            auto jt = available_poses_arm2_[cam_name].begin() + ind;
                            available_poses_arm2_[cam_name].erase(jt);
                            if (available_poses_arm2_[cam_name].size() == 0 or available_ind_arm2_[cam_name].size() == 0)
                            {
                                available_poses_arm2_.erase(cam_name);
                                available_ind_arm2_.erase(cam_name);
                            }
                        }
                    }
                }
            }
        }
    }
    // if (common_poses_arm2_.count(cam_name))
    // {
    //     if (count_.count(cam_name) == 0)
    //     {
    //         count_[cam_name] = 0;
    //     }
    //     common_poses_arm2_[cam_name].push_back(setStaticBinPoseInWorld(cam_name, cam_pose, static_poses_.at(count_[cam_name])));
    //     count_[cam_name]++;
    // }
    // else
    // {
    //     std::vector<geometry_msgs::Pose> poses_vec{setStaticBinPoseInWorld(cam_name, cam_pose, static_poses_.at(count_[cam_name]))};
    //     common_poses_arm2_[cam_name] = poses_vec;
    //     count_[cam_name]++;
    // }
} // add a pose to a camera in arm2 reach i.e. 3 bins

void AvailableBinPoses::clearBinFromArm1(std::string cam_name) {
    if (available_ind_arm1_.count(cam_name) and available_poses_arm1_.count(cam_name))
    {
        available_ind_arm1_.erase(cam_name);
        available_poses_arm1_.erase(cam_name);
    }
}

void AvailableBinPoses::clearBinFromArm2(std::string cam_name) {
    if (available_ind_arm2_.count(cam_name) and available_poses_arm2_.count(cam_name)) 
    {
        available_ind_arm2_.erase(cam_name);
        available_poses_arm2_.erase(cam_name);
    }
}

geometry_msgs::Pose AvailableBinPoses::getAvailableBinPoseArm1()
{
    // get avaiable pose from any of the cameras
    // once you get a pose, delete it from the vector<poses>
    auto first_available_cam_it = available_poses_arm1_.begin(); //std::map<std::string, std::vector<geometry_msgs::Pose>>
    auto pose = first_available_cam_it->second.back();
    // first_available_cam_it->second.pop_back();
    // available_ind_arm1_[first_available_cam_it->first].pop_back();
    return pose;
} // returns a pose available in one of the 3bins for arm2

geometry_msgs::Pose AvailableBinPoses::getAvailableBinPoseArm2()
{
    // get avaiable pose from any of the cameras
    // once you get a pose, delete it from the vector<poses>
    auto first_available_cam_it = available_poses_arm2_.begin(); //std::map<std::string, std::vector<geometry_msgs::Pose>>
    auto pose = first_available_cam_it->second.back();
    // first_available_cam_it->second.pop_back();
    // available_ind_arm2_[first_available_cam_it->first].pop_back();
    return pose;
} // returns a pose available in one of the 3bins for arm1
