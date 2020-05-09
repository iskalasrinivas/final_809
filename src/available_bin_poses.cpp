#include <available_bin_poses.h>
#include <cmath>
#include <algorithm>

AvailableBinPoses::AvailableBinPoses(std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* abp): all_bin_parts(abp){
	emptyplace_ensured = false;
	for (size_t i = 0; i < 4; ++i)
	{
		static_poses_.emplace_back();
	}

	// static_poses_.at(0).position.x = 0.50310;
	// static_poses_.at(0).position.y = 0.20;
	// static_poses_.at(0).position.z = 0.1838;
	// static_poses_.at(0).orientation.x = 0.0150;
	// static_poses_.at(0).orientation.y = -0.523;
	// static_poses_.at(0).orientation.z = 0.013;
	// static_poses_.at(0).orientation.w = 0.8522;

	// static_poses_.at(1).position.x = 0.67722;
	// static_poses_.at(1).position.y = -0.1486;
	// static_poses_.at(1).position.z = 0.12;
	// static_poses_.at(1).orientation.x = 0.0150;
	// static_poses_.at(1).orientation.y = -0.523;
	// static_poses_.at(1).orientation.z = 0.013;
	// static_poses_.at(1).orientation.w = 0.8522;

	// static_poses_.at(2).position.x = 0.7993;
	// static_poses_.at(2).position.y = 0.1512;
	// static_poses_.at(2).position.z = 0.12;
	// static_poses_.at(2).orientation.x = 0.0150;
	// static_poses_.at(2).orientation.y = -0.523;
	// static_poses_.at(2).orientation.z = 0.013;
	// static_poses_.at(2).orientation.w = 0.8522;

	// static_poses_.at(3).position.x = 0.7993;
	// static_poses_.at(3).position.y = -0.1486;
	// static_poses_.at(3).position.z =  0.12;
	// static_poses_.at(3).orientation.x = 0.0150;
	// static_poses_.at(3).orientation.y = -0.523;
	// static_poses_.at(3).orientation.z = 0.013;
	// static_poses_.at(3).orientation.w = 0.8522;

	static_poses_.at(0).position.x = 0.5031;
	static_poses_.at(0).position.y = 0.20;
	static_poses_.at(0).position.z = 0.1838;
	static_poses_.at(0).orientation.x = -0.163;
	static_poses_.at(0).orientation.y = -0.399;
	static_poses_.at(0).orientation.z = 0.349;
	static_poses_.at(0).orientation.w = 0.8314;

	static_poses_.at(1).position.x = 0.5031;
	static_poses_.at(1).position.y = -0.20;
	static_poses_.at(1).position.z = 0.1837;
	static_poses_.at(1).orientation.x = -0.170;
	static_poses_.at(1).orientation.y = -0.397;
	static_poses_.at(1).orientation.z = 0.337;
	static_poses_.at(1).orientation.w = 0.8357;

	static_poses_.at(2).position.x = 0.3473;
	static_poses_.at(2).position.y = 0.1985;
	static_poses_.at(2).position.z = -0.010;
	static_poses_.at(2).orientation.x = -0.169;
	static_poses_.at(2).orientation.y = -0.40;
	static_poses_.at(2).orientation.z = 0.347;
	static_poses_.at(2).orientation.w = 0.829;

	static_poses_.at(3).position.x = 0.346;
	static_poses_.at(3).position.y = -0.20;
	static_poses_.at(3).position.z =  0.10;
	static_poses_.at(3).orientation.x = -0.158;
	static_poses_.at(3).orientation.y = -0.40;
	static_poses_.at(3).orientation.z = 0.345;
	static_poses_.at(3).orientation.w = 0.832;

}


AvailableBinPoses::~AvailableBinPoses()
{}

// geometry_msgs::Pose AvailableBinPoses::getStaticBinPoseInWorld(std::string cam_name, geometry_msgs::Pose cam_pose, geometry_msgs::Pose child_pose)
// {
//     transform_.fromCameraName(cam_name);
//     transform_.setChildPose(child_pose);
//     transform_.setParentPose(cam_pose);
//     transform_.setWorldTransform();
//     geometry_msgs::Pose world_pose = transform_.getChildWorldPose();
//     return world_pose;
// }

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

void AvailableBinPoses::takesCareofAllCamera() {
	available_poses_arm1_ = temp_poses_arm1_;
	available_poses_arm2_ = temp_poses_arm2_;
	temp_poses_arm1_.clear();
	temp_poses_arm2_.clear();
	//	ROS_INFO_STREAM(" AB : " << available_poses_arm1_.size() <<", "<< available_poses_arm2_.size());
}

void AvailableBinPoses::addToAvailableBinPoses(std::string cam_name, geometry_msgs::Pose cam_pose) {
	// auto all_bin_parts = env->getAllBinParts; //std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> *getAllBinParts();
	auto parts_in_bin = (*all_bin_parts)[cam_name];


	if(parts_in_bin.empty()) {
		for (int i = 0; i < static_poses_.size(); ++i) {
			// auto static_world_pose = getStaticBinPoseInWorld(cam_name, cam_pose, static_poses_.at(i));
			geometry_msgs::Pose static_world_pose = transform_.getChildPose(cam_name, static_poses_.at(i));
			//			ROS_WARN_STREAM(cam_name <<" Empty "<<static_world_pose.position.x<<", " << static_world_pose.position.y);
			if(static_world_pose.position.y > 0.0) {
				temp_poses_arm1_.push_back(static_world_pose);
			}
			if(static_world_pose.position.y < 0.0) {
				temp_poses_arm2_.push_back(static_world_pose);
			}
		}
	}else {

		for (int i = 0; i < static_poses_.size(); ++i) {
			bool willcollide = false;
			geometry_msgs::Pose static_world_pose = transform_.getChildPose(cam_name, static_poses_.at(i));
			//		ROS_WARN_STREAM(cam_name << " Not empty "<<static_world_pose.position.x <<" , "<< static_world_pose.position.y);
			for (auto cam_elements_it = parts_in_bin.begin(); cam_elements_it != parts_in_bin.end(); ++cam_elements_it) {
				//			ROS_WARN_STREAM(" NE Size : " << cam_elements_it->first << ",  "<< cam_elements_it->second.size());
				for (auto pose_it = cam_elements_it->second.begin(); pose_it != cam_elements_it->second.end(); ++pose_it) {
					if (isInProximity(*pose_it, static_world_pose)) {
						willcollide = true;

					}

				}
			}
			if(!willcollide) {
				if(static_world_pose.position.y > 0.0) {
					temp_poses_arm1_.push_back(static_world_pose);
				}
				if(static_world_pose.position.y < 0.0) {
					temp_poses_arm2_.push_back(static_world_pose);
				}
			}

		}
	}
	//	ROS_WARN_STREAM(cam_name<<" DebugAB : " << temp_poses_arm1_.size() <<", "<< temp_poses_arm2_.size());
}



// add a pose to a camera in arm1 reach i.e. 3 bins

geometry_msgs::Pose AvailableBinPoses::getAvailableBinPoseArm1(){
	geometry_msgs::Pose pose_ = available_poses_arm1_.front();
	auto remove_it = available_poses_arm1_.begin();

	for (auto pose_it = available_poses_arm1_.begin(); pose_it != available_poses_arm1_.end(); ++pose_it) {
		if(pose_it->position.y < 0) {
			available_poses_arm1_.erase(pose_it--);
			continue;
		}
		//		ROS_INFO_STREAM( "AP1: " << pose_it->position.x << ", " << pose_it->position.y);
		if(pose_.position.y >  pose_it->position.y) {
			pose_ = *pose_it;
			remove_it = pose_it;
		}
	}
	ROS_INFO_STREAM( "AP1: " << pose_.position.x << ", " << pose_.position.y<< ", " << pose_.position.z);
	available_poses_arm1_.erase(remove_it);
	return pose_;
}

geometry_msgs::Pose AvailableBinPoses::getAvailableBinPoseArm2() {
	//	ROS_ERROR_STREAM("AP A2 : " <<available_poses_arm2_.size());
	//	auto first_available_cam_it = available_poses_arm2_.begin(); //std::map<std::string, std::vector<geometry_msgs::Pose>>

	geometry_msgs::Pose pose_ = available_poses_arm2_.front();
	auto remove_it = available_poses_arm2_.begin();

	for (auto pose_it = available_poses_arm2_.begin(); pose_it != available_poses_arm2_.end(); ++pose_it) {
		//		ROS_INFO_STREAM( "AP2: " << pose_it->position.x << ", " << pose_it->position.y);
		if(pose_it->position.y > 0) {
			available_poses_arm1_.erase(pose_it--);
			continue;
		}
		if(pose_.position.y <  pose_it->position.y) {
			pose_ = *pose_it;
			remove_it = pose_it;
		}
	}
	ROS_INFO_STREAM( "AP2: " << pose_.position.x << ", " << pose_.position.y<< ", " << pose_.position.z);
	available_poses_arm2_.erase(remove_it);
	return pose_;
}
