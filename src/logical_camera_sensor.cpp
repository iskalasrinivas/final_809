#include <logical_camera_sensor.h>

LogicalCameraSensor::LogicalCameraSensor(std::string topic, Environment* env, bool blc, bool bc, bool tc, bool trigcam)
: async_spinner(0),
  environment_(env),
  topic_(topic),
  bincam_(bc),
  traycam_(tc),
  beltcam_(blc),
  triggercam_(trigcam),
  transform_(topic) {
	async_spinner.start();
	getCameraName(topic);
	if (bincam_)
	{
		std::map<std::string, bool> * bincamboolmap_ = environment_->getBinCamBoolMap();
		(*bincamboolmap_)[cam_name] = false;
	}
	if (traycam_)
	{
		std::map<std::string, bool> * traycamboolmap_ = environment_->getTrayCamBoolMap();
		(*traycamboolmap_)[cam_name] = false;
	}

	if (beltcam_)
	{
		std::map<std::string, bool> * beltcamboolmap_ = environment_->getBeltCamBoolMap();
		(*beltcamboolmap_)[cam_name] = false;
	}

	logical_subscriber_ = logical_nh_.subscribe(topic_, 10, &LogicalCameraSensor::logicalCameraCallback, this);
}

LogicalCameraSensor::~LogicalCameraSensor() {}

std::string LogicalCameraSensor::getCameraName(std::string topic_)
{
	std::stringstream ss(topic_);
	const char delim = '/';
	std::string p_;
	while (std::getline(ss, p_, delim))
	{
	}
	cam_name = p_;

	return cam_name;
}



void LogicalCameraSensor::SortAllBinParts() {
	ROS_INFO_STREAM("<<<<<Sorting all bin parts>>>>>");
	auto sorted_all_binParts = environment_->getSortedBinParts();
	auto all_binParts = environment_->getAllBinParts();
	sorted_all_binParts->clear();
	for (auto cam_id : *all_binParts) {
		for (auto map_parts : cam_id.second) {
			auto part_type = map_parts.first;
			auto vec_parts = map_parts.second;
			if (sorted_all_binParts->count(part_type)) {
				(*sorted_all_binParts)[part_type].insert((*sorted_all_binParts)[part_type].end(), vec_parts.begin(),
						vec_parts.end());
			} else {
				(*sorted_all_binParts)[part_type] = vec_parts;
			}
		}
	}

}

void LogicalCameraSensor::logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{ 
	if (beltcam_ == true) {
		if(cam_name == "logical_camera_9") {
			beltLogicalCameraCallback("agv1", image_msg);
		} else if(cam_name == "logical_camera_11") {
			beltLogicalCameraCallback("agv2", image_msg);
		}
	}
	if (triggercam_ == true) {
		beltTriggerLogicalCameraCallback(image_msg);
	}
	if(bincam_ == true or traycam_ == true){

		binAndTrayLogicalCameraCallback(image_msg);

	}
}

// TODO
void LogicalCameraSensor::beltTriggerLogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
	std::map<std::string, std::set<OrderPart *> > * unavaialbaleParts = environment_->getUnavailableParts();
	std::map<std::string, std::map<std::string, geometry_msgs::Pose*> >* pickuplocations =   environment_->getPickupLocations();

	if (!image_msg->models.empty()) {
		for (auto it = image_msg->models.begin(); it != image_msg->models.end(); ++it) {
			bool should_break = false;
			for (auto uap_map_it = unavaialbaleParts->begin(); uap_map_it != unavaialbaleParts->end(); ++uap_map_it ) {
				for (auto part_it = uap_map_it->second.begin(); part_it != uap_map_it->second.end(); ++part_it ) {
					if((*part_it)->getPartType() == it->type) {
						(*part_it)->setHighestPriority();
						(*pickuplocations)[(*part_it)->getAgvId()][(*part_it)->getPartType()] = nullptr;
						should_break = true;
					}
					if(should_break) { break; }
				}
				if(should_break) { break; }
			}
		}
	}

//	ROS_INFO_STREAM("<<<<<Sequence of Trigger Camera Callback is finished !!>>>>>");
}


// TODO : @  
void LogicalCameraSensor::beltLogicalCameraCallback(std::string agv_id, const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
	auto sensor_pose = image_msg->pose;
	transform_.setParentPose(sensor_pose);
	auto armpickuplocation = &(*environment_->getPickupLocations())[agv_id];

	for (auto it = image_msg->models.begin(); it != image_msg->models.end(); ++it) {
		if (armpickuplocation->count(it->type)) {
			transform_.setChildPose(it->pose);
			transform_.setWorldTransform();
			geometry_msgs::Pose pose = transform_.getChildWorldPose();
			if (armpickuplocation->at(it->type) == nullptr) {
				armpickuplocation->at(it->type) = new geometry_msgs::Pose(pose);
			} else if (armpickuplocation->at(it->type)->position.y > it->pose.position.y) {
				*(armpickuplocation->at(it->type)) = pose;
			}
		}
	}

//	ROS_INFO_STREAM("<<<<<Sequence of Belt Pick up Finished !!>>>>>");
}

void LogicalCameraSensor::binAndTrayLogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {

	if (environment_->isBinCameraRequired() || environment_->isTrayCameraRequired()) {

		std::map<std::string, bool> * bincammap_ = environment_->getBinCamBoolMap();
		std::map<std::string, bool> * traycammap_ = environment_->getTrayCamBoolMap();  // std::map<std::string, bool>*
		auto sensor_pose = image_msg->pose;
		transform_.setParentPose(sensor_pose);

		std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* currentPartsPtr;
		if ((*bincammap_).count(cam_name)) {
			currentPartsPtr = environment_->getAllBinParts();
		}
		else if (traycammap_->count(cam_name)) {

			currentPartsPtr = environment_->getAllTrayParts();
		}

		if (currentPartsPtr->count(cam_name) == 1) {
			(*currentPartsPtr)[cam_name].clear();
		}

		for (auto it = image_msg->models.begin(); it != image_msg->models.end(); ++it) {
			transform_.setChildPose(it->pose);
			transform_.setWorldTransform();
			auto partType = it->type;
			geometry_msgs::Pose pose = transform_.getChildWorldPose();
			if ((*currentPartsPtr)[cam_name].count(partType)) {
				(*currentPartsPtr)[cam_name][partType].push_back(pose);
			}
			else {
				(*currentPartsPtr)[cam_name][partType] = std::vector<geometry_msgs::Pose>{ pose };
			}
		}

//		if (bincammap_->count(cam_name)) { // sort every time :not effective :see 191
//
//			SortAllBinParts();
//		}
//	}
		if (bincam_) {
			auto bincambool_ = environment_->getBinCamBoolMap();
			(*bincambool_)[cam_name] = true;
			auto bincamsize_ = bincambool_->size();
			int count = 0;
			for (auto it = bincambool_->begin(); it != bincambool_->end(); ++it) {
				if ((*it).second == true) {
					count += 1;
				}
			}
			ROS_INFO_STREAM(cam_name << " : Bin Debug : " << count << " of " << bincamsize_);
			if (count == bincamsize_) {
				SortAllBinParts();
				environment_->setAllBinCameraCalled(true);
				environment_->setBinCameraRequired(false);
				environment_->resetBinCamBoolmap();
			}
		}

		if (traycam_) {
			auto traycambool_ = environment_->getTrayCamBoolMap();
			(*traycambool_)[cam_name] = true;
			auto traycamsize_ = traycambool_->size();
			int count = 0;
			for (auto it = traycambool_->begin(); it != traycambool_->end(); ++it) {
				if ((*it).second == true) {
					count += 1;
				}
			}
			ROS_INFO_STREAM(cam_name << " : Tray Debug : " << count << " of " <<traycamsize_);
			if (count == traycamsize_) {
				environment_->setAllTrayCameraCalled(true);
				environment_->setTrayCameraRequired(false);
				environment_->resetTrayCamBoolmap();
			}
		}
		if (bincam_) {
			if (image_msg->models.size() < 4) {
				if (image_msg->pose.position.y >= 0) {  // get camera position with respect to world and check it
					environment_->getAvailableBinPosesObject()->addToAvailableBinPosesArm1(cam_name, image_msg->pose);
				} else if (image_msg->pose.position.y < 0) {
					environment_->getAvailableBinPosesObject()->addToAvailableBinPosesArm2(cam_name, image_msg->pose);
				}
			} else {
				if (image_msg->pose.position.y >= 0) {
					environment_->getAvailableBinPosesObject()->clearBinFromArm1(cam_name);
				} else if (image_msg->pose.position.y < 0) {
					environment_->getAvailableBinPosesObject()->clearBinFromArm2(cam_name);
				}
			}
		}
//		ROS_INFO_STREAM("<<<<<Sequence of Tray & bin Camera Callback is finished !!>>>>>");
	}
}
