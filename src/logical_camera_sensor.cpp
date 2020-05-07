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
	//	was_trigger_cam_empty = true;

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
	ros::Duration(0.2).sleep();
	logical_subscriber_ = logical_nh_.subscribe(topic_, 10, &LogicalCameraSensor::logicalCameraCallback, this);
	ros::Duration(0.2).sleep();
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
	//	ROS_INFO_STREAM("<<<<<Sorting all bin parts>>>>>");
	auto sorted_all_binParts = environment_->getSortedBinParts();
	auto all_binParts = environment_->getAllBinParts();
	std::map<std::string, std::vector<geometry_msgs::Pose>> temp = *sorted_all_binParts;
	//	sorted_all_binParts->clear();
	for (auto cam_id : *all_binParts) {
		for (auto map_parts : cam_id.second) {
			auto part_type = map_parts.first;
			auto vec_parts = map_parts.second;
			if (temp.count(part_type)) {
				temp[part_type].insert(temp[part_type].end(), vec_parts.begin(),
						vec_parts.end());
			} else {
				temp[part_type] = vec_parts;
			}
		}
	}
	*sorted_all_binParts = temp;

}

void LogicalCameraSensor::logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{ 

	if (beltcam_ == true) {
		ros::Duration(0.001).sleep();
		if(cam_name == "logical_camera_9") {
			beltLogicalCameraCallback("agv1", image_msg);
		} else if(cam_name == "logical_camera_11") {
			beltLogicalCameraCallback("agv2", image_msg);
		}
	}
	if (triggercam_ == true) {
		ros::Duration(0.01).sleep();
		beltTriggerLogicalCameraCallback(image_msg);
	}
	if(bincam_ == true or traycam_ == true){
		ros::Duration(1.0).sleep();

		binAndTrayLogicalCameraCallback(image_msg);

	}
}

bool LogicalCameraSensor::was_trigger_cam_empty {true};
// TODO
void LogicalCameraSensor::beltTriggerLogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
	std::map<std::string, std::set<OrderPart *> > * unavaialbaleParts = environment_->getUnavailableParts();
	std::map<std::string, std::map<std::string, geometry_msgs::Pose*> >* pickuplocations =   environment_->getPickupLocations();
	if(image_msg->models.empty()) {was_trigger_cam_empty = true;}

	if(was_trigger_cam_empty) {
		if (!image_msg->models.empty()) {
			was_trigger_cam_empty = false;
			for (auto it = image_msg->models.begin(); it != image_msg->models.end(); ++it) {
				bool should_break = false;
				for (auto uap_map_it = unavaialbaleParts->begin(); uap_map_it != unavaialbaleParts->end(); ++uap_map_it ) {
					for (auto part_it = uap_map_it->second.begin(); part_it != uap_map_it->second.end(); ++part_it ) {
						if((*part_it)->getPartType() == it->type) {
							(*part_it)->setHighestPriority();
							if((*pickuplocations)[(*part_it)->getAgvId()].count((*part_it)->getPartType()) == 0) {
								std::map<std::string, geometry_msgs::Pose*> temp;
								//								temp.insert(std::pair<std::string, geometry_msgs::Pose*>((*part_it)->getPartType(), nullptr));
								//								pickuplocations->insert(std::pair<std::string,std::map<std::string, geometry_msgs::Pose*>>((*part_it)->getAgvId(), temp));
								(*pickuplocations)[(*part_it)->getAgvId()][(*part_it)->getPartType()] = nullptr;

								//								ROS_WARN_STREAM("LC Created T : " << (*part_it)->getAgvId() << " " << (*part_it)->getPartType());
								if((*pickuplocations)[(*part_it)->getAgvId()][(*part_it)->getPartType()] == nullptr) {
									//									ROS_INFO_STREAM("LC : nullptr");
								}
								should_break = true;
							}
						}
						if(should_break) { break; }
					}
					if(should_break) { break; }
				}
			}
		}
	}

	//		ROS_INFO_STREAM("<<<<<Sequence of Trigger Camera Callback is finished !!>>>>>");
}


// TODO : @
void LogicalCameraSensor::beltLogicalCameraCallback(std::string agv_id, const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
	auto sensor_pose = image_msg->pose;
	// transform_.setParentPose(sensor_pose);

	std::map<std::string, geometry_msgs::Pose*>* armpickuplocation = &((*environment_->getPickupLocations())[agv_id]);
	//	std::map<std::string, std::map<std::string, geometry_msgs::Pose*>>* apl = environment_->getPickupLocations();
	//	std::map<std::string, std::map<std::string, geometry_msgs::Pose*>>::iterator armpickuplocation = apl->find(agv_id);
	//	ROS_WARN_STREAM("STUCK0");
	//	ROS_WARN_STREAM("Size of belt array : " << image_msg->models.size());
	for (auto it = image_msg->models.begin(); it != image_msg->models.end(); ++it) {
		//		ROS_WARN_STREAM("STUCK11");
		if (armpickuplocation->count(it->type)) {
			//			ROS_WARN_STREAM("STUCK1");
			geometry_msgs::Pose pose = transform_.getChildPose(it->pose);
			if (armpickuplocation->at(it->type) == nullptr) {
				//				ROS_WARN_STREAM("STUCK2");
				geometry_msgs::Pose* pose_ = new geometry_msgs::Pose();
				*pose_ = pose;
				//				armpickuplocation->second.insert(std::pair<std::string, geometry_msgs::Pose*>(it->type, pose_ ));
				//				armpickuplocation->insert(std::make_pair(it->type, pose_ ));
				//				armpickuplocation->at(it->type) = new geometry_msgs::Pose();
				armpickuplocation->at(it->type) = pose_;
				//				ROS_WARN_STREAM("STUCK22");
				//				ROS_WARN_STREAM("LC New B " << *(environment_->getPickupLocations()->at(agv_id).at(it->type)));
				//				ROS_WARN_STREAM("Value of pose assigned is new Pose()");
			} else if (armpickuplocation->at(it->type)->position.y > it->pose.position.y) {
				//								ROS_WARN_STREAM("STUCK3");
				//				*(armpickuplocation->at(it->type)) = pose;
				geometry_msgs::Pose* pose_ = armpickuplocation->at(it->type);
				*pose_ = pose;
				//				armpickuplocation->insert(std::pair<std::string, geometry_msgs::Pose*>(it->type, pose ));
				//				ROS_WARN_STREAM("LC Old B " << *(environment_->getPickupLocations()->at(agv_id).at(it->type)));
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
		// transform_.setParentPose(sensor_pose);

		std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* currentPartsPtr;

		if ((*bincammap_).count(cam_name)) {
			currentPartsPtr = environment_->getAllBinParts();
		}
		else if (traycammap_->count(cam_name)) {

			currentPartsPtr = environment_->getAllTrayParts();
		}

		std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> currentParts = *currentPartsPtr;

		if (currentParts.count(cam_name) == 1) {
			currentParts[cam_name].clear();
		}

		for (auto it = image_msg->models.begin(); it != image_msg->models.end(); ++it) {
			// transform_.setChildPose(it->pose);
			// transform_.setWorldTransform();
			auto partType = it->type;
			geometry_msgs::Pose pose = transform_.getChildPose(it->pose);
			if (currentParts[cam_name].count(partType))
			{
				currentParts[cam_name][partType].push_back(pose);
			}
			else {
				currentParts[cam_name][partType] = std::vector<geometry_msgs::Pose>{ pose };
			}
		}
		*currentPartsPtr = currentParts;
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
			//			ROS_INFO_STREAM(cam_name << " : Bin Debug : " << count << " of " << bincamsize_);
			if (count == bincamsize_) {
				SortAllBinParts();
				environment_->setAllBinCameraCalled(true);
				//				environment_->setBinCameraRequired(false);
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
			//			ROS_INFO_STREAM(cam_name << " : Tray Debug : " << count << " of " <<traycamsize_);
			if (count == traycamsize_) {
				environment_->setAllTrayCameraCalled(true);
				//				environment_->setTrayCameraRequired(false);
				environment_->resetTrayCamBoolmap();
			}
		}
		if (bincam_) {
			if (image_msg->models.size() < 4) {
				environment_->addToAvailableBinPoses(cam_name, image_msg->pose, image_msg->models.size());
				//
			} else {
				environment_->clearBinFromArm1(cam_name);
			}
		}
		//		ROS_INFO_STREAM("<<<<<Sequence of Tray & bin Camera Callback is finished !!>>>>>");
	}
}
