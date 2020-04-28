#include <environment.h>

Environment::Environment()
: availablebinposes_(&all_binParts)
, all_binCamera_called(false)
, all_trayCamera_called(false)
, all_qualityCamera_called(false)
, trayCameraRequired(false)
, order_manager_status(false)
, binCameraRequired(false)
, conveyorTrigger(false)
{
	// common trash pose
	trash_bin_pose_.position.x = -0.20;
	trash_bin_pose_.position.y = 0.0;
	trash_bin_pose_.position.z = 0.95;
	trash_bin_pose_.orientation.w = 0;
	trash_bin_pose_.orientation.x = 0;
	trash_bin_pose_.orientation.y = 0;
	trash_bin_pose_.orientation.z = 0;
};

Environment::~Environment(){};

std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* Environment::getAllBinParts() {
	return &all_binParts;
}

std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getSortedBinParts() {
	return &sorted_all_binParts;
}

std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getTray1Parts() {
	return &tray_parts["logical_camera_1"];
}

std::map<std::string, std::vector<geometry_msgs::Pose>>* Environment::getTray2Parts() {
	return &tray_parts["logical_camera_8"];
}
void  Environment::ensureAllPartsinBothTraysareUpdated() {
	setTrayCameraRequired(true);

	while (!isAllTrayCameraCalled())
	{
		ros::Duration(0.1).sleep();
	}
	setTrayCameraRequired(false);
}
void  Environment::ensureAllPartsinAllBinsareUpdated() {
	setBinCameraRequired(true);

	while (!isAllBinCameraCalled())
	{
		ros::Duration(0.1).sleep();
	}
	setBinCameraRequired(false);
}

std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* Environment::getAllTrayParts() {

	return &tray_parts;
}

geometry_msgs::Pose Environment::getTrashBinPose() {
	return trash_bin_pose_;
}

void Environment::setorderManagerStatus(bool status) {
	order_manager_status = status;
}

bool Environment::getOrderManagerStatus() const {
	return order_manager_status;
}

bool Environment::isAllBinCameraCalled() {
	return all_binCamera_called;
}

bool Environment::isAllTrayCameraCalled() {
	return all_trayCamera_called;
}

bool Environment::isBinCameraRequired() {
	return binCameraRequired;
}

bool Environment::isTrayCameraRequired(){
	return trayCameraRequired;
}

void Environment::setBinCameraRequired(const bool cond) {
	binCameraRequired = cond;
}

void Environment::setTrayCameraRequired(const bool cond)
{
	trayCameraRequired = cond;
}

std::map<std::string, bool>* Environment::getBinCamBoolMap()
{
	return &bin_cam_bool_map_;
}

std::map<std::string, bool>* Environment::getTrayCamBoolMap()
{
	return &tray_cam_bool_map_;
}

std::map<std::string, bool>* Environment::getQualityCamBoolMap()
{
	return &quality_cam_bool_map_;
}

std::map<std::string, bool>* Environment::getBeltCamBoolMap()
{
	return &belt_cam_bool_map_;
}

std::map<std::string, std::map<std::string, geometry_msgs::Pose*>>* Environment::getPickupLocations()
{
	return &pickuplocations;
}

void Environment::setAllBinCameraCalled(const bool cond)
{
	all_binCamera_called = cond;
}

void Environment::setAllTrayCameraCalled(const bool cond)
{
	all_trayCamera_called = cond;
}

void Environment::resetBinCamBoolmap()
{
	for (auto& it : bin_cam_bool_map_)
	{
		it.second = false;
	}
}

void Environment::resetTrayCamBoolmap()
{
	for (auto& it : tray_cam_bool_map_)
	{
		it.second = false;
	}
}

void Environment::resetQualityCamBoolmap()
{
	for (auto& it : quality_cam_bool_map_)
	{
		it.second = false;
	}
}

std::map<std::string, bool>* Environment::getQualityCamerasPartfaulty()
{
	return &is_faulty;
}

bool Environment::isQualityCamera1Partfaulty()
{
	return is_faulty["agv1"];
}
bool Environment::isQualityCamera2Partfaulty()
{
	return is_faulty["agv2"];
}

void Environment::setQualityCameraRequiredForArm1(bool status)
{
	quality_cam_bool_map_["agv1"] = status;
}

void Environment::setQualityCameraRequiredForArm2(bool status)
{
	quality_cam_bool_map_["agv2"] = status;
}

void Environment::setQualityCameraRequired(std::string agv_id, bool status)
{
	quality_cam_bool_map_[agv_id] = status;
}

bool Environment::isQualityCameraRequired(std::string agv_id)
{
	return quality_cam_bool_map_[agv_id];
}

bool Environment::isPartFaulty(std::string agv_id) {
	return is_faulty[agv_id];
}


bool Environment::isQualityCameraCalled(std::string agv_id)
{
	return all_qualityCamera_called[agv_id];
}


bool Environment::isQuality1Called()
{
	return all_qualityCamera_called["agv1"];
}

bool Environment::isQuality2Called()
{
	return all_qualityCamera_called["agv2"];
}

void Environment::setConveyor1Trigger(const bool& status)
{
	conveyorTrigger = status;
}

bool Environment::isConveyor1Triggered() const
{
	return conveyorTrigger;
}

void Environment::setConveyor2Trigger(const bool& status)
{
	conveyorTrigger = status;
}

bool Environment::isConveyor2Triggered() const
{
	return conveyorTrigger;
}

AvailableBinPoses* Environment::getAvailableBinPosesObject()
{
	return &availablebinposes_;
}

std::map<std::string, PriorityQueue>* Environment::getPriorityQueue()
{
	return &pq;
}
void Environment::clearPriorityQueue()
{
	pq.clear();
}

std::array<std::map<std::string, int>, 2> Environment::getCountOfavailablePartsArmWise() {
	setTrayCameraRequired(true);
	setBinCameraRequired(true);
	ensureAllPartsinAllBinsareUpdated();
	ensureAllPartsinBothTraysareUpdated();
	for (auto type_it = sorted_all_binParts.begin(); type_it != sorted_all_binParts.end(); ++type_it) {
		auto part_type = type_it->first;
		parttype_count_agv1[part_type] = 0;
		parttype_count_agv2[part_type] = 0;
		for (auto part_it = type_it->second.begin(); part_it != type_it->second.end(); ++part_it) {
			if (part_it->position.y > 0) {
				parttype_count_agv1[part_type] += 1;
			} else {
				parttype_count_agv2[part_type] += 1;
			}
		}
	}
	auto tray1_Parts = tray_parts["logical_camera_1"];
	for (auto tray1_it = tray1_Parts.begin(); tray1_it != tray1_Parts.end(); ++tray1_it) {
		if (!parttype_count_agv1.count(tray1_it->first)) {
			parttype_count_agv1[tray1_it->first] = 0;
		} else {
			parttype_count_agv1[tray1_it->first] += tray1_it->second.size();
		}
	}

	auto tray2_Parts = tray_parts["logical_camera_8"];
	for (auto tray2_it = tray2_Parts.begin(); tray2_it != tray2_Parts.end(); ++tray2_it)
	{
		if (!parttype_count_agv2.count(tray2_it->first))
		{
			parttype_count_agv2[tray2_it->first] = 0;
		}
		else
		{
			parttype_count_agv2[tray2_it->first] += tray2_it->second.size();
		}
	}

	setTrayCameraRequired(false);
	setBinCameraRequired(false);

	std::array<std::map<std::string, int>, 2> retArray{ parttype_count_agv1, parttype_count_agv2 };
	return retArray;
}

void Environment::pushToUnavailableParts(OrderPart* part)
{
	unAvailablePartsForArm[part->getAgvId()].insert(part);
}

std::set<OrderPart*>* Environment::getUnavailablePartsForArm1()
{
	return &unAvailablePartsForArm["agv1"];
}

std::set<OrderPart*>* Environment::getUnavailablePartsForArm2()
{
	return &unAvailablePartsForArm["agv2"];
}

std::map<std::string, std::set<OrderPart*>>* Environment::getUnavailableParts()
{
	return &unAvailablePartsForArm;
}

std::map<int, std::map<std::string, int>>* Environment::getShipments()
{
	return &shipments_;
}

std::map<std::string, std::vector<OrderPart*>>* Environment::getCompletedShipment()
{
	return &completed_shipment_;
}
