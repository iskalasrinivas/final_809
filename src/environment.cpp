#include <environment.h>

Environment::Environment()
: availablebinposes_(&all_binParts)
, all_binCamera_called(false)
, all_trayCamera_called(false)
, all_qualityCamera_called(false)
, trayCameraRequired(true)  // Earlier :false
, order_manager_status(false)
, binCameraRequired(true) // Earlier :false
, conveyorTrigger(false)
{
	// common trash pose
	//	trash_bin_pose_.position.x = -0.20;
	//	trash_bin_pose_.position.y = 0.0;
	//	trash_bin_pose_.position.z = 0.95;
	//	trash_bin_pose_.orientation.w = 0;
	//	trash_bin_pose_.orientation.x = 0;
	//	trash_bin_pose_.orientation.y = 0;
	//	trash_bin_pose_.orientation.z = 0;
	pq["agv1"] = new PriorityQueue();
	pq["agv2"] = new PriorityQueue();
	pickuplocations["agv1"];
	pickuplocations["agv2"];

	loopFunction();

};

void Environment::loopFunction() {

	available_bin_thread = std::thread(&Environment::updateAvailableBinPoses, this);
	//	available_bin_thread_agv2 = std::thread(&Environment::updateAvailablebinPosesAGV2, this);

	//		available_bin_thread_agv1.join();
}

void Environment::singleUpdateforAvailableBinPoses() {
	std::map<std::string,geometry_msgs::Pose>available_cam_pose = available_cam_pose_map;
	for(auto cam_it = available_cam_pose.begin(); cam_it != available_cam_pose.end();++cam_it) {
//		ROS_INFO_STREAM("Updating Available Bin Poses");
		availablebinposes_.addToAvailableBinPoses(cam_it->first, cam_it->second);
	}
}

void Environment::updateAvailableBinPoses() {
	while(ros::ok()) {
		ensureAllPartsinAllBinsareUpdated();
		availablebinposes_.takesCareofAllCamera();
		singleUpdateforAvailableBinPoses();

	}

	ros::Duration(1.5).sleep();
}




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
	setAllTrayCameraCalled(false);

	while (!isAllTrayCameraCalled())
	{
		ros::Duration(0.1).sleep();
	}
	//	setTrayCameraRequired(false);
}
void  Environment::ensureAllPartsinAllBinsareUpdated() {
	setBinCameraRequired(true);
	setAllBinCameraCalled(false);
	while (!isAllBinCameraCalled())
	{
		ros::Duration(0.1).sleep();
	}
	//	setBinCameraRequired(false);
}

std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* Environment::getAllTrayParts() {

	return &tray_parts;
}

geometry_msgs::Pose Environment::getTrashBinPose(std::string agv_id) {
	geometry_msgs::Pose trashbinpose;
	if(agv_id == "agv1") {
		trashbinpose = trash_bin_pose_agv1;
	} else if(agv_id == "agv1") {
		trashbinpose =  trash_bin_pose_agv2;
	}
	return trashbinpose;
}

void Environment::setTrashBinPose(geometry_msgs::Pose t1, geometry_msgs::Pose t2){
	trash_bin_pose_agv1= t1;
	trash_bin_pose_agv2 =t2;

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
	return quality_cam_bool_map_[agv_id];
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

std::map<std::string, PriorityQueue*>* Environment::getPriorityQueue()
{
	return &pq;
}
void Environment::clearPriorityQueue()
{
	pq["agv1"]->clear();
	pq["agv2"]->clear();
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
			if (part_it->position.y > -1.525) {
				parttype_count_agv1[part_type] += 1;
			}
			if (part_it->position.y < 1.525) {
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

	//	setTrayCameraRequired(false);
	//	setBinCameraRequired(false);

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

std::vector<std::vector <OrderPart*>> * Environment::getShipmentsOfAnyTagId() {
	return &shipment_with_ANY_tag;
}
void Environment::clearANYvector() {
	shipment_with_ANY_tag.clear();
}

std::vector<std::vector<OrderPart*>>* Environment::getshipmentVector(){
	return &shipment_vector_;
}


void Environment::clearShipmentVector() {
	shipment_vector_.clear();
}

PriorityQueue* Environment::getPreOrderForArm2() {
	return &pre_order_arm2;
}

PriorityQueue* Environment::getPreOrderForArm1() {
	return &pre_order_arm1;
}


void Environment::clearBinFromArm1(std::string cam_name){
	available_cam_pose_map.erase(cam_name);
}

void Environment::clearBinFromArm2(std::string cam_name){
	available_cam_pose_map.erase(cam_name);
}

void Environment::addToAvailableBinPoses(std::string cam_name , geometry_msgs::Pose cam_pose, int size){
	if(!available_cam_size.count(cam_name)){
		available_cam_size[cam_name] = size;
		available_cam_pose_map[cam_name] = cam_pose;
	} else {
		if(available_cam_size[cam_name] != size) {
			available_cam_pose_map[cam_name] = cam_pose;
		}
	}
}


