#ifndef GROUP6_RWA4_ENVIRONMENT_H
#define GROUP6_RWA4_ENVIRONMENT_H

#include <available_bin_poses.h>
#include <geometry_msgs/Pose.h>
#include <order_part.h>
#include <osrf_gear/Order.h>
#include <priority_queue.h>
#include <ros/ros.h>
#include <array>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

class Environment
{
private:
  std::map<std::string, std::set<OrderPart *>> unAvailablePartsForArm;  // agv_id

  std::map<std::string, std::map<std::string, geometry_msgs::Pose*>> pickuplocations;  // agv_id, part_type
  std::map<std::string, PriorityQueue> pq;                                            //  string here is agv_id
  AvailableBinPoses availablebinposes_;
  std::map<std::string, int> parttype_count_agv1, parttype_count_agv2;
  std::map<std::string, int> parttype_bin_count_agv1, parttype_bin_count_agv2;

  std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> all_binParts;
  std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> tray_parts;
  std::map<std::string, std::vector<geometry_msgs::Pose> > sorted_all_binParts;
  std::map<int, std::map<std::string, int> > shipments_;
  std::map<std::string, std::vector<OrderPart*> > completed_shipment_;

  std::map<std::string, bool> is_faulty;
  geometry_msgs::Pose trash_bin_pose_;

  //  to check whether the callback function is called or not
  std::map<std::string, bool> bin_cam_bool_map_;
  std::map<std::string, bool> tray_cam_bool_map_;
  std::map<std::string, bool> quality_cam_bool_map_;
  std::map<std::string, bool> belt_cam_bool_map_;

  bool all_binCamera_called;
  bool all_trayCamera_called;
  bool all_qualityCamera_called;

  bool binCameraRequired;
  bool trayCameraRequired;

  bool order_manager_status;

  bool conveyorTrigger;

public:
  Environment();
  ~Environment();

  void setAllBinParts();
  void sortAllBinParts();
  void setorderManagerStatus(bool);

  std::array<std::map<std::string, int>, 2> getCountOfavailablePartsArmWise();

  std::map<std::string, std::vector<geometry_msgs::Pose>> *getSortedBinParts();
  std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> *getAllBinParts();
  std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> *getAllTrayParts();
  std::map<std::string, std::vector<geometry_msgs::Pose> > *getTray1Parts();
  std::map<std::string, std::vector<geometry_msgs::Pose> > *getTray2Parts();
  geometry_msgs::Pose getTrashBinPose();
  std::map<std::string, std::map<std::string, geometry_msgs::Pose*>> *getPickupLocations();

  bool getOrderManagerStatus() const;

  std::map<std::string, bool> *getBinCamBoolMap();
  std::map<std::string, bool> *getTrayCamBoolMap();
  std::map<std::string, bool> *getQualityCamBoolMap();
  std::map<std::string, bool> *getBeltCamBoolMap();
  std::map<std::string, bool> *getQualityCamerasPartfaulty();

  bool isQualityCamera1Partfaulty();
  bool isQualityCamera2Partfaulty();
  bool isAllTrayCameraCalled();
  void setAllTrayCameraCalled(const bool);

  bool isBinCameraRequired();
  void setBinCameraRequired(const bool);

  bool isAllBinCameraCalled();
  void setAllBinCameraCalled(const bool);

  bool isTrayCameraRequired();
  void setTrayCameraRequired(const bool);

  void resetBinCamBoolmap();
  void resetTrayCamBoolmap();
  void resetQualityCamBoolmap();

  void setSeeQualityCamera1(bool);
  void setSeeQualityCamera2(bool);
  bool seeQualityCamera(std::string);

  bool isQuality1Called();
  bool isQuality2Called();

  void setConveyor1Trigger(const bool &);
  bool isConveyor1Triggered() const;

  void setConveyor2Trigger(const bool &);
  bool isConveyor2Triggered() const;

  // priority queue related member functions here please!!!
  std::map<std::string, PriorityQueue> *getPriorityQueue();
  void clearPriorityQueue();

  AvailableBinPoses *getAvailableBinPosesObject();

  void pushToUnavailableParts(OrderPart *part);
  std::set<OrderPart *> *getUnavailablePartsForArm1();
  std::set<OrderPart *> *getUnavailablePartsForArm2();
  std::map<std::string, std::set<OrderPart *>> *getUnavailableParts();
  std::array<std::map<std::string, int>, 2> getCountOfBinParts();

  std::map<int, std::map<std::string, int> >* getShipments();
  std::map<std::string, std::vector<OrderPart*> >* getCompletedShipment();
  void ensureAllPartsinAllBinsareUpdated();
  void ensureAllPartsinBothTraysareUpdated();
};

#endif  // GROUP6_RWA4_ENVIRONMENT_H
