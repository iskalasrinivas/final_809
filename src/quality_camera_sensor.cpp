#include <quality_camera_sensor.h>

QualityCameraSensor::QualityCameraSensor(std::string topic,std::string a_id, Environment * env):
async_spinner(0), agv_id(a_id), environment_(env) {
 async_spinner.start();
 auto qualitycamboolmap_ = environment_->getQualityCamBoolMap();
 (*qualitycamboolmap_)[agv_id]= false;
 quality_subscriber_= quality_nh_.subscribe(topic, 10 ,&QualityCameraSensor::qualityControlSensorCallback, this);
}

QualityCameraSensor::~QualityCameraSensor(){}



void QualityCameraSensor::qualityControlSensorCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

 // update the faulty status for respective camname
 if(environment_->isQualityCameraRequired(agv_id)) {
  auto qualitycamcalledmap_ = environment_->getQualityCamBoolMap();
  auto qualitycamfaultymap_ = environment_->getQualityCamerasPartfaulty();
  (*qualitycamfaultymap_)[agv_id] = !image_msg->models.empty();
  (*qualitycamcalledmap_)[agv_id] = true;
 }
}
