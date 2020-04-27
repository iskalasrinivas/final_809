#include <quality_camera_sensor.h>

QualityCameraSensor::QualityCameraSensor(std::string topic, Environment * env):async_spinner(0), environment_(env) {
 async_spinner.start();
 getCameraName(topic);
 auto qualitycamboolmap_ = environment_->getQualityCamBoolMap();
 (*qualitycamboolmap_)[cam_name]= false;
 is_faulty = false;
 quality_subscriber_= quality_nh_.subscribe(topic, 10 ,&QualityCameraSensor::qualityControlSensorCallback, this);
}

QualityCameraSensor::~QualityCameraSensor(){}

std::string QualityCameraSensor::getCameraName(std::string topic_) {
 std::stringstream ss(topic_);
 const char delim = '/';
 std::string p_;
 while (std::getline(ss, p_, delim)) {
 }
 cam_name = p_;

 return cam_name;
}

void QualityCameraSensor::qualityControlSensorCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

 // update the faulty status for respective camname
 if(environment_->seeQualityCamera(cam_name)) {
  auto qualitycamcalledmap_ = environment_->getQualityCamBoolMap();
  auto qualitycamfaultymap_ = environment_->getQualityCamerasPartfaulty();
  (*qualitycamfaultymap_)[cam_name] = !image_msg->models.empty();
  (*qualitycamcalledmap_)[cam_name] = true;
 }
}

bool QualityCameraSensor::isPartFaulty() {
 return is_faulty;
}