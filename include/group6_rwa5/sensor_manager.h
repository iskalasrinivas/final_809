#ifndef GROUP6_RWA5_SENSORMANAGER_H_
#define GROUP6_RWA5_SENSORMANAGER_H_

#include <environment.h>
#include <logical_camera_sensor.h>
#include <quality_camera_sensor.h>

class SensorManager{

private:
	ros::NodeHandle sensor_nh_;
	ros::AsyncSpinner async_spinner;
	Environment * env_;
	LogicalCameraSensor lcamera_bin1;
	LogicalCameraSensor lcamera_bin2;
	LogicalCameraSensor lcamera_common_bin;
	LogicalCameraSensor lcamera_bin4;
	LogicalCameraSensor lcamera_bin5;
	LogicalCameraSensor lcamera_bin6;
	LogicalCameraSensor lcamera_agv1;
	LogicalCameraSensor lcamera_agv2;
	LogicalCameraSensor lcamera_belt;
	LogicalCameraSensor lcamera_trigger;
	LogicalCameraSensor lcamera_belt2;
	QualityCameraSensor qcamera_agv1;
	QualityCameraSensor qcamera_agv2;

public:
	SensorManager(Environment *);
	~SensorManager();
}; 

#endif // GROUP6_RWA5_SENSORMANAGER_H_
