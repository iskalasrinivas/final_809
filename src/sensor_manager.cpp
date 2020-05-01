#include<sensor_manager.h>

SensorManager::SensorManager(Environment * env):
 async_spinner(0),
 env_(env),
 lcamera_agv1("/ariac/logical_camera_1", env_, false, false, true, false),
 lcamera_bin1("/ariac/logical_camera_2", env_, false, true, false, false),
 lcamera_bin2("/ariac/logical_camera_3", env_, false, true, false, false),
 lcamera_common_bin("/ariac/logical_camera_4", env_, false, true, false, false),
 lcamera_bin4("/ariac/logical_camera_5", env_, false, true, false, false),
 lcamera_bin5("/ariac/logical_camera_6", env_, false, true, false, false),
 lcamera_bin6("/ariac/logical_camera_7", env_, false, true, false, false),
 lcamera_agv2("/ariac/logical_camera_8", env_, false, false, true, false),
 lcamera_belt("/ariac/logical_camera_9", env_, true, false, false, false),
 lcamera_trigger("/ariac/logical_camera_10", env_, false, false, false, true),
 lcamera_belt2("/ariac/logical_camera_11", env_, true, false, false, false),
 qcamera_agv1("/ariac/quality_control_sensor_1","agv1", env_),
 qcamera_agv2("/ariac/quality_control_sensor_2","agv2", env_)
{
async_spinner.start();
}

SensorManager::~SensorManager(){};
