
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "include/robobo/qr_sensor.h"
#include "gazebo_plugins/gazebo_ros_camera.h"
#include <string>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include "gazebo_plugins/gazebo_ros_utils.h"
#include "robobo_msgs/QrCode.h"
#include "robobo_msgs/QrCodeChange.h"

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/LocalBlockBinarizer.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosQRDetector)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosQRDetector::GazeboRosQRDetector():
  //_nh("robot"),
  _fov(6),
  _range(10)
  {

  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosQRDetector::~GazeboRosQRDetector()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void GazeboRosQRDetector::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    std::string sensorName = _parent->Name();
    this->topic_name_ = sensorName;
    this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "QrDetector");

    this->_nh = new ros::NodeHandle(this->robot_namespace_);
    
    this->_reader = new QRCodeMultiReader();

    this->_qrCodePublisher = this->_nh->advertise<robobo_msgs::QrCode>(topic_name_, 10);
    this->_qrCodeChangePublisher = this->_nh->advertise<robobo_msgs::QrCode>(topic_name_, 10);

    CameraPlugin::Load(_parent, _sdf);
    // copying from CameraPlugin into GazeboRosCameraUtilsexit
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(_parent, _sdf);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosQRDetector::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    static int seq=0;

    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();

    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
        this->parentSensor->SetActive(true);
    }
    else
    {
      if ((*this->image_connect_count_) > 0)
      {
        common::Time cur_time = this->world_->SimTime();
        if (cur_time - this->last_update_time_ >= this->update_period_)
        {
          this->PutCameraData(_image);
          this->PublishCameraInfo();
          this->last_update_time_ = cur_time;


          /// We are passing the ROS image to the CV bridge, and the Mat to
          /// Zxing. This is innefficient, and can be improved.

          Ref<OpenCVBitmapSource> source(new OpenCVBitmapSource(_image));
          Ref<BinaryBitmap> bitmap(new BinaryBitmap(new HybridBinarizer(source)));

          
          Ref<Result> res = this->_reader.decode(bitmap);
          //QRInfo
          if(res.getStatus() == DecodeStatus.NotFound){
            this->_lostCount++;
          } else if((res.getStatus() == DecodeStatus.ChecksumError)||(res.getStatus() == DecodeStatus.FormatError)){
            this->_formatErrorCount++;
          } else {
            if(!this->_currentQR){
              notifyQRAppear(qr);
              currentQr = qr;
            } else if(!this->_currentQR.getIdString().equals(qr.getIdString())){
              notifyQRDisappear(currentQr);
              notifyQRAppear(qr);
              currentQr = qr;
            }
            
            notifyQR(qr);
            lostCount = 0;
            formatErrorCount = 0;

          }
          if(((this->_lostCount + this->_formatErrorCount/2) > this->_threshold)&& (this->_currentQR)){
            notifyDissapear(this->_currentQR);
            this->_currentQR = null;
          }
          
        }
      }
    }
  }
}
