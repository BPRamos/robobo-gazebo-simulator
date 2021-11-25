#ifndef GAZEBO_ROS_QR_SENSOR_HH
#define GAZEBO_ROS_QR_SENSOR_HH
   
#include <string>
    
// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>
    
#include <gazebo_plugins/gazebo_ros_camera_utils.h>


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>

#include "gazebo_msgs/LinkStates.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"

#include <zxing/qrcode/QRCodeReader.h>

namespace gazebo
{
  class GazeboRosQRDetector : public CameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosQRDetector();
 
    /// \brief Destructor
    public: ~GazeboRosQRDetector();
  
    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    
    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);
   
    //brief topic name
    std::string topic_name_;

    //brief for setting ROS name space
    std::string robot_namespace_;

    //std::unique_ptr<ros::NodeHandle> _nh;
    ros::NodeHandle* _nh;
    ros::Publisher _qrCodePublisher;
    ros::Publisher _qrCodeChangePublisher;
    
    QRCodeReader _reader;
    
    int _lostThreshold = 5;
    int _formatErrorCount = 0;
    int _lostCount = 0;
    QRInfo _currentQR = NULL;
    
    double _fov;
    double _range;
  };
}
#endif
