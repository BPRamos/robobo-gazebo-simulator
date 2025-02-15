/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013-2015, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jose Capriles, Bence Magyar. */

#include "gazebo_plugins/gazebo_ros_utils.h"
#include "include/robobo/infrared_range.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(InfraredRange)

////////////////////////////////////////////////////////////////////////////////
// Constructor
InfraredRange::InfraredRange()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
InfraredRange::~InfraredRange()
{
  this->range_queue_.clear();
  this->range_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void InfraredRange::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;

  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // Create the range topic name with the model name and sensor name
  std::string sensorName = _parent->Name();

  this->topic_name_ = sensorName;

  // save pointers
  this->sdf = _sdf;

  this->last_update_time_ = common::Time(0);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("InfraredRange controller requires a Ray Sensor as its parent");

  this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "Infrared Range");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("range", "Range plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  
  if (!this->sdf->HasElement("fov"))
  {
    ROS_WARN_NAMED("range", "Range plugin missing <fov>, defaults to 0.05");
    this->fov_ = 0.05;
  }
  else
    this->fov_ = _sdf->GetElement("fov")->Get<double>();

  if (!this->sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO_NAMED("range", "Range plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = this->sdf->Get<double>("gaussianNoise");

  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_INFO_NAMED("range", "Range plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = this->sdf->Get<double>("updateRate");

  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0 / this->update_rate_;
  else
    this->update_period_ = 0.0;

  this->range_connect_count_ = 0;

  this->range_msg_.header.frame_id = this->frame_name_;
  this->range_msg_.radiation_type = sensor_msgs::Range::INFRARED;

  this->range_msg_.field_of_view = fov_;
  this->range_msg_.max_range = this->parent_ray_sensor_->RangeMax();
  this->range_msg_.min_range = this->parent_ray_sensor_->RangeMin();

  // Init ROS
  if (ros::isInitialized())
  {
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(boost::bind(&InfraredRange::LoadThread, this));
  }
  else
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void InfraredRange::LoadThread()
{
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Range>(
        this->topic_name_, 1, boost::bind(&InfraredRange::RangeConnect, this),
        boost::bind(&InfraredRange::RangeDisconnect, this), ros::VoidPtr(), &this->range_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  // start custom queue for range
  this->callback_queue_thread_ = boost::thread(boost::bind(&InfraredRange::RangeQueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void InfraredRange::RangeConnect()
{
  this->range_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void InfraredRange::RangeDisconnect()
{
  this->range_connect_count_--;

  if (this->range_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the plugin
void InfraredRange::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time cur_time = this->world_->SimTime();
#else
    common::Time cur_time = this->world_->GetSimTime();
#endif
    if (cur_time < this->last_update_time_)
    {
      ROS_WARN_NAMED("range", "Negative sensor update time difference detected.");
      this->last_update_time_ = cur_time;
    }

    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      common::Time sensor_update_time = this->parent_sensor_->LastUpdateTime();
      this->PutRangeData(sensor_update_time);
      this->last_update_time_ = cur_time;
    }
  }
  else
  {
    ROS_INFO_NAMED("range", "gazebo_ros_range topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put range data to the interface
void InfraredRange::PutRangeData(common::Time &_updateTime)
{
  this->parent_ray_sensor_->SetActive(false);

  /***************************************************************/
  /*                                                             */
  /*  point scan from ray sensor                                 */
  /*                                                             */
  /***************************************************************/
  {
    boost::mutex::scoped_lock lock(this->lock_);
    // Add Frame Name
    this->range_msg_.header.frame_id = this->frame_name_;
    this->range_msg_.header.stamp.sec = _updateTime.sec;
    this->range_msg_.header.stamp.nsec = _updateTime.nsec;

    // find ray with minimal range
    range_msg_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();

    int num_ranges =
        parent_ray_sensor_->LaserShape()->GetSampleCount() * parent_ray_sensor_->LaserShape()->GetVerticalSampleCount();

    double ir_value = 0;

    for (int i = 0; i < num_ranges; ++i)
    {
      double ray = parent_ray_sensor_->LaserShape()->GetRange(i);
      int index = i;

      ir_value = ir_value + 0.1288 * pow(ray, -1.7887);
    }

    range_msg_.range = int(ir_value + this->GaussianKernel(0, gaussian_noise_));

    this->parent_ray_sensor_->SetActive(true);

    // send data out via ros message
    if (this->range_connect_count_ > 0 && this->topic_name_ != "")
      this->pub_.publish(this->range_msg_);
  }
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double InfraredRange::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put range data to the interface
void InfraredRange::RangeQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->range_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}  // namespace gazebo
