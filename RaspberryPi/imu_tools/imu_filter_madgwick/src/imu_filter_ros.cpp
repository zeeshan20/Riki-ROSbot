/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "imu_filter_madgwick/imu_filter_ros.h"
#include "imu_filter_madgwick/stateless_orientation.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ImuFilterRos::ImuFilterRos(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  initialized_(false)
{
  ROS_INFO ("Starting ImuFilter");

  // **** get paramters
  if (!nh_private_.getParam ("stateless", stateless_))
    stateless_ = false;
  if (!nh_private_.getParam ("use_mag", use_mag_))
   use_mag_ = true;
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
   publish_tf_ = true;
  if (!nh_private_.getParam ("reverse_tf", reverse_tf_))
   reverse_tf_ = false;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
   fixed_frame_ = "base_link";
  if (!nh_private_.getParam ("constant_dt", constant_dt_))
    constant_dt_ = 0.0;
  if (!nh_private_.getParam ("publish_debug_topics", publish_debug_topics_))
    publish_debug_topics_= false;
  if (!nh_private_.getParam ("use_magnetic_field_msg", use_magnetic_field_msg_))
    use_magnetic_field_msg_ = true;

  std::string world_frame;
  // Default should become false for next release
  if (!nh_private_.getParam ("world_frame", world_frame)) {
    world_frame = "nwu";
    ROS_WARN("Deprecation Warning: The parameter world_frame was not set, default is 'nwu'.");
    ROS_WARN("Starting with ROS Lunar, world_frame will default to 'enu'!");
  }

  if (world_frame == "ned") {
    world_frame_ = WorldFrame::NED;
  } else if (world_frame == "nwu"){
    world_frame_ = WorldFrame::NWU;
  } else if (world_frame == "enu"){
    world_frame_ = WorldFrame::ENU;
  } else {
    ROS_ERROR("The parameter world_frame was set to invalid value '%s'.", world_frame.c_str());
    ROS_ERROR("Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
    world_frame_ = WorldFrame::ENU;
  }
  filter_.setWorldFrame(world_frame_);

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    ROS_FATAL("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
    constant_dt_ = 0.0;
  }

  // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
  // otherwise, it will be constant
  if (constant_dt_ == 0.0)
    ROS_INFO("Using dt computed from message headers");
  else
    ROS_INFO("Using constant dt of %f sec", constant_dt_);

  // **** register dynamic reconfigure
  config_server_.reset(new FilterConfigServer(nh_private_));
  FilterConfigServer::CallbackType f = boost::bind(&ImuFilterRos::reconfigCallback, this, _1, _2);
  config_server_->setCallback(f);

  // **** register publishers
  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(
    ros::names::resolve("imu") + "/data", 5);

  if (publish_debug_topics_)
  {
    rpy_filtered_debug_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      ros::names::resolve("imu") + "/rpy/filtered", 5);

    rpy_raw_debug_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      ros::names::resolve("imu") + "/rpy/raw", 5);
  }

  // **** register subscribers
  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;

  imu_subscriber_.reset(new ImuSubscriber(
    nh_, ros::names::resolve("imu") + "/data_raw", queue_size));

  if (use_mag_)
  {
    if (use_magnetic_field_msg_)
    {
      mag_subscriber_.reset(new MagSubscriber(
        nh_, ros::names::resolve("imu") + "/mag", queue_size));
    }
    else
    {
      mag_subscriber_.reset(new MagSubscriber(
        nh_, ros::names::resolve("imu") + "/magnetic_field", queue_size));

      // Initialize the shim to support republishing Vector3Stamped messages from /mag as MagneticField
      // messages on the /magnetic_field topic.
      mag_republisher_ = nh_.advertise<MagMsg>(
        ros::names::resolve("imu") + "/magnetic_field", 5);
      vector_mag_subscriber_.reset(new MagVectorSubscriber(
        nh_, ros::names::resolve("imu") + "/mag", queue_size));
      vector_mag_subscriber_->registerCallback(&ImuFilterRos::imuMagVectorCallback, this);
    }

    sync_.reset(new Synchronizer(
      SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
    sync_->registerCallback(boost::bind(&ImuFilterRos::imuMagCallback, this, _1, _2));
  }
  else
  {
    imu_subscriber_->registerCallback(&ImuFilterRos::imuCallback, this);
  }

  check_topics_timer_ = nh_.createTimer(ros::Duration(10.0), &ImuFilterRos::checkTopicsTimerCallback, this);
}

ImuFilterRos::~ImuFilterRos()
{
  ROS_INFO ("Destroying ImuFilter");

  // Explicitly stop callbacks; they could execute after we're destroyed
  check_topics_timer_.stop();
}

void ImuFilterRos::imuCallback(const ImuMsg::ConstPtr& imu_msg_raw)
{
  boost::mutex::scoped_lock lock(mutex_);

  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration;

  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  if (!initialized_)
  {
    check_topics_timer_.stop();
    ROS_INFO("First IMU message received.");
  }

  if (!initialized_ || stateless_)
  {
    geometry_msgs::Quaternion init_q;
    StatelessOrientation::computeOrientation(world_frame_, lin_acc, init_q);
    filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

    // initialize time
    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
  {
    dt = (time - last_time_).toSec();
    if (time.isZero())
      ROS_WARN_STREAM_THROTTLE(5.0, "The IMU message time stamp is zero, and the parameter constant_dt is not set!" <<
                                    " The filter will not update the orientation.");
  }

  last_time_ = time;

  if (!stateless_)
    filter_.madgwickAHRSupdateIMU(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);
}

void ImuFilterRos::imuMagCallback(
  const ImuMsg::ConstPtr& imu_msg_raw,
  const MagMsg::ConstPtr& mag_msg)
{
  boost::mutex::scoped_lock lock(mutex_);

  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
  const geometry_msgs::Vector3& mag_fld = mag_msg->magnetic_field;

  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  /*** Compensate for hard iron ***/
  geometry_msgs::Vector3 mag_compensated;
  mag_compensated.x = mag_fld.x - mag_bias_.x;
  mag_compensated.y = mag_fld.y - mag_bias_.y;
  mag_compensated.z = mag_fld.z - mag_bias_.z;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  if (!initialized_)
  {
    check_topics_timer_.stop();
    ROS_INFO("First pair of IMU and magnetometer messages received.");
  }

  if (!initialized_ || stateless_)
  {
    // wait for mag message without NaN / inf
    if(!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) || !std::isfinite(mag_fld.z))
    {
      return;
    }

    geometry_msgs::Quaternion init_q;
    StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, init_q);
    filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
  {
    dt = (time - last_time_).toSec();
    if (time.isZero())
      ROS_WARN_STREAM_THROTTLE(5.0, "The IMU message time stamp is zero, and the parameter constant_dt is not set!" <<
                                    " The filter will not update the orientation.");
  }

  last_time_ = time;

  if (!stateless_)
    filter_.madgwickAHRSupdate(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      mag_compensated.x, mag_compensated.y, mag_compensated.z,
      dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);

  if(publish_debug_topics_)
  {
    geometry_msgs::Quaternion orientation;
    if (StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, orientation))
    {
      tf2::Matrix3x3(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw, 0);
      publishRawMsg(time, roll, pitch, yaw);
    }
  }
}

void ImuFilterRos::publishTransform(const ImuMsg::ConstPtr& imu_msg_raw)
{
  double q0,q1,q2,q3;
  filter_.getOrientation(q0,q1,q2,q3);
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = imu_msg_raw->header.stamp;
  if (reverse_tf_)
  {
    transform.header.frame_id = imu_frame_;
    transform.child_frame_id = fixed_frame_;
    transform.transform.rotation.w = q0;
    transform.transform.rotation.x = -q1;
    transform.transform.rotation.y = -q2;
    transform.transform.rotation.z = -q3;
  }
  else {
    transform.header.frame_id = fixed_frame_;
    transform.child_frame_id = imu_frame_;
    transform.transform.rotation.w = q0;
    transform.transform.rotation.x = q1;
    transform.transform.rotation.y = q2;
    transform.transform.rotation.z = q3;
  }
  tf_broadcaster_.sendTransform(transform);

}

void ImuFilterRos::publishFilteredMsg(const ImuMsg::ConstPtr& imu_msg_raw)
{
  double q0,q1,q2,q3;
  filter_.getOrientation(q0,q1,q2,q3);

  // create and publish filtered IMU message
  boost::shared_ptr<ImuMsg> imu_msg =
    boost::make_shared<ImuMsg>(*imu_msg_raw);

  imu_msg->orientation.w = q0;
  imu_msg->orientation.x = q1;
  imu_msg->orientation.y = q2;
  imu_msg->orientation.z = q3;

  imu_msg->orientation_covariance[0] = orientation_variance_;
  imu_msg->orientation_covariance[1] = 0.0;
  imu_msg->orientation_covariance[2] = 0.0;
  imu_msg->orientation_covariance[3] = 0.0;
  imu_msg->orientation_covariance[4] = orientation_variance_;
  imu_msg->orientation_covariance[5] = 0.0;
  imu_msg->orientation_covariance[6] = 0.0;
  imu_msg->orientation_covariance[7] = 0.0;
  imu_msg->orientation_covariance[8] = orientation_variance_;

  imu_publisher_.publish(imu_msg);

  if(publish_debug_topics_)
  {
    geometry_msgs::Vector3Stamped rpy;
    tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

    rpy.header = imu_msg_raw->header;
    rpy_filtered_debug_publisher_.publish(rpy);
  }
}

void ImuFilterRos::publishRawMsg(const ros::Time& t,
  float roll, float pitch, float yaw)
{
  geometry_msgs::Vector3Stamped rpy;
  rpy.vector.x = roll;
  rpy.vector.y = pitch;
  rpy.vector.z = yaw ;
  rpy.header.stamp = t;
  rpy.header.frame_id = imu_frame_;
  rpy_raw_debug_publisher_.publish(rpy);
}


void ImuFilterRos::reconfigCallback(FilterConfig& config, uint32_t level)
{
  double gain, zeta;
  boost::mutex::scoped_lock lock(mutex_);
  gain = config.gain;
  zeta = config.zeta;
  filter_.setAlgorithmGain(gain);
  filter_.setDriftBiasGain(zeta);
  ROS_INFO("Imu filter gain set to %f", gain);
  ROS_INFO("Gyro drift bias set to %f", zeta);
  mag_bias_.x = config.mag_bias_x;
  mag_bias_.y = config.mag_bias_y;
  mag_bias_.z = config.mag_bias_z;
  orientation_variance_ = config.orientation_stddev * config.orientation_stddev;
  ROS_INFO("Magnetometer bias values: %f %f %f", mag_bias_.x, mag_bias_.y, mag_bias_.z);
}

void ImuFilterRos::imuMagVectorCallback(const MagVectorMsg::ConstPtr& mag_vector_msg)
{
  MagMsg mag_msg;
  mag_msg.header = mag_vector_msg->header;
  mag_msg.magnetic_field = mag_vector_msg->vector;
  // leaving mag_msg.magnetic_field_covariance set to all zeros (= "covariance unknown")
  mag_republisher_.publish(mag_msg);
}

void ImuFilterRos::checkTopicsTimerCallback(const ros::TimerEvent&)
{
  if (use_mag_)
    ROS_WARN_STREAM("Still waiting for data on topics " << ros::names::resolve("imu") << "/data_raw"
                    << " and " << ros::names::resolve("imu") << "/mag" << "...");
  else
    ROS_WARN_STREAM("Still waiting for data on topic " << ros::names::resolve("imu") << "/data_raw" << "...");
}
