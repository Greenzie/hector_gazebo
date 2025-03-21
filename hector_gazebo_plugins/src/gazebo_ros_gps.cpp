//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_gazebo_plugins/gazebo_ros_gps.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/SphericalCoordinates.hh>

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;
namespace gazebo {

GazeboRosGps::GazeboRosGps()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGps::~GazeboRosGps()
{
  updateTimer.Disconnect(updateConnection);

  dynamic_reconfigure_server_position_.reset();
  dynamic_reconfigure_server_velocity_.reset();
  dynamic_reconfigure_server_status_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("GazeboRosGps plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = "/world";
  fix_topic_ = "fix";
  velocity_topic_ = "fix_velocity";

  reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
  reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
  reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

  if (_sdf->HasElement("useWorldSphericalCoordinates"))
  {
    bool use_world_coords = false;
    if (_sdf->GetElement("useWorldSphericalCoordinates")->GetValue()->Get(use_world_coords) && use_world_coords)
    {
      common::SphericalCoordinatesPtr spherical_coords = world->SphericalCoords();
      reference_latitude_ = spherical_coords->LatitudeReference().Degree();
      reference_longitude_ = spherical_coords->LongitudeReference().Degree();
      // SDF specifies heading counter-clockwise from east, but here it's measured clockwise from north
      reference_heading_ = (M_PI / 2.0) - spherical_coords->HeadingOffset().Radian();
      reference_altitude_ = spherical_coords->GetElevationReference();
    }
  }

  fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
  fix_.status.service = 0;

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    fix_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("velocityTopicName"))
    velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("referenceLatitude"))
    _sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);

  if (_sdf->HasElement("referenceLongitude"))
    _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);

  if (_sdf->HasElement("referenceHeading"))
    if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
      reference_heading_ *= M_PI/180.0;

  if (_sdf->HasElement("referenceAltitude"))
    _sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);

  if (_sdf->HasElement("status")) {
    int status = fix_.status.status;
    if (_sdf->GetElement("status")->GetValue()->Get(status))
      fix_.status.status = static_cast<sensor_msgs::NavSatStatus::_status_type>(status);
  }

  if (_sdf->HasElement("service")) {
    unsigned int service = fix_.status.service;
    if (_sdf->GetElement("service")->GetValue()->Get(service))
      fix_.status.service = static_cast<sensor_msgs::NavSatStatus::_service_type>(service);
  }

  // Dropouts
  if (_sdf->HasElement("delayedStartMinS")) {
    _sdf->GetElement("delayedStartMinS")->GetValue()->Get(delayed_start_min_s_);
  }
  if (_sdf->HasElement("delayedStartMaxS")) {
    _sdf->GetElement("delayedStartMaxS")->GetValue()->Get(delayed_start_max_s_);
  }
  if (_sdf->HasElement("dropoutLengthMinS")) {
    _sdf->GetElement("dropoutLengthMinS")->GetValue()->Get(dropout_length_min_s_);
  }
  if (_sdf->HasElement("dropoutLengthMaxS")) {
    _sdf->GetElement("dropoutLengthMaxS")->GetValue()->Get(dropout_length_max_s_);
  }
  if (_sdf->HasElement("dropoutDelayMinS")) {
    _sdf->GetElement("dropoutDelayMinS")->GetValue()->Get(dropout_delay_min_s_);
  }
  if (_sdf->HasElement("dropoutDelayMaxS")) {
    _sdf->GetElement("dropoutDelayMaxS")->GetValue()->Get(dropout_delay_max_s_);
  }
  
  int dropout_set = static_cast<int>(dropout_set_);
  if (_sdf->HasElement("dropoutSet")) {
    _sdf->GetElement("dropoutSet")->GetValue()->Get(dropout_set);
  }
  dropout_set_ = static_cast<DropoutSet>(dropout_set);

  // Discrete jumps
  if (_sdf->HasElement("jumpDelayMinS")) {
    _sdf->GetElement("jumpDelayMinS")->GetValue()->Get(jump_delay_min_s_);
  }
  if (_sdf->HasElement("jumpDelayMaxS")) {
    _sdf->GetElement("jumpDelayMaxS")->GetValue()->Get(jump_delay_max_s_);
  }
  if (_sdf->HasElement("jumpMinM")) {
    _sdf->GetElement("jumpMinM")->GetValue()->Get(jump_min_m_);
  }
  if (_sdf->HasElement("jumpMaxM")) {
    _sdf->GetElement("jumpMaxM")->GetValue()->Get(jump_max_m_);
  }
  if (_sdf->HasElement("jump3D")) {
    _sdf->GetElement("jump3D")->GetValue()->Get(jump_3d_);
  }
  
  int jump_set = static_cast<int>(jump_set_);
  if (_sdf->HasElement("jumpSet")) {
    _sdf->GetElement("jumpSet")->GetValue()->Get(jump_set);
  }
  jump_set_ = static_cast<DropoutSet>(jump_set);

  fix_.header.frame_id = frame_id_;
  velocity_.header.frame_id = frame_id_;

  position_error_model_.Load(_sdf);
  velocity_error_model_.Load(_sdf, "velocity");

  // calculate earth radii
  double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
  double prime_vertical_radius = equatorial_radius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
  radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);
  velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 10);

  set_geopose_srv_ = node_handle_->advertiseService(fix_topic_ + "/set_reference_geopose", &GazeboRosGps::setGeoposeCb, this);

  // setup dynamic_reconfigure servers
  dynamic_reconfigure_server_position_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/position")));
  dynamic_reconfigure_server_velocity_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/velocity")));
  dynamic_reconfigure_server_status_.reset(new dynamic_reconfigure::Server<GNSSConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/status")));
  dynamic_reconfigure_server_position_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &position_error_model_, _1, _2));
  dynamic_reconfigure_server_velocity_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &velocity_error_model_, _1, _2));
  dynamic_reconfigure_server_status_->setCallback(boost::bind(&GazeboRosGps::dynamicReconfigureCallback, this, _1, _2));

  Reset();

  // Set up the dropouts
  if(delayed_start_max_s_ > 0.00001)
  {
    ROS_DEBUG_STREAM("GPS has no fix");
    has_fix_ = false;  // Initialize to not fixed
    next_dropout_change_s_ = (delayed_start_max_s_ - delayed_start_min_s_) *
      (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + delayed_start_min_s_;
  } else if(dropout_set_ != DropoutSet::DROPOUT_NONE) {
    // Set the first time to dropout
    next_dropout_change_s_ = (dropout_delay_max_s_ - dropout_delay_min_s_) *
      (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + dropout_delay_min_s_;
  }
  // Set up the first jump
  if(jump_set_ != DropoutSet::DROPOUT_NONE) {
    // Set the first time to jump
    next_jump_change_s_ = (jump_delay_max_s_ - jump_delay_min_s_) *
      (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + jump_delay_min_s_;
  }
#if (GAZEBO_MAJOR_VERSION >= 8)
  last_dropout_change_ = world->SimTime();
#else
  last_dropout_change_ = world->GetSimTime();
#endif
  last_jump_change_ = last_dropout_change_;

  // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGps::Update, this));
}

bool GazeboRosGps::setGeoposeCb(hector_gazebo_plugins::SetReferenceGeoPose::Request& request,
                               hector_gazebo_plugins::SetReferenceGeoPose::Response&)
{
  reference_latitude_ = request.geo_pose.position.latitude;
  reference_longitude_ = request.geo_pose.position.longitude;
  tf::Quaternion q(request.geo_pose.orientation.x,
                   request.geo_pose.orientation.y,
                   request.geo_pose.orientation.z,
                   request.geo_pose.orientation.w);
  tf::Matrix3x3 m(q);
  tfScalar yaw, pitch, roll;
  m.getEulerYPR(yaw, pitch, roll);
  reference_heading_ = (M_PI / 2.0) - yaw;
  reference_altitude_ = request.geo_pose.position.altitude;

  Reset();

  return true;
}

void GazeboRosGps::Reset()
{
  updateTimer.Reset();
  position_error_model_.reset();
  velocity_error_model_.reset();
}

void GazeboRosGps::dynamicReconfigureCallback(GazeboRosGps::GNSSConfig &config, uint32_t level)
{
  using sensor_msgs::NavSatStatus;
  if (level == 1) {
    if (!config.STATUS_FIX) {
      fix_.status.status = NavSatStatus::STATUS_NO_FIX;
    } else {
      fix_.status.status = (config.STATUS_SBAS_FIX ? NavSatStatus::STATUS_SBAS_FIX : 0) |
                           (config.STATUS_GBAS_FIX ? NavSatStatus::STATUS_GBAS_FIX : 0);
    }
    fix_.status.service = (config.SERVICE_GPS     ? NavSatStatus::SERVICE_GPS : 0) |
                          (config.SERVICE_GLONASS ? NavSatStatus::SERVICE_GLONASS : 0) |
                          (config.SERVICE_COMPASS ? NavSatStatus::SERVICE_COMPASS : 0) |
                          (config.SERVICE_GALILEO ? NavSatStatus::SERVICE_GALILEO : 0);
  } else {
    config.STATUS_FIX      = (fix_.status.status != NavSatStatus::STATUS_NO_FIX);
    config.STATUS_SBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_SBAS_FIX);
    config.STATUS_GBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_GBAS_FIX);
    config.SERVICE_GPS     = (fix_.status.service & NavSatStatus::SERVICE_GPS);
    config.SERVICE_GLONASS = (fix_.status.service & NavSatStatus::SERVICE_GLONASS);
    config.SERVICE_COMPASS = (fix_.status.service & NavSatStatus::SERVICE_COMPASS);
    config.SERVICE_GALILEO = (fix_.status.service & NavSatStatus::SERVICE_GALILEO);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGps::Update()
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world->SimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  ignition::math::Pose3d pose = link->WorldPose();

  ignition::math::Vector3d velocity = velocity_error_model_(link->WorldLinearVel(), dt);
  ignition::math::Vector3d position = position_error_model_(pose.Pos(), dt);
#else
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  math::Pose pose = link->GetWorldPose();

  gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
  gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);
#endif

  // An offset error in the velocity is integrated into the position error for the next timestep.
  // Note: Usually GNSS receivers have almost no drift in the velocity signal.
  position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());

  fix_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  velocity_.header.stamp = fix_.header.stamp;

#if (GAZEBO_MAJOR_VERSION >= 8)
  fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.X() + sin(reference_heading_) * position.Y()) / radius_north_ * 180.0/M_PI;
  fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.X() + cos(reference_heading_) * position.Y()) / radius_east_  * 180.0/M_PI;
  fix_.altitude  = reference_altitude_  + position.Z();
  velocity_.vector.x =  cos(reference_heading_) * velocity.X() + sin(reference_heading_) * velocity.Y();
  velocity_.vector.y = -sin(reference_heading_) * velocity.X() + cos(reference_heading_) * velocity.Y();
  velocity_.vector.z = velocity.Z();

  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_.position_covariance[0] = position_error_model_.drift.X()*position_error_model_.drift.X() + position_error_model_.gaussian_noise.X()*position_error_model_.gaussian_noise.X();
  fix_.position_covariance[4] = position_error_model_.drift.Y()*position_error_model_.drift.Y() + position_error_model_.gaussian_noise.Y()*position_error_model_.gaussian_noise.Y();
  fix_.position_covariance[8] = position_error_model_.drift.Z()*position_error_model_.drift.Z() + position_error_model_.gaussian_noise.Z()*position_error_model_.gaussian_noise.Z();
#else
  fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.x + sin(reference_heading_) * position.y) / radius_north_ * 180.0/M_PI;
  fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.x + cos(reference_heading_) * position.y) / radius_east_  * 180.0/M_PI;
  fix_.altitude  = reference_altitude_  + position.z;
  velocity_.vector.x =  cos(reference_heading_) * velocity.x + sin(reference_heading_) * velocity.y;
  velocity_.vector.y = -sin(reference_heading_) * velocity.x + cos(reference_heading_) * velocity.y;
  velocity_.vector.z = velocity.z;

  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_.position_covariance[0] = position_error_model_.drift.x*position_error_model_.drift.x + position_error_model_.gaussian_noise.x*position_error_model_.gaussian_noise.x;
  fix_.position_covariance[4] = position_error_model_.drift.y*position_error_model_.drift.y + position_error_model_.gaussian_noise.y*position_error_model_.gaussian_noise.y;
  fix_.position_covariance[8] = position_error_model_.drift.z*position_error_model_.drift.z + position_error_model_.gaussian_noise.z*position_error_model_.gaussian_noise.z;
#endif

  // Handle whether there is a fix
  if(!has_fix_) {
    int8_t temp = fix_.status.status;
    fix_.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;  // Currently no fix
    fix_publisher_.publish(fix_);
    fix_.status.status = temp;  // Reset for next time
  } else {
    fix_publisher_.publish(fix_);
  }
  velocity_publisher_.publish(velocity_);

  // Handle dropout update
  if((dropout_set_ != DropoutSet::DROPOUT_NONE) || (!has_fix_))
  {
    // If dropouts are defined or a delay occurred (so no read currently)
    if((sim_time - last_dropout_change_).Double() > next_dropout_change_s_)
    {
      if(has_fix_)
      {
        // Going to no fix - need to see if we turn off any future dropouts
        if(dropout_set_ == DropoutSet::DROPOUT_ONCE)
        {
          // By doing the change this way, a delay and a single dropout still functions correctly.
          dropout_set_ = DropoutSet::DROPOUT_NONE;
        }
        // Already know a dropout should occur in this if statement
        // Calculate the length of the dropout
        next_dropout_change_s_ = (dropout_length_max_s_ - dropout_length_min_s_) *
          (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + dropout_length_min_s_;
      }
      else if(dropout_set_ != DropoutSet::DROPOUT_NONE)
      {
        // Currently is no fixand will have another dropout
        next_dropout_change_s_ = (dropout_delay_max_s_ - dropout_delay_min_s_) *
          (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + dropout_delay_min_s_;
      }
      // else: is no fix, but no more dropouts - just go to fix without anything further.

      // Make the change
      has_fix_ = !has_fix_;
      ROS_DEBUG_STREAM("GPS has fix: " << has_fix_);
      // Reset the last dropout change
      last_dropout_change_ = sim_time;
    }
  }

  // Handle jump update
  if((jump_set_ != DropoutSet::DROPOUT_NONE) &&
    ((sim_time - last_jump_change_).Double() > next_jump_change_s_))
  {
    // Making jump - check if future jumps required
    if(jump_set_ == DropoutSet::DROPOUT_ONCE)
    {
      jump_set_ = DropoutSet::DROPOUT_NONE;
    }
    // Make the jump
    uint8_t max_index = (jump_3d_) ? 3 : 2;  // Handle 2D or 3D jumps
    for(uint8_t index = 0; index < max_index; index++) {
      position_error_model_.offset[index] +=
        ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2.0 - 1.0) *
        (jump_max_m_ - jump_min_m_) + jump_min_m_;
    }
    ROS_DEBUG_STREAM("GPS jumped");

    // Calculate the length of the dropout
    next_jump_change_s_ = (jump_delay_max_s_ - jump_delay_min_s_) *
      (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) + jump_delay_min_s_;

    // Reset the last jump change
    last_jump_change_ = sim_time;
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGps)

} // namespace gazebo
