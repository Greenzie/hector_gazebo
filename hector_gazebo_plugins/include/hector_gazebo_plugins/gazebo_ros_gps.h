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

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hector_gazebo_plugins/SetReferenceGeoPose.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>
#include <tf/tf.h>

#include <dynamic_reconfigure/server.h>
#include <hector_gazebo_plugins/GNSSConfig.h>

namespace gazebo
{

class GazeboRosGps : public ModelPlugin
{
public:
  enum DropoutSet
  {
      DROPOUT_NONE = 0,
      DROPOUT_ONCE = 1,
      DROPOUT_REPEAT = 2
  };

  GazeboRosGps();
  virtual ~GazeboRosGps();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

  typedef hector_gazebo_plugins::GNSSConfig GNSSConfig;
  void dynamicReconfigureCallback(GNSSConfig &config, uint32_t level);

private:

  /// \brief Callback for the set_spherical_coordinates service
  bool setGeoposeCb(hector_gazebo_plugins::SetReferenceGeoPose::Request& request,
                    hector_gazebo_plugins::SetReferenceGeoPose::Response&);

  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::Publisher fix_publisher_;
  ros::Publisher velocity_publisher_;

  ros::ServiceServer set_geopose_srv_;

  sensor_msgs::NavSatFix fix_;
  geometry_msgs::Vector3Stamped velocity_;

  std::string namespace_;
  std::string link_name_;
  std::string frame_id_;
  std::string fix_topic_;
  std::string velocity_topic_;

  double reference_latitude_;
  double reference_longitude_;
  double reference_heading_;
  double reference_altitude_;

  double radius_north_;
  double radius_east_;

  // Optional delay in seconds before first fix
  double delayed_start_min_s_{0.0};
  double delayed_start_max_s_{0.0};
  // Optional dropout parameters (loss of fix)
  double dropout_length_min_s_{0.0};
  double dropout_length_max_s_{0.0};
  double dropout_delay_min_s_{0.0};
  double dropout_delay_max_s_{0.0};
  // Sets whether the dropout occurs (0=no, 1=once, 2=repeating)
  DropoutSet dropout_set_{DropoutSet::DROPOUT_NONE};

  // Discrete jumps
  double jump_delay_min_s_{0.0};
  double jump_delay_max_s_{0.0};
  double jump_max_m_{0.0};  // In each dimension
  double jump_min_m_{0.0};  // In each dimension
  bool jump_3d_{false};  // Whether can jump 3D
  DropoutSet jump_set_{DropoutSet::DROPOUT_NONE};

  // Handles dropout/delay control
  bool has_fix_{true};  // Default unless in a dropout or delayed start
  double next_dropout_change_s_{0.0};  // Time until next dropout change in seconds from the last change
  common::Time last_dropout_change_{0.0};  // Time of the last dropout change in seconds
  double next_jump_change_s_{0.0};  // Time until next jump change in seconds from the last change
  common::Time last_jump_change_{0.0};  // Time of the last jump change in seconds

  SensorModel3 position_error_model_;
  SensorModel3 velocity_error_model_;

  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;

  boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_position_, dynamic_reconfigure_server_velocity_;
  boost::shared_ptr<dynamic_reconfigure::Server<GNSSConfig> > dynamic_reconfigure_server_status_;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
