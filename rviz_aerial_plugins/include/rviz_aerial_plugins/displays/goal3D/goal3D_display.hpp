// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef RVIZ_AERIAL_PLUGINS__DISPLAYS__GOAL__3D_DISPLAY_HPP_
#define RVIZ_AERIAL_PLUGINS__DISPLAYS__GOAL__3D_DISPLAY_HPP_

#include <memory>

#ifndef Q_MOC_RUN

#include "rviz_common/panel.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include "rviz_aerial_plugins/visibility_control.hpp"

#include "interactive_markers/interactive_marker_server.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "px4_msgs/msg/vehicle_gps_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/position_setpoint.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

#include "visualization_msgs/msg/interactive_marker.hpp"
#include <interactive_markers/menu_handler.hpp>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/utils.h"

#include <QLineEdit>
#include <QPushButton>
#include <QGridLayout>
#include <QLabel>

#include <math.h>
#include <eigen3/Eigen/Dense>

#endif

namespace rviz_aerial_plugins
{

namespace displays
{

  class GeodeticConverter
  {
   public:
    GeodeticConverter(double latitude = 0, double longitude = 0, double altitude = 0)
      : home_latitude_(latitude), home_longitude_(longitude)
    {

      kSemimajorAxis = 6378137;
      kSemiminorAxis = 6356752.3142;
      kFirstEccentricitySquared = 6.69437999014 * 0.001;
      kSecondEccentricitySquared = 6.73949674228 * 0.001;
      kFlattening = 1 / 298.257223563;


      // Save NED origin
      home_latitude_rad_ = deg2Rad(latitude);
      home_longitude_rad_ = deg2Rad(longitude);
      home_altitude_ = altitude;

      // Compute ECEF of NED origin
      geodetic2Ecef(latitude, longitude, altitude, home_ecef_x_, home_ecef_y_, home_ecef_z_);

      // Compute ECEF to NED and NED to ECEF matrices
      double phiP = atan2(home_ecef_z_, sqrt(pow(home_ecef_x_, 2) + pow(home_ecef_y_, 2)));

      ecef_to_ned_matrix_ = nRe(phiP, home_longitude_rad_);
      ned_to_ecef_matrix_ = nRe(home_latitude_rad_, home_longitude_rad_).transpose();
    }

    void getHome(double* latitude, double* longitude, double* altitude)
    {
      *latitude = home_latitude_;
      *longitude = home_longitude_;
      *altitude = home_altitude_;
    }

    void geodetic2Ecef(const double latitude, const double longitude, const double altitude,
                      double& x, double& y, double& z)
    {
      // Convert geodetic coordinates to ECEF.
      // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
      double lat_rad = deg2Rad(latitude);
      double lon_rad = deg2Rad(longitude);
      double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
      x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
      y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
      z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
    }

    void ecef2Geodetic(const double x, const double y, const double z,
                      double& latitude, double& longitude, double& altitude)
    {
      // Convert ECEF coordinates to geodetic coordinates.
      // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
      // to geodetic coordinates," IEEE Transactions on Aerospace and
      // Electronic Systems, vol. 30, pp. 957-961, 1994.

      double r = sqrt(x * x + y * y);
      double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
      double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
      double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
      double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
      double S = cbrt(1 + C + sqrt(C * C + 2 * C));
      double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
      double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
      double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
          + sqrt(
              0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
                  - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
      double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
      double V = sqrt(
          pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
      double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);
      altitude = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
      latitude = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
      longitude = rad2Deg(atan2(y, x));
    }

private:
      // Geodetic system parameters
    double kSemimajorAxis;
    double kSemiminorAxis;
    double kFirstEccentricitySquared;
    double kSecondEccentricitySquared;
    double kFlattening;

    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;

    inline Matrix3x3d nRe(const double lat_radians, const double lon_radians)
    {
      const double sLat = sin(lat_radians);
      const double sLon = sin(lon_radians);
      const double cLat = cos(lat_radians);
      const double cLon = cos(lon_radians);

      Matrix3x3d ret;
      ret(0, 0) = -sLat * cLon;
      ret(0, 1) = -sLat * sLon;
      ret(0, 2) = cLat;
      ret(1, 0) = -sLon;
      ret(1, 1) = cLon;
      ret(1, 2) = 0.0;
      ret(2, 0) = cLat * cLon;
      ret(2, 1) = cLat * sLon;
      ret(2, 2) = sLat;

      return ret;
    }

    inline double rad2Deg(const double radians)
    {
      return (radians / M_PI) * 180.0;
    }

    inline double deg2Rad(const double degrees)
    {
      return (degrees / 180.0) * M_PI;
    }

    double home_latitude_rad_, home_latitude_;
    double home_longitude_rad_, home_longitude_;
    double home_altitude_;

    double home_ecef_x_;
    double home_ecef_y_;
    double home_ecef_z_;

    Matrix3x3d ecef_to_ned_matrix_;
    Matrix3x3d ned_to_ecef_matrix_;

  };


class RVIZ_AERIAL_PLUGINS_PUBLIC Goal3DDisplay:
  public rviz_common::Panel
{
Q_OBJECT
public:

  Goal3DDisplay(QWidget* parent = 0);
  ~Goal3DDisplay() override;

  void onInitialize() override;

private slots:
  void on_click_subscribeButton();
  void on_click_armButton();
  void valueChangedInterface();
  void on_click_takeoffButton();
  void on_click_position_setpointButton();

private:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  void subcribe2topics();

  void makeQuadrocopterMarker(const tf2::Vector3& position);
  void
  processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  visualization_msgs::msg::InteractiveMarkerControl &
    makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg);

  visualization_msgs::msg::Marker
    makeBox(const visualization_msgs::msg::InteractiveMarker & msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr vehicle_gps_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_vehicle_command_;
  rclcpp::Publisher<px4_msgs::msg::PositionSetpoint>::SharedPtr publisher_setpoint_;

  QLineEdit* namespace_;
  QLabel* label_arming_state_;
  QLabel* label_name_arming_state_;
  int arming_state_;

  QLabel* label_vehicle_type_;
  QLabel* label_name_vehicle_type_;

  std::string vehicle_gps_position_name_;
  std::string vehicle_command_name_;
  std::string vehicle_status_name_;
  std::string attitude_topic_name_;
  std::string odometry_topic_name_;
  std::string position_setpoint_topic_name_;
  std::string vehicle_land_detected_topic_name_;

  QPushButton* button_arm_;
  QPushButton* button_takeoff_;
  QPushButton* button_position_setpoint_;
  void vehicle_status_callback(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg);

  float latitude_;
  float longitude_;
  float altitude_;
  float heading_;
  float altitude_rel_;
  bool flying_;

  std::shared_ptr<GeodeticConverter> geodetic_converter;

signals:
      void valueChangedInterface_signal();
};

} // namespace displays

} // namespace rviz_hrim_plugin_

#endif // RVIZ_AERIAL_PLUGINS__DISPLAYS__GOAL__3D_DISPLAY_HPP_
