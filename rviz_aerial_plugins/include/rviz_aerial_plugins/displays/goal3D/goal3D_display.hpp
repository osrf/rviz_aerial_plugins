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
#include <string>
#include <rviz_aerial_plugins/utils/utils.hpp>

#ifndef Q_MOC_RUN

#include "rviz_common/panel.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include "rviz_aerial_plugins/visibility_control.hpp"

#include "interactive_markers/interactive_marker_server.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
#include <QComboBox>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/create_timer_ros.h"

#endif

using namespace std::chrono_literals;

namespace rviz_aerial_plugins
{

namespace displays
{

class RVIZ_AERIAL_PLUGINS_PUBLIC Goal3DDisplay:
  public rviz_common::Panel
{
Q_OBJECT
public:

  Goal3DDisplay(QWidget* parent = 0);
  ~Goal3DDisplay() override;

  void onInitialize() override;

private slots:
  void on_click_refresheButton();
  void on_click_armButton();
  void valueChangedInterface();
  void on_click_takeoffButton();
  void on_click_position_setpointButton();
  void on_changed_namespace(const QString &text);
private:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  void subcribe2topics();
  void add_namespaces_to_combobox();
  int getTargetSystem();

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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_stamped_;

  QComboBox* namespace_;
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
  std::string pose_stamped_name_;

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

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  rclcpp::TimerBase::SharedPtr timer_;

signals:
      void valueChangedInterface_signal();
};

} // namespace displays

} // namespace rviz_hrim_plugin_

#endif // RVIZ_AERIAL_PLUGINS__DISPLAYS__GOAL__3D_DISPLAY_HPP_
