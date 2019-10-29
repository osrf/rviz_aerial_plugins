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

#include "rviz_aerial_plugins/displays/goal3D/goal3D_display.hpp"

#include <memory>
#include <string>

namespace rviz_aerial_plugins
{

namespace displays
{

Goal3DDisplay::Goal3DDisplay(QWidget* parent):
  rviz_common::Panel(parent), rviz_ros_node_()
{
  vehicle_gps_position_name_ = "/iris_0/vehicle_gps_position";
  vehicle_command_name_ = "/iris_0/vehicle_command";
  vehicle_status_name_ = "/iris_0/vehicle_status";
  attitude_topic_name_ = "/iris_0/vehicle_attitude";
  odometry_topic_name_ = "/iris_0/vehicle_odometry";
  position_setpoint_topic_name_ = "/iris_0/position_setpoint";
  vehicle_land_detected_topic_name_ = "/iris_0/vehicle_land_detected";

  arming_state_ = 0;
  latitude_ = 0;
  longitude_ = 0;
  altitude_ = 0;
  heading_ = 0;
  flying_ = false;
}

void Goal3DDisplay::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("drone_goal", rviz_ros_node_.lock()->get_raw_node());

  namespace_ = new QLineEdit("");
  QPushButton* subcribe_button = new QPushButton("Subcribe");
  QGridLayout *grid = new QGridLayout;

  label_arming_state_ = new QLabel();
  label_name_arming_state_ = new QLabel("Arming state:");
  label_vehicle_type_ = new QLabel();
  label_name_vehicle_type_ = new QLabel("Vehicle type:");

  button_arm_ = new QPushButton("Arm");
  button_takeoff_ = new QPushButton("Takeoff");
  button_takeoff_->setEnabled(false);
  button_position_setpoint_ = new QPushButton("Go to point");

  grid->addWidget(namespace_, 0, 0);
  grid->addWidget(subcribe_button, 0, 1);
  grid->addWidget(label_name_arming_state_, 1, 0);
  grid->addWidget(label_arming_state_, 1, 1);
  grid->addWidget(label_name_vehicle_type_, 2, 0);
  grid->addWidget(label_vehicle_type_, 2, 1);
  grid->addWidget(button_arm_, 3, 0, 1, 2);
  grid->addWidget(button_takeoff_, 4, 0, 1, 2);
  grid->addWidget(button_position_setpoint_, 5, 0, 1, 2);

  setLayout(grid);
  QObject::connect(subcribe_button, SIGNAL(clicked()),this, SLOT(on_click_subscribeButton()));
  QObject::connect(button_arm_, SIGNAL(clicked()),this, SLOT(on_click_armButton()));
  QObject::connect(button_takeoff_, SIGNAL(clicked()),this, SLOT(on_click_takeoffButton()));
  QObject::connect(button_position_setpoint_, SIGNAL(clicked()),this, SLOT(on_click_position_setpointButton()));
  QObject::connect(this, SIGNAL(valueChangedInterface_signal()),this, SLOT(valueChangedInterface()));

  subcribe2topics();

  makeQuadrocopterMarker(tf2::Vector3(0, 0, 3));
  server_->applyChanges();
}

void Goal3DDisplay::on_click_position_setpointButton()
{

  double x, y, z;
  double lat, lon, alt;
  geodetic_converter->geodetic2Ecef(latitude_, longitude_, altitude_, x, y, z);

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(),
      "x %.5f\ty %.5f\tz %.5f", x, y, z);

  visualization_msgs::msg::InteractiveMarker int_marker;
  server_->get("quadrocopter", int_marker);

  x+=int_marker.pose.position.x;
  y+=int_marker.pose.position.y;

  geodetic_converter->ecef2Geodetic(x, y, z, lat, lon, alt);
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(),
      "lat %.5f\tlon %.5f", lat, lon);

  auto time_node = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();
  px4_msgs::msg::VehicleCommand msg_vehicle_command;
  msg_vehicle_command.timestamp = time_node.nanoseconds()/1000 - 3000000;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
  msg_vehicle_command.param1 = -1;
  msg_vehicle_command.param2 = 1;
  msg_vehicle_command.param3 = 0;
  msg_vehicle_command.param4 = heading_;
  msg_vehicle_command.param5 = lat;
  msg_vehicle_command.param6 = lon;
  msg_vehicle_command.param7 = 3.0;
  msg_vehicle_command.confirmation = 0;
  msg_vehicle_command.source_system = 255;
  msg_vehicle_command.target_system = 1;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  publisher_vehicle_command_->publish(msg_vehicle_command);
}

void Goal3DDisplay::on_click_takeoffButton()
{
  if(latitude_ != 0 && longitude_ != 0){

    RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "flying_ %d", flying_);

    if(!flying_){
      auto time_node = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();
      px4_msgs::msg::VehicleCommand msg_vehicle_command;
      msg_vehicle_command.timestamp = time_node.nanoseconds()/1000 - 3000000;
      msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
      msg_vehicle_command.param1 = 0.1;
      msg_vehicle_command.param2 = 0;
      msg_vehicle_command.param3 = 0;
      msg_vehicle_command.param4 = heading_;
      msg_vehicle_command.param5 = latitude_;
      msg_vehicle_command.param6 = longitude_;
      msg_vehicle_command.param7 = 3.0;
      msg_vehicle_command.confirmation = 1;
      msg_vehicle_command.source_system = 255;
      msg_vehicle_command.target_system = 1;
      msg_vehicle_command.target_component = 1;
      msg_vehicle_command.from_external = true;
      publisher_vehicle_command_->publish(msg_vehicle_command);
      RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "VEHICLE_CMD_NAV_TAKEOFF %.5f %.5f", latitude_, longitude_);
    }else{
      auto time_node = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();
      px4_msgs::msg::VehicleCommand msg_vehicle_command;
      msg_vehicle_command.timestamp = time_node.nanoseconds()/1000 - 3000000;
      msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
      msg_vehicle_command.param1 = 0.1;
      msg_vehicle_command.param2 = 0;
      msg_vehicle_command.param3 = 0;
      msg_vehicle_command.param4 = heading_;
      msg_vehicle_command.param5 = latitude_;
      msg_vehicle_command.param6 = longitude_;
      msg_vehicle_command.param7 = 3.0;
      msg_vehicle_command.confirmation = 1;
      msg_vehicle_command.source_system = 255;
      msg_vehicle_command.target_system = 1;
      msg_vehicle_command.target_component = 1;
      msg_vehicle_command.from_external = true;
      publisher_vehicle_command_->publish(msg_vehicle_command);
      RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "VEHICLE_CMD_NAV_TAKEOFF %5.f %5.f", latitude_, longitude_);
    }
  }
}

void Goal3DDisplay::on_click_armButton()
{
  auto time_node = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();

  std::ostringstream oss;
  oss << "arming_state " << arming_state_;

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), oss.str());

  if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
    px4_msgs::msg::VehicleCommand msg_vehicle_command;
    msg_vehicle_command.timestamp = time_node.nanoseconds()/1000 - 3000000;
    msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg_vehicle_command.param1 = 1;
    msg_vehicle_command.confirmation = 1;
    msg_vehicle_command.source_system = 255;
    msg_vehicle_command.target_system = 1;
    msg_vehicle_command.target_component = 1;
    msg_vehicle_command.from_external = true;
    publisher_vehicle_command_->publish(msg_vehicle_command);
    RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "ARMING_STATE_INIT");

  }else if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
  {
    px4_msgs::msg::VehicleCommand msg_vehicle_command;
    msg_vehicle_command.timestamp = time_node.nanoseconds()/1000 - 3000000;
    msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg_vehicle_command.param1 = 0;
    msg_vehicle_command.confirmation = 1;
    msg_vehicle_command.source_system = 255;
    msg_vehicle_command.target_system = 1;
    msg_vehicle_command.target_component = 1;
    msg_vehicle_command.from_external = true;
    publisher_vehicle_command_->publish(msg_vehicle_command);
    RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "VEHICLE_CMD_COMPONENT_ARM_DISARM");
  }
}

void Goal3DDisplay::valueChangedInterface()
{

  if(flying_){
    button_takeoff_->setText("Land");
  }else{
    button_takeoff_->setText("TakeOff");
  }

  if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_INIT){
    label_arming_state_->setText(QString("Init"));
  }else if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY){
    label_arming_state_->setText(QString("Standby"));
    button_arm_->setText(QString("Arm"));
    button_takeoff_->setEnabled(false);
  }else if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
    label_arming_state_->setText(QString("Armed"));
    button_arm_->setText(QString("Disarm"));
    button_takeoff_->setEnabled(true);
  }else if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY_ERROR){
    label_arming_state_->setText(QString("Standby error"));
  }else if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_SHUTDOWN){
    label_arming_state_->setText(QString("Shutdown"));
  }else if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_IN_AIR_RESTORE){
    label_arming_state_->setText(QString("In air restore"));
  }else if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_MAX){
    label_arming_state_->setText(QString("Max"));
  }
}

void Goal3DDisplay::vehicle_status_callback(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  if (msg->vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING){
    label_vehicle_type_->setText("Quadcopter");
  }
  else if (msg->vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING){
    label_vehicle_type_->setText("Fixed wing");
  }
  else if (msg->vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROVER){
    label_vehicle_type_->setText("Rover");
  }else{
    label_vehicle_type_->setText("Unknown");
  }

  if(arming_state_ != msg->arming_state){
    arming_state_ = msg->arming_state;
    // RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "arming_state %d", arming_state);
    emit valueChangedInterface_signal();
  }
}

void Goal3DDisplay::subcribe2topics()
{
  vehicle_gps_position_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<px4_msgs::msg::VehicleGpsPosition>(
        vehicle_gps_position_name_,
      10,
      [this](px4_msgs::msg::VehicleGpsPosition::ConstSharedPtr msg) {
        latitude_ = msg->lat*1E-7;
        longitude_ = msg->lon*1E-7;
        altitude_ = msg->alt*1E-3;

        if(geodetic_converter==nullptr){
          geodetic_converter = std::make_shared<GeodeticConverter>(latitude_, longitude_, msg->alt*1E-3);
        }

    });

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + vehicle_gps_position_name_);

  vehicle_attitude_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic_name_,
      10,
      [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg) {

        geometry_msgs::msg::Quaternion q;
        q.x = msg->q[1];
        q.y = msg->q[2];
        q.z = msg->q[3];
        q.w = msg->q[0];
        double yaw, pitch, roll;
        tf2::getEulerYPR(q, yaw, pitch, roll);
        heading_ = yaw;
    });
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + attitude_topic_name_);

  vehicle_odometry_sub_ = rviz_ros_node_.lock()->get_raw_node()->
        template create_subscription<px4_msgs::msg::VehicleOdometry>(
          odometry_topic_name_,
        10,
        [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
          altitude_rel_ = -msg->z;
      });
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + odometry_topic_name_);

  vehicle_land_detected_sub_ = rviz_ros_node_.lock()->get_raw_node()->
          template create_subscription<px4_msgs::msg::VehicleLandDetected>(
            vehicle_land_detected_topic_name_,
          10,
          [this](px4_msgs::msg::VehicleLandDetected::ConstSharedPtr msg) {
            if(msg->landed != !flying_){
              flying_ = !msg->landed;
              emit valueChangedInterface_signal();
            }
        });
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + vehicle_land_detected_topic_name_);

  vehicle_status_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_name_,
      10, std::bind(&Goal3DDisplay::vehicle_status_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + vehicle_status_name_);

  publisher_vehicle_command_ =
    rviz_ros_node_.lock()->get_raw_node()->
        create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_name_, 10);
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Publish to: " + vehicle_command_name_);

  publisher_setpoint_ =
    rviz_ros_node_.lock()->get_raw_node()->
      create_publisher<px4_msgs::msg::PositionSetpoint>(position_setpoint_topic_name_, 10);
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Publish to: " + position_setpoint_topic_name_);
}

void Goal3DDisplay::on_click_subscribeButton()
{

  std::string namespace_str(namespace_->text().toUtf8().constData());

  vehicle_gps_position_name_ = "/" + namespace_str + "/vehicle_gps_position";
  vehicle_command_name_ = "/" + namespace_str + "/vehicle_command";
  vehicle_status_name_ = "/" + namespace_str + "/vehicle_status";
  attitude_topic_name_ = "/" + namespace_str + "/vehicle_attitude";
  odometry_topic_name_ = "/" + namespace_str + "/vehicle_odometry";
  position_setpoint_topic_name_ = "/" + namespace_str + "/position_setpoint";

  vehicle_gps_position_sub_.reset();
  vehicle_status_sub_.reset();
  vehicle_attitude_sub_.reset();
  vehicle_land_detected_sub_.reset();
  vehicle_odometry_sub_.reset();
  publisher_vehicle_command_.reset();
  publisher_setpoint_.reset();

  subcribe2topics();
}

geometry_msgs::msg::TransformStamped toMsg(const tf2::Stamped<tf2::Transform>& in)
{
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = tf2_ros::toMsg(in.stamp_);
  out.header.frame_id = in.frame_id_;
  out.transform.translation.x = in.getOrigin().getX();
  out.transform.translation.y = in.getOrigin().getY();
  out.transform.translation.z = in.getOrigin().getZ();
  out.transform.rotation = toMsg(in.getRotation());
  return out;
}


void
Goal3DDisplay::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  std::ostringstream oss;
  oss << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid) {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      oss << ": button click" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), oss.str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      oss << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".";
      RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), oss.str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      oss << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nanosec << " nsec" ;
      RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), oss.str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
      oss << ": mouse down" << mouse_point_ss.str() << "." ;
      RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), oss.str());
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
      oss << ": mouse up" << mouse_point_ss.str() << "." ;
      RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), oss.str());
      break;
  }

  server_->applyChanges();
}

visualization_msgs::msg::Marker
Goal3DDisplay::makeBox(const visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::msg::InteractiveMarkerControl &
Goal3DDisplay::makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void Goal3DDisplay::makeQuadrocopterMarker(const tf2::Vector3 & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();
  int_marker.scale = 1;

  int_marker.name = "quadrocopter";
  int_marker.description = "Quadrocopter";

  makeBoxControl(int_marker);

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&Goal3DDisplay::processFeedback, this, std::placeholders::_1));
}

Goal3DDisplay::~Goal3DDisplay()
{
  server_.reset();
}

} // namespace displays

} // namespace rviz_aerial_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_aerial_plugins::displays::Goal3DDisplay, rviz_common::Panel)
