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
#include "rviz_common/load_resource.hpp"

namespace rviz_aerial_plugins
{

namespace displays
{

Goal3DDisplay::Goal3DDisplay(QWidget* parent):
  rviz_common::Panel(parent), rviz_ros_node_()

{
  vehicle_gps_position_name_ = "/iris_0/gps";
  attitude_topic_name_ = "/iris_0/attitude";
  pose_stamped_name_ = "/iris_0/command_pose";
  set_flight_mode_name_ = "/iris_0/set_flight_mode";
  flight_mode_name_ = "/iris_0/flight_mode";
  vehicle_status_name_ = "/iris_0/status";

  latitude_ = 0;
  longitude_ = 0;
  altitude_ = 0;
  heading_ = 0;
  flying_ = false;
}

void Goal3DDisplay::add_namespaces_to_combobox()
{
  auto names_and_namespaces = rviz_ros_node_.lock()->get_raw_node()->get_node_names();

  std::set<std::string> namespaces = get_namespaces(names_and_namespaces);

  namespace_->blockSignals(true);
  namespace_->clear();
  for(auto n: namespaces){
    namespace_->addItem(QString(n.c_str()));
  }
  namespace_->blockSignals(false);
}

int Goal3DDisplay::getTargetSystem()
{
  int result = -1;
  std::string current_namespace(namespace_->currentText().toUtf8().constData());

  std::vector<std::string> namespace_tokens = split (current_namespace, '_');
  if(namespace_tokens.size() > 1){
    result = atoi(namespace_tokens[1].c_str());
  }
  return result + 1;
}

void Goal3DDisplay::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("drone_goal", rviz_ros_node_.lock()->get_raw_node());

  namespace_ = new QComboBox();
  add_namespaces_to_combobox();
  QPushButton* refresh_button = new QPushButton("Refresh");
  refresh_button->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/Refresh.png"));

  QGridLayout *grid = new QGridLayout;

  label_arming_state_ = new QLabel();
  label_name_arming_state_ = new QLabel("State:");
  label_vehicle_type_ = new QLabel();
  label_name_vehicle_type_ = new QLabel("Type:");

  button_arm_ = new QPushButton("Arm");
  button_arm_->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/PowerOn.png"));
  button_takeoff_ = new QPushButton("Takeoff");
  button_takeoff_->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/Takeoff.png"));
  button_position_setpoint_ = new QPushButton("Go to");
  button_position_setpoint_->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/Goal3D.png"));
  QPushButton* button_rtl = new QPushButton("RTL");

  grid->addWidget(namespace_, 0, 0);
  grid->addWidget(refresh_button, 0, 1);
  grid->addWidget(label_name_arming_state_, 1, 0);
  grid->addWidget(label_arming_state_, 1, 1);
  grid->addWidget(label_name_vehicle_type_, 2, 0);
  grid->addWidget(label_vehicle_type_, 2, 1);
  grid->addWidget(button_arm_, 4, 0, 1, 2);
  grid->addWidget(button_takeoff_, 5, 0, 1, 2);
  grid->addWidget(button_position_setpoint_, 6, 0, 1, 2);
  grid->addWidget(button_rtl, 7, 0, 1, 2);

  setLayout(grid);
  QObject::connect(namespace_, SIGNAL(currentIndexChanged(QString)),this, SLOT(on_changed_namespace(QString)));
  QObject::connect(refresh_button, SIGNAL(clicked()),this, SLOT(on_click_refresheButton()));
  QObject::connect(button_arm_, SIGNAL(clicked()),this, SLOT(on_click_armButton()));
  QObject::connect(button_rtl, SIGNAL(clicked()),this, SLOT(on_click_rltButton()));
  QObject::connect(button_takeoff_, SIGNAL(clicked()),this, SLOT(on_click_takeoffButton()));
  QObject::connect(button_position_setpoint_, SIGNAL(clicked()),this, SLOT(on_click_position_setpointButton()));
  QObject::connect(this, SIGNAL(valueChangedInterface_signal()),this, SLOT(valueChangedInterface()));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rviz_ros_node_.lock()->get_raw_node());

  auto clock_ros = rviz_ros_node_.lock()->get_raw_node()->get_clock();
  buffer_ = std::make_shared<tf2_ros::Buffer>(clock_ros);
  buffer_->setUsingDedicatedThread(true);
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    rviz_ros_node_.lock()->get_raw_node()->get_node_base_interface(),
    rviz_ros_node_.lock()->get_raw_node()->get_node_timers_interface());
  buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

  auto timer_callback =
     [this]() -> void {
       visualization_msgs::msg::InteractiveMarker int_marker;
       server_->get("quadrocopter", int_marker);

       auto odom_tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
       odom_tf_msg->header.frame_id = "map";
       odom_tf_msg->child_frame_id = "quadcopter_goal";
       // Stuff and publish /tf
       odom_tf_msg->header.stamp = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();
       odom_tf_msg->transform.translation.x = int_marker.pose.position.x;
       odom_tf_msg->transform.translation.y = int_marker.pose.position.y;
       odom_tf_msg->transform.translation.z = int_marker.pose.position.z;
       odom_tf_msg->transform.rotation.x = int_marker.pose.orientation.x;
       odom_tf_msg->transform.rotation.y = int_marker.pose.orientation.y;
       odom_tf_msg->transform.rotation.z = int_marker.pose.orientation.z;
       odom_tf_msg->transform.rotation.w = int_marker.pose.orientation.w;

       tf_broadcaster_->sendTransform(*odom_tf_msg);

     };
  timer_ = rviz_ros_node_.lock()->get_raw_node()->create_wall_timer(10ms, timer_callback);

  subcribe2topics();

  makeQuadrocopterMarker(tf2::Vector3(0, 0, 3));
  server_->applyChanges();
}

void Goal3DDisplay::on_click_rltButton()
{
  if (!set_flight_mode_client_->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(rviz_ros_node_.lock()->get_raw_node()->get_logger(),
                 "Action server not available after waiting");
    return;
  }

  // Populate a goal
  auto goal_msg =  proposed_aerial_msgs::action::SetFlightMode::Goal();
  goal_msg.goal.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_RTL;

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "RTL request");
  auto send_goal_options = rclcpp_action::Client<proposed_aerial_msgs::action::SetFlightMode>::SendGoalOptions();
  send_goal_options.goal_response_callback =
     std::bind(&Goal3DDisplay::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
     std::bind(&Goal3DDisplay::result_callback, this, std::placeholders::_1);
   auto goal_handle_future = set_flight_mode_client_->async_send_goal(goal_msg, send_goal_options);
}

void Goal3DDisplay::on_click_position_setpointButton()
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  server_->get("quadrocopter", int_marker);

  std::string namespace_combobox = std::string(namespace_->currentText().toUtf8().constData());

  if(namespace_combobox.empty())
    return;

  std::string str_test = std::string(namespace_->currentText().toUtf8().constData()) + "/odom";
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "map %s", str_test.c_str());

  geometry_msgs::msg::TransformStamped transform_callback_result = buffer_->lookupTransform("map", str_test, tf2::TimePoint());

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();

  msg.pose.position.x = int_marker.pose.position.x - transform_callback_result.transform.translation.x;
  msg.pose.position.y = int_marker.pose.position.y - transform_callback_result.transform.translation.y;
  msg.pose.position.z = int_marker.pose.position.z;

  msg.pose.orientation.x = int_marker.pose.orientation.x;
  msg.pose.orientation.y = int_marker.pose.orientation.y;
  msg.pose.orientation.z = int_marker.pose.orientation.z;
  msg.pose.orientation.w = int_marker.pose.orientation.w;

  publisher_pose_stamped_->publish(msg);

}

void Goal3DDisplay::on_click_takeoffButton()
{
  if (!set_flight_mode_client_->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(rviz_ros_node_.lock()->get_raw_node()->get_logger(),
                 "Action server not available after waiting");
    return;
  }

  // Populate a goal
  auto goal_msg =  proposed_aerial_msgs::action::SetFlightMode::Goal();
  goal_msg.goal.flight_mode = 10;

  if(!flying_){
    goal_msg.goal.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING;
  }else{
    goal_msg.goal.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_LANDED;
  }

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Arm/Disarm request");
  auto send_goal_options = rclcpp_action::Client<proposed_aerial_msgs::action::SetFlightMode>::SendGoalOptions();
  send_goal_options.goal_response_callback =
     std::bind(&Goal3DDisplay::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
     std::bind(&Goal3DDisplay::result_callback, this, std::placeholders::_1);
   auto goal_handle_future = set_flight_mode_client_->async_send_goal(goal_msg, send_goal_options);
}

void Goal3DDisplay::result_callback(const GoalHandleSetFlightModeAction::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Arm/Disarm was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Arm/Disarm was canceled");
      return;
    default:
      RCLCPP_ERROR(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Unknown result code");
      return;
  }
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "result received %d", result.result->success);
}

void Goal3DDisplay::goal_response_callback(std::shared_future<GoalHandleSetFlightModeAction::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "goal was rejected by server");
  }else{
    RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "accepted by server, waiting for result");
  }
}

void Goal3DDisplay::on_click_armButton()
{

  if (!set_flight_mode_client_->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(rviz_ros_node_.lock()->get_raw_node()->get_logger(),
                 "Action server not available after waiting");
    return;
  }

  // Populate a goal
  auto goal_msg =  proposed_aerial_msgs::action::SetFlightMode::Goal();
  goal_msg.goal.flight_mode = 10;

  if(flight_mode_ == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED){
    goal_msg.goal.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED;
  }else if (flight_mode_ == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED){
    goal_msg.goal.flight_mode = proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED;
  }

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Arm/Disarm request");
  auto send_goal_options = rclcpp_action::Client<proposed_aerial_msgs::action::SetFlightMode>::SendGoalOptions();
  send_goal_options.goal_response_callback =
     std::bind(&Goal3DDisplay::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
     std::bind(&Goal3DDisplay::result_callback, this, std::placeholders::_1);
   auto goal_handle_future = set_flight_mode_client_->async_send_goal(goal_msg, send_goal_options);
}

void Goal3DDisplay::valueChangedInterface()
{

  if(flying_){
    button_takeoff_->setText("Land");
    button_takeoff_->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/Land.png"));
    button_arm_->setDisabled(true);
  }else{
    button_takeoff_->setText("TakeOff");
    button_takeoff_->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/Takeoff.png"));
    button_arm_->setDisabled(false);
  }

  if(flight_mode_ == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED){
    label_arming_state_->setText(QString("Standby"));
    button_arm_->setText(QString("Arm"));
    button_arm_->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/PowerOn.png"));
  }else if(flight_mode_ == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED){
    label_arming_state_->setText(QString("Armed"));
    button_arm_->setText(QString("Disarm"));
    button_arm_->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/PowerOff.png"));
  }else if(flight_mode_ == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_FLYING){
    label_arming_state_->setText("Flying");
  }else if(flight_mode_ == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_LANDED){
    label_arming_state_->setText("Landed");
  }else if(flight_mode_ == proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_RTL){
    label_arming_state_->setText("RTL");
  }
}

void Goal3DDisplay::subcribe2topics()
{

  set_flight_mode_client_ =  rclcpp_action::create_client<proposed_aerial_msgs::action::SetFlightMode>(
      rviz_ros_node_.lock()->get_raw_node()->get_node_base_interface(),
      rviz_ros_node_.lock()->get_raw_node()->get_node_graph_interface(),
      rviz_ros_node_.lock()->get_raw_node()->get_node_logging_interface(),
      rviz_ros_node_.lock()->get_raw_node()->get_node_waitables_interface(),
      set_flight_mode_name_);
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Action client: " + set_flight_mode_name_);

  vehicle_gps_position_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<sensor_msgs::msg::NavSatFix>(
        vehicle_gps_position_name_,
      10,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        latitude_ = msg->latitude;
        longitude_ = msg->longitude;
        altitude_ = msg->altitude;
    });

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + vehicle_gps_position_name_);

  vehicle_attitude_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<proposed_aerial_msgs::msg::Attitude>(
        attitude_topic_name_,
      10,
      [this](proposed_aerial_msgs::msg::Attitude::ConstSharedPtr msg) {

        geometry_msgs::msg::Quaternion q;
        q.x = msg->orientation.x;
        q.y = msg->orientation.y;
        q.z = msg->orientation.z;
        q.w = msg->orientation.w;
        double yaw, pitch, roll;
        tf2::getEulerYPR(q, yaw, pitch, roll);
        heading_ = yaw;
    });
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + attitude_topic_name_);

  flight_mode_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<proposed_aerial_msgs::msg::FlightMode>(
        flight_mode_name_,
      10,
      [this](proposed_aerial_msgs::msg::FlightMode::ConstSharedPtr msg) {
        flying_ = msg->flight_mode != proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_DISARMED &&
          msg->flight_mode != proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_LANDED &&
          msg->flight_mode != proposed_aerial_msgs::msg::FlightMode::FLIGHT_MODE_ARMED;
        if(flight_mode_ != msg->flight_mode){
          flight_mode_ = msg->flight_mode;
          emit valueChangedInterface_signal();
        }
    });

  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + flight_mode_name_);

  vehicle_status_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<proposed_aerial_msgs::msg::VehicleStatus>(
        vehicle_status_name_,
      10,
      [this](proposed_aerial_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
        if(msg->system_type == proposed_aerial_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING){
          label_vehicle_type_->setText("Rotary wings");
        }else if(msg->system_type == proposed_aerial_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING){
          label_vehicle_type_->setText("Fixed wing");
        }else if(msg->system_type == proposed_aerial_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROVER){
          label_vehicle_type_->setText("Rover");
        }else if(msg->system_type == proposed_aerial_msgs::msg::VehicleStatus::VEHICLE_TYPE_VTOL){
          label_vehicle_type_->setText("VTOL");
        }else{
          label_vehicle_type_->setText("UNKNOWN");
        }
    });
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Subscribe to: " + vehicle_status_name_);

  publisher_pose_stamped_ =
    rviz_ros_node_.lock()->get_raw_node()->
      create_publisher<geometry_msgs::msg::PoseStamped>(pose_stamped_name_, 10);
  RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Publish to: " + pose_stamped_name_);

}

void Goal3DDisplay::on_changed_namespace(const QString& text)
{
  std::string namespace_str(text.toUtf8().constData());

  vehicle_gps_position_name_ = "/" + namespace_str + "/gps";
  attitude_topic_name_ = "/" + namespace_str + "/attitude";
  pose_stamped_name_ = "/" + namespace_str + "/command_pose";
  set_flight_mode_name_ = "/" + namespace_str + "/set_flight_mode";
  flight_mode_name_ = "/" + namespace_str + "/flight_mode";
  vehicle_status_name_ = "/" + namespace_str + "/status";

  vehicle_gps_position_sub_.reset();
  vehicle_attitude_sub_.reset();
  set_flight_mode_client_.reset();
  flight_mode_sub_.reset();
  publisher_pose_stamped_.reset();
  vehicle_status_sub_.reset();

  subcribe2topics();
}

void Goal3DDisplay::on_click_refresheButton()
{
  add_namespaces_to_combobox();
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

void Goal3DDisplay::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  std::ostringstream oss;
  oss << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  switch (feedback->event_type) {
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
          << "\nframe: " << feedback->header.frame_id;
      RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), oss.str());
      break;
  }

  server_->applyChanges();
}

visualization_msgs::msg::Marker
Goal3DDisplay::makeBox(const visualization_msgs::msg::InteractiveMarker & msg)
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://mavlink_sitl_gazebo/models/rotors_description/meshes/iris.stl";
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
