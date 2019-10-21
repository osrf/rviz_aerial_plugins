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

#include "rviz_aerial_plugins/displays/flight_info/flight_info_panel.hpp"
#include <QVBoxLayout>
#include <QLabel>

namespace rviz_aerial_plugins
{

namespace displays
{

FlighInfoDisplay::FlighInfoDisplay(QWidget* parent):
 rviz_common::Panel(parent), rviz_ros_node_()
{
}

FlighInfoDisplay::~FlighInfoDisplay()
{

}

void FlighInfoDisplay::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();

  compass_widget_ = new CompassWidget();
  adi_widget_ = new ADIWidget();
  vi_widget_ = new VehicleInformationWidget();

  QGridLayout *grid = new QGridLayout;

  QComboBox* combo_topics = new QComboBox();

  grid->addWidget( compass_widget_ , 0, 0);
  grid->addWidget( adi_widget_, 0, 1);
  grid->addWidget( vi_widget_, 1, 0, 1, 2);
  grid->addWidget( combo_topics, 2, 0, 1, 2);

  setLayout(grid);

  vehicle_attitude_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/vehicle_attitude",
      10,
      [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg) {

        geometry_msgs::msg::Quaternion q;
        q.x = msg->q[1];
        q.y = msg->q[2];
        q.z = msg->q[3];
        q.w = msg->q[0];
        double yaw, pitch, roll;
        tf2::getEulerYPR(q, yaw, pitch, roll);
        compass_widget_->setAngle(yaw*180/3.1416);
        compass_widget_->update();
        adi_widget_->setPitch(pitch*180/3.1416);
        adi_widget_->setRoll(-roll*180/3.1416);
        adi_widget_->update();
    });

  vehicle_odometry_sub_ = rviz_ros_node_.lock()->get_raw_node()->
        template create_subscription<px4_msgs::msg::VehicleOdometry>(
          "/vehicle_odometry",
        10,
        [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
          vi_widget_->setGroundSpeed(sqrt(msg->vx*msg->vx + msg->vy*msg->vy + msg->vz*msg->vz));
          vi_widget_->setAlt(-msg->z);
      });
}

} // namespace displays

} // namespace rviz_aerial_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_aerial_plugins::displays::FlighInfoDisplay, rviz_common::Panel)
