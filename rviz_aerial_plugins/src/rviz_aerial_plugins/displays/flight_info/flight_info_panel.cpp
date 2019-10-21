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
 rviz_common::Panel(parent)
{
  compass_widget_ = new CompassWidget();
  adi_widget_ = new ADIWidget();
  vi_widget_ = new VehicleInformationWidget();

  QGridLayout *grid = new QGridLayout;

  grid->addWidget( compass_widget_ , 0, 0);
  grid->addWidget( adi_widget_, 0, 1);
  grid->addWidget( vi_widget_, 1, 0, 1, 2);

  setLayout(grid);
}

FlighInfoDisplay::~FlighInfoDisplay()
{

}

void FlighInfoDisplay::onInitialize()
{
  sub_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->
        template create_subscription<px4_msgs::msg::SensorCombined>(
          "/sensor_combined",
        10,
        [this](px4_msgs::msg::SensorCombined::ConstSharedPtr msg) {
          compass_widget_->setAngle(0);
          compass_widget_->update();

      });
}

} // namespace displays

} // namespace rviz_aerial_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_aerial_plugins::displays::FlighInfoDisplay, rviz_common::Panel)
