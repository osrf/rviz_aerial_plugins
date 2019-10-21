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

#include "rviz_aerial_plugins/displays/battery/battery_display.hpp"
// #include "rviz_aerial_plugins/displays/imu/imu_visual.hpp"

#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <tf2_ros/transform_listener.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/frame_manager_iface.hpp"

namespace rviz_aerial_plugins
{

namespace displays
{

BatteryDisplay::BatteryDisplay()
{

  battery_status_property_ = new rviz_common::properties::FloatProperty( "Battery Status", 1,
                                                    "Battery voltage.", this);

  battery_cells_property_ = new rviz_common::properties::IntProperty( "Battery cells", 1,
                                                     "Battery cells.", this);
}

void BatteryDisplay::onInitialize()
{
  RTDClass::onInitialize();
  setTopic(QString("/battery_status"), QString(""));
}

BatteryDisplay::~BatteryDisplay()
{
}

// Clear the visuals by deleting their objects.
void BatteryDisplay::reset()
{
  RTDClass::reset();
}

// // Handle an incoming message.
void BatteryDisplay::processMessage( const px4_msgs::msg::BatteryStatus::ConstSharedPtr msg )
{
  battery_status_property_->setFloat(msg->voltage_v);
  battery_cells_property_->setInt(msg->cell_count);
}

} // namespace displays

} // namespace rviz_aerial_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_aerial_plugins::displays::BatteryDisplay, rviz_common::Display)
