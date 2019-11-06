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


#ifndef RVIZ_AERIAL_PLUGINS__DISPLAYS__BATTERY__BATTERY_DISPLAY_HPP_
#define RVIZ_AERIAL_PLUGINS__DISPLAYS__BATTERY__BATTERY_DISPLAY_HPP_

#include <memory>

#ifndef Q_MOC_RUN

#include "rviz_common/ros_topic_display.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

# include "rviz_aerial_plugins/visibility_control.hpp"
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{

namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

}

namespace rviz_aerial_plugins
{

namespace displays
{

class RVIZ_AERIAL_PLUGINS_PUBLIC BatteryDisplay:
  public rviz_common::RosTopicDisplay<sensor_msgs::msg::BatteryState>
{
Q_OBJECT
public:

  BatteryDisplay();
  ~BatteryDisplay() override;

  void onInitialize() override;
  void reset() override;

private:
  void processMessage( const sensor_msgs::msg::BatteryState::ConstSharedPtr msg ) override;

  rviz_common::properties::IntProperty * battery_cells_property_;
  rviz_common::properties::FloatProperty * battery_status_property_;
};

} // namespace displays

} // namespace rviz_hrim_plugin_

#endif // RVIZ_AERIAL_PLUGINS__DISPLAYS__BATTERY__BATTERY_DISPLAY_HPP_
