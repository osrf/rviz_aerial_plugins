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

#ifndef GOAL3D_WIDGET_WIDGET_H
#define GOAL3D_WIDGET_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QGroupBox>

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

class Goal3DWidget: public QWidget
{
public:
    Goal3DWidget(QWidget* parent = 0);

};

#endif // GOAL3D_WIDGET_WIDGET_H
