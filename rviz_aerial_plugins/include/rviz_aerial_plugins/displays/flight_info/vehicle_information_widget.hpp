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

#ifndef VEHICLE_INFORMATION_WIDGET_H
#define VEHICLE_INFORMATION_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QGroupBox>

class VehicleInformationWidget: public QWidget
{
public:
    VehicleInformationWidget(QWidget* parent = 0);

public:
    QLabel* alt_text;
    QLabel* ground_speed_text;
    QLabel* flight_time_text;
    QLabel* alt_label;
    QLabel* ground_speed_label;
    QLabel* flight_time_label;
};

#endif // VEHICLE_INFORMATION_WIDGET_H
