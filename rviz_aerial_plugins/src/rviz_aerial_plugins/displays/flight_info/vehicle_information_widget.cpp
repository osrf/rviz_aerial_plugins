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

#include "rviz_aerial_plugins/displays/flight_info/vehicle_information_widget.hpp"

VehicleInformationWidget::VehicleInformationWidget(QWidget* parent)
{
  QFont font = QFont();
  font.setBold(true);

  alt_text = new QLabel("Alt (Rel)(m)");
  alt_label = new QLabel("-0.1");
  alt_text->setAlignment(Qt::AlignCenter);
  alt_label->setAlignment(Qt::AlignCenter);
  alt_text->setFont(font);

  ground_speed_text = new QLabel("Ground Speed (m/s)");
  ground_speed_label = new QLabel("0.0");
  ground_speed_text->setAlignment(Qt::AlignCenter);
  ground_speed_label->setAlignment(Qt::AlignCenter);
  ground_speed_text->setFont(font);

  flight_time_text = new QLabel("Fligh time");
  flight_time_label = new QLabel("00:00:00");
  flight_time_text->setAlignment(Qt::AlignCenter);
  flight_time_label->setAlignment(Qt::AlignCenter);
  flight_time_text->setFont(font);

  QGroupBox* groupBox = new QGroupBox();

  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget( alt_text );
  topic_layout->addWidget( alt_label );
  topic_layout->addWidget( ground_speed_text );
  topic_layout->addWidget( ground_speed_label );
  topic_layout->addWidget( flight_time_text );
  topic_layout->addWidget( flight_time_label );

  groupBox->setLayout(topic_layout);

  QGridLayout *grid = new QGridLayout;
  grid->addWidget(groupBox, 0, 0);

  setLayout(grid);
}

void VehicleInformationWidget::setFlightTime()
{
}

void VehicleInformationWidget::setGroundSpeed(float speed)
{
  ground_speed_label->setText(QString("%1").arg(speed));
}

void VehicleInformationWidget::setAlt(float alt)
{
  alt_label->setText(QString("%1").arg(alt));
}
