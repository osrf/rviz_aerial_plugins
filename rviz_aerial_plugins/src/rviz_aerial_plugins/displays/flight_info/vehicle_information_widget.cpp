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
