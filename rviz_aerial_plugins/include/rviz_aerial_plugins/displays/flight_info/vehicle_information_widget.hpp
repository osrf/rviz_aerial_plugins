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
