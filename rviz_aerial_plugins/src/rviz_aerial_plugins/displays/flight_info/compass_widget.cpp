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

#include "rviz_aerial_plugins/displays/flight_info/compass_widget.hpp"

namespace rviz_aerial_plugins
{

namespace displays
{

CompassWidget::CompassWidget( QWidget* parent )
  : QWidget( parent )
{
  angle_ = 0;
  margins_ = 10;
  pointText_ = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  pointNumber_ = {0, 45, 90, 135, 180, 225, 280, 315};

  setMinimumSize(200, 200);
  setMaximumSize(600, 600);
  resize(200, 200);
}

CompassWidget::~CompassWidget()
{

}

void CompassWidget::setAngle(float angle)
{
  std::lock_guard<std::mutex> lock(mutex);
  angle_ = angle;
}

float CompassWidget::getAngle()
{
  std::lock_guard<std::mutex> lock(mutex);
  return angle_;
}

void CompassWidget::paintEvent( QPaintEvent* event )
{
  QPainter painter;
  painter.begin(this);
  painter.setRenderHint(QPainter::Antialiasing);

//  painter.fillRect(event->rect(), palette().brush(QPalette::Window));
  drawMarkings(painter);
  drawNeedle(painter);

  painter.end();
}

void CompassWidget::drawMarkings(QPainter& painter)
{
  painter.save();
  painter.translate(width()/2, height()/2);
  float scale = std::min((width()  - margins_)/120.0,
                         (height() - margins_)/120.0);
  painter.scale(scale, scale);

  QFont font = QFont();
  font.setPixelSize(10);
  QFontMetricsF metrics = QFontMetricsF(font);

  painter.setFont(font);
  painter.setBrush(QBrush(Qt::black));

  int i = 0;
  int j = 0;
  while(i < 360){
      if(i%45==0){
          painter.drawLine(0, -40, 0, -50);
          painter.drawText(-metrics.width(QString(pointText_[j].c_str()))/2.0, -52,
                           QString(pointText_[j].c_str()));
          j++;
      }else{
          painter.drawLine(0, -45, 0, -50);
      }
      painter.rotate(15);
      i += 15;
  }
  painter.restore();
}
void CompassWidget::drawNeedle(QPainter& painter)
{
  std::lock_guard<std::mutex> lock(mutex);
  painter.save();
  painter.translate(width()/2, height()/2);
  painter.rotate(angle_);
  float scale = std::min((width() - margins_)/120.0,
              (height() - margins_)/120.0);
  painter.scale(scale, scale);

  painter.setBrush(QBrush(Qt::red));
  painter.setPen(Qt::NoPen);

  QVector<QPoint> vector_points;
  vector_points.append(QPoint(-10, 0));
  vector_points.append(QPoint(0, -45));
  vector_points.append(QPoint(10, 0));
  vector_points.append(QPoint(0, -15));

  painter.drawPolygon(QPolygon(vector_points));

  painter.restore();
}
} // namespace displays

} // end namespace rviz_aerial_plugins
