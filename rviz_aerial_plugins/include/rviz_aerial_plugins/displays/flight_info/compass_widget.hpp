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

#ifndef COMPASS_WIDGET_H
#define COMPASS_WIDGET_H

#include <QWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QPainter>
#include <QPolygon>
#include <QPoint>
#include <mutex>
#include <math.h>

namespace rviz_aerial_plugins
{

namespace displays
{

class CompassWidget: public QWidget
{
    Q_OBJECT
public:
    explicit CompassWidget( QWidget* parent = 0 );
    ~CompassWidget();

  /// We override QWidget::paintEvent() to paint the compass.
  virtual void paintEvent( QPaintEvent* event );

  /// Override sizeHint() to give the layout managers some idea of a
  /// good size for this.
  virtual QSize sizeHint() const { return QSize( 150, 150 ); }

  /// Set the angle of the needle.
  void setAngle(float angle);

  /// Return the angle.
  float getAngle();

  /// Draw the marking arround the compass
  void drawMarkings(QPainter& painter);

  /// Draw the needle
  void drawNeedle(QPainter& painter);

private:
  std::mutex mutex;
  float angle_;
  float margins_;
  std::vector<std::string> pointText_;
  std::vector<int> pointNumber_;
};


} // namespace displays

} // end namespace rviz_plugin_tutorials

#endif // COMPASS_WIDGET_H
