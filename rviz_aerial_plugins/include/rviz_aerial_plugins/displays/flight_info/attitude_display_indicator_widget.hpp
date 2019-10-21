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

#ifndef ALTITUDE_DISPLAY_INDICATOR_WIDGET_H
#define ALTITUDE_DISPLAY_INDICATOR_WIDGET_H

#include <QWidget>
#include <QPainter>
#include <math.h>
#include <mutex>

class ADIWidget: public QWidget
{
    Q_OBJECT
public:
    ADIWidget(QWidget* parent = 0 );
    ~ADIWidget();

    // We override QWidget::paintEvent() to do custom painting.
    virtual void paintEvent( QPaintEvent* event );

    // Override sizeHint() to give the layout managers some idea of a
    // good size for this.
    virtual QSize sizeHint() const { return QSize( 150, 150 ); }

    void setPitch(float pitch);
    void setRoll(float roll);
    float getPitch();
    float getRoll();

protected:
    int     size_, offset_;               ///< current size & offset
    double  roll_;                        ///< roll angle (in degree)
    double  pitch_;                       ///< pitch angle (in degree)

private:
    std::mutex mutex;

    //// draw background
    void drawBackgound(QPainter& painter);

    //// draw roll lines & marker
    void drawRoll(QPainter& painter);

    //// draw pitch lines & marker
    void drawPitch(QPainter& painter);

    QBrush bgSky;
    QBrush bgGround;

    QPen   whitePen;
    QPen   blackPen;
    QPen   pitchPen;
    QPen   pitchZero;
};

#endif // ALTITUDE_DISPLAY_INDICATOR_WIDGET_H
