#include "rviz_aerial_plugins/displays/flight_info/attitude_display_indicator_widget.hpp"

ADIWidget::ADIWidget(QWidget* parent) : QWidget( parent ),
  whitePen(Qt::white),
  blackPen(Qt::black),
  pitchPen(Qt::white),
  pitchZero(Qt::green),
  bgSky(QColor(48,172,220)),
  bgGround(QColor(0, 147, 57))
{
  offset_ = 2;
  size_ = 200 - 2*offset_;

  setMinimumSize(200, 200);
  setMaximumSize(600, 600);
  resize(200, 200);

  setFocusPolicy(Qt::NoFocus);

  roll_  = 0.0;
  pitch_ = 0.0;

  whitePen.setWidth(2);
  blackPen.setWidth(2);
  pitchZero.setWidth(3);
}

ADIWidget::~ADIWidget()
{

}

void ADIWidget::setPitch(float pitch)
{
  std::lock_guard<std::mutex> lock(mutex);
  pitch_ = pitch;
}
void ADIWidget::setRoll(float roll)
{
  std::lock_guard<std::mutex> lock(mutex);
  roll_ = roll;
}

float ADIWidget::getPitch()
{
  std::lock_guard<std::mutex> lock(mutex);
  return pitch_;
}

float ADIWidget::getRoll()
{
  std::lock_guard<std::mutex> lock(mutex);
  return roll_;
}

void ADIWidget::paintEvent( QPaintEvent* )
{
  QPainter painter(this);

  painter.setRenderHint(QPainter::Antialiasing);

  painter.translate(width() / 2, height() / 2);
  painter.rotate(roll_);

  drawBackgound(painter);
  drawPitch(painter);
  drawRoll(painter);
}

void ADIWidget::drawBackgound(QPainter& painter)
{
  int y_min, y_max;

  y_min = size_/2*-40.0/45.0;
  y_max = size_/2* 40.0/45.0;

  int y = size_/2*-pitch_/45.;
  if( y < y_min ) y = y_min;
  if( y > y_max ) y = y_max;

  int x = sqrt(size_*size_/4 - y*y);
  qreal gr = atan((double)(y)/x);
  gr = gr * 180./3.1415926;

  painter.setPen(QPen(Qt::black));
  painter.setBrush(QColor(48,172,220));
  painter.drawChord(-size_/2, -size_/2, size_, size_,
                    gr*16, (180-2*gr)*16);

  painter.setBrush(bgGround);
  painter.drawChord(-size_/2, -size_/2, size_, size_,
                    gr*16, -(180+2*gr)*16);
}

void ADIWidget::drawPitch(QPainter& painter)
{
  std::lock_guard<std::mutex> lock(mutex);

  // set mask
  QRegion maskRegion(-size_/2, -size_/2, size_, size_, QRegion::Ellipse);
  painter.setClipRegion(maskRegion);

  int x, y, x1, y1;
  int textWidth;
  double p, r;
  int ll = size_/8, l;

  int     fontSize = 8;
  QString s;

  pitchPen.setWidth(2);
  painter.setFont(QFont("", fontSize));

  // draw lines
  for(int i=-9; i<=9; i++) {
      p = i*10;

      s = QString("%1").arg(-p);

      if( i % 3 == 0 )
          l = ll;
      else
          l = ll/2;

      if( i == 0 ) {
          painter.setPen(pitchZero);
          l = l * 1.8;
      } else {
          painter.setPen(pitchPen);
      }

      y = size_/2*p/45.0 - size_/2*-pitch_/45.;
      x = l;

      r = sqrt(x*x + y*y);
      if( r > size_/2 ) continue;

      painter.drawLine(QPointF(-l, 1.0*y), QPointF(l, 1.0*y));

      textWidth = 100;

      if( i % 3 == 0 && i != 0 ) {
          painter.setPen(QPen(Qt::white));

          x1 = -x-2-textWidth;
          y1 = y - fontSize/2 - 1;
          painter.drawText(QRectF(x1, y1, textWidth, fontSize+2),
                           Qt::AlignRight|Qt::AlignVCenter, s);
      }
  }

  // draw marker
  int     markerSize = size_/20;
  float   fx1, fy1, fx2, fy2, fx3, fy3;

  painter.setBrush(QBrush(Qt::red));
  painter.setPen(Qt::NoPen);

  fx1 = markerSize;
  fy1 = 0;
  fx2 = fx1 + markerSize;
  fy2 = -markerSize/2;
  fx3 = fx1 + markerSize;
  fy3 = markerSize/2;

  QPointF points[3] = {
      QPointF(fx1, fy1),
      QPointF(fx2, fy2),
      QPointF(fx3, fy3)
  };
  painter.drawPolygon(points, 3);

  QPointF points2[3] = {
      QPointF(-fx1, fy1),
      QPointF(-fx2, fy2),
      QPointF(-fx3, fy3)
  };
  painter.drawPolygon(points2, 3);
}

void ADIWidget::drawRoll(QPainter& painter)
{
  std::lock_guard<std::mutex> lock(mutex);

  int     nRollLines = 36;
  float   rotAng = 360.0 / nRollLines;
  int     rollLineLeng = size_/25;
  double  fx1, fy1, fx2, fy2, fx3, fy3;
  int     fontSize = 8;
  QString s;

  blackPen.setWidth(1);
  painter.setPen(blackPen);
  painter.setFont(QFont("", fontSize));

  for(int i=0; i<nRollLines; i++) {
      if( i < nRollLines/2 )
          s = QString("%1").arg(-i*rotAng);
      else
          s = QString("%1").arg(360-i*rotAng);

      fx1 = 0;
      fy1 = -size_/2 + offset_;
      fx2 = 0;

      if( i % 3 == 0 ) {
          fy2 = fy1 + rollLineLeng;
          painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));

          fy2 = fy1 + rollLineLeng+2;
          painter.drawText(QRectF(-50, fy2, 100, fontSize+2),
                           Qt::AlignCenter, s);
      } else {
          fy2 = fy1 + rollLineLeng/2;
          painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
      }

      painter.rotate(rotAng);
  }

  // draw roll marker
  int     rollMarkerSize = size_/25;

  painter.rotate(-roll_);
  painter.setBrush(QBrush(Qt::black));

  fx1 = 0;
  fy1 = -size_/2 + offset_;
  fx2 = fx1 - rollMarkerSize/2;
  fy2 = fy1 + rollMarkerSize;
  fx3 = fx1 + rollMarkerSize/2;
  fy3 = fy1 + rollMarkerSize;

  QPointF points[3] = {
      QPointF(fx1, fy1),
      QPointF(fx2, fy2),
      QPointF(fx3, fy3)
  };
  painter.drawPolygon(points, 3);
}
