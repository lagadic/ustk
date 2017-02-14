/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Pierre Chatelain
 *
 *****************************************************************************/

#include <visp3/ustk_core/usRectangle.h>

#include <cmath>

usRectangle::usRectangle() : m_cx(0.0), m_cy(0.0), m_width(0.0), m_height(0.0), m_theta(0.0),
           m_x1(0.0), m_x2(0.0), m_x3(0.0), m_x4(0.0),
           m_y1(0.0), m_y2(0.0), m_y3(0.0), m_y4(0.0) {}

usRectangle::usRectangle(double cx, double cy, double width, double height, double theta)
{
  m_cx = cx;
  m_cy = cy;
  m_width = width;
  m_height = height;
  m_theta = theta;
  m_x1 = m_cx - m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2.0;
  m_y1 = m_cy + m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2.0;
  m_x2 = m_cx + m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2.0;
  m_y2 = m_cy - m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2.0;
  m_x3 = m_cx + m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2.0;
  m_y3 = m_cy - m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2.0;
  m_x4 = m_cx - m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2.0;
  m_y4 = m_cy + m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2.0;
}

void usRectangle::setPoints(double x1, double y1, double x2, double y2,
          double x3, double y3, double x4, double y4)
{
  m_x1 = x1;
  m_x2 = x2;
  m_x3 = x3;
  m_x4 = x4;
  m_y1 = y1;
  m_y2 = y2;
  m_y3 = y3;
  m_y4 = y4;
  m_cx = (x1 + x2 + x3 + x4) / 4.0;
  m_cy = (y1 + y2 + y3 + y4) / 4.0;
  m_width = sqrt((x4 - x1) * (x4 - x1) + (y4 - y1) * (y4 - y1));
  m_height = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  m_theta = atan2(x4 - x1, y4 - y1);
}

void usRectangle::setCenter(double cx, double cy)
{
  m_x1 += cx - m_cx;
  m_x2 += cx - m_cx;
  m_x3 += cx - m_cx;
  m_x4 += cx - m_cx;
  m_y1 += cy - m_cy;
  m_y2 += cy - m_cy;
  m_y3 += cy - m_cy;
  m_y4 += cy - m_cy;
  m_cx = cx;
  m_cy = cy;
}

double usRectangle::getCx() const { return m_cx; }
double usRectangle::getCy() const { return m_cy; }
double usRectangle::getX1() const { return m_x1; }
double usRectangle::getX2() const { return m_x2; }
double usRectangle::getX3() const { return m_x3; }
double usRectangle::getX4() const { return m_x4; }
double usRectangle::getY1() const { return m_y1; }
double usRectangle::getY2() const { return m_y2; }
double usRectangle::getY3() const { return m_y3; }
double usRectangle::getY4() const { return m_y4; }

void usRectangle::setSize(double width, double height)
{
  m_width = width;
  m_height = height;
  m_x1 = m_cx - m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2;
  m_y1 = m_cy + m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2;
  m_x2 = m_cx + m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2;
  m_y2 = m_cy - m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2;
  m_x3 = m_cx + m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2;
  m_y3 = m_cy - m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2;
  m_x4 = m_cx - m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2;
  m_y4 = m_cy + m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2;
}

double usRectangle::getWidth() const { return m_width; }
double usRectangle::getHeight() const { return m_height; }

void usRectangle::setOrientation(double theta)
{
  m_theta = theta;
  m_x1 = m_cx - m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2;
  m_y1 = m_cy + m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2;
  m_x2 = m_cx + m_height * cos(m_theta) / 2.0 - m_width * sin(m_theta) / 2;
  m_y2 = m_cy - m_height * sin(m_theta) / 2.0 - m_width * cos(m_theta) / 2;
  m_x3 = m_cx + m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2;
  m_y3 = m_cy - m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2;
  m_x4 = m_cx - m_height * cos(m_theta) / 2.0 + m_width * sin(m_theta) / 2;
  m_y4 = m_cy + m_height * sin(m_theta) / 2.0 + m_width * cos(m_theta) / 2;
}

double usRectangle::getOrientation() const { return m_theta; }

bool usRectangle::isInside(double x, double y) const
{
  if (!isLeft(x, y, m_x1, m_y1, m_x2, m_y2))
    return false;
  if (!isLeft(x, y, m_x2, m_y2, m_x3, m_y3))
    return false;
  if (!isLeft(x, y, m_x3, m_y3, m_x4, m_y4))
    return false;
  if (!isLeft(x, y, m_x4, m_y4, m_x1, m_y1))
    return false;
  return true;
}

bool usRectangle::isLeft(double x, double y, double x1, double y1, double x2, double y2) const
{
  double a = y1 - y2;
  double b = x2 - x1;
  double c = - (a * x1 + b * y1);
  double d = a * x + b * y + c;
  return (d > 0);
}
