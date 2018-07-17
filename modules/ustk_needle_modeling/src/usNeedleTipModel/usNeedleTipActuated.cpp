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
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

#include <visp3/ustk_needle_modeling/usNeedleTipActuated.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>

usNeedleTipActuated::usNeedleTipActuated() : usNeedleTip(), _diameter(0), _length(0), _angle(0), _steeringAngle(0) {}

usNeedleTipActuated::usNeedleTipActuated(const usNeedleTipActuated &tip)
  : usNeedleTip(tip), _diameter(tip._diameter), _length(tip._length), _angle(tip._angle),
    _steeringAngle(tip._steeringAngle)
{
}

usNeedleTipActuated::~usNeedleTipActuated() {}

usNeedleTipActuated &usNeedleTipActuated::operator=(const usNeedleTipActuated &tip)
{
  this->usNeedleTip::operator=(tip);

  _diameter = tip._diameter;
  _length = tip._length;
  _angle = tip._angle;
  _steeringAngle = tip._steeringAngle;

  return (*this);
}

usNeedleTipActuated *usNeedleTipActuated::clone() const { return new usNeedleTipActuated(*this); }

void usNeedleTipActuated::setDiameter(double diameter)
{
  if (diameter > 0)
    _diameter = diameter;
}

double usNeedleTipActuated::getDiameter() const { return _diameter; }

void usNeedleTipActuated::setLength(double l)
{
  if (l >= 0)
    _length = l;
}

double usNeedleTipActuated::getLength() const { return _length; }

void usNeedleTipActuated::setTipAngleRad(double angle)
{
  if (angle >= 0 && angle <= M_PI / 2)
    _angle = angle;
}

double usNeedleTipActuated::getTipAngleRad() const { return _angle; }

void usNeedleTipActuated::setTipAngleDeg(double angle)
{
  if (angle >= 0 && angle <= 90)
    _angle = M_PI / 180 * angle;
}

double usNeedleTipActuated::getTipAngleDeg() const { return 180 / M_PI * _angle; }

void usNeedleTipActuated::setSteeringAngleRad(double angle)
{
  while (angle <= -M_PI)
    angle += 2 * M_PI;
  while (angle > M_PI)
    angle -= 2 * M_PI;
  _steeringAngle = angle;
}

double usNeedleTipActuated::getSteeringAngleRad() const { return _steeringAngle; }

void usNeedleTipActuated::setSteeringAngleDeg(double angle)
{
  while (angle <= -180)
    angle += 360;
  while (angle > 180)
    angle -= 360;
  _steeringAngle = M_PI / 180 * angle;
}

double usNeedleTipActuated::getSteeringAngleDeg() const { return 180 / M_PI * _steeringAngle; }

std::ostream &operator<<(std::ostream &s, const usNeedleTipActuated &tip)
{
  s << "usNeedleTipActuated\n";
  s << (const usNeedleTip &)tip;

  s << tip._diameter << '\n';
  s << tip._length << '\n';
  s << tip._angle << '\n';
  s << tip._steeringAngle << '\n';

  s.flush();
  return s;
}

std::istream &operator>>(std::istream &s, usNeedleTipActuated &tip)
{
  std::string c;
  s >> c;
  if (c != "usNeedleTipActuated") {
    vpException e(vpException::ioError, "Stream does not contain usNeedleTipActuated data");
    throw e;
  }
  s >> (usNeedleTip &)tip;
  s >> tip._diameter;
  s >> tip._length;
  s >> tip._angle;
  s >> tip._steeringAngle;
  s.get();

  return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleTipActuated &tip)
{
  s.write("usNeedleTipActuated", 20);

  s <<= (const usNeedleTip &)tip;
  s.write((char *)&(tip._diameter), sizeof(double));
  s.write((char *)&(tip._length), sizeof(double));
  s.write((char *)&(tip._angle), sizeof(double));
  s.write((char *)&(tip._steeringAngle), sizeof(double));

  s.flush();
  return s;
}

std::istream &operator>>=(std::istream &s, usNeedleTipActuated &tip)
{
  char c[20];
  s.read(c, 20);
  if (strcmp(c, "usNeedleTipActuated")) {
    vpException e(vpException::ioError, "Stream does not contain usNeedleTipActuated data");
    throw e;
  }

  s >>= (usNeedleTip &)tip;
  s.read((char *)&(tip._diameter), sizeof(double));
  s.read((char *)&(tip._length), sizeof(double));
  s.read((char *)&(tip._angle), sizeof(double));
  s.read((char *)&(tip._steeringAngle), sizeof(double));
  return s;
}

void usNeedleTipActuated::updateTipPose()
{
  vpHomogeneousMatrix H(_length * cos(_steeringAngle) * sin(_angle), _length * sin(_steeringAngle) * sin(_angle),
                        _length * cos(_angle), -_angle * sin(_steeringAngle), _angle * cos(_steeringAngle), 0);

  m_worldMtip = m_worldMbase * H;
  m_tipPose.buildFrom(m_worldMtip);
}
