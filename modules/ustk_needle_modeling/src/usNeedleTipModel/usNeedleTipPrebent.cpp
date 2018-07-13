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

#include <visp3/ustk_needle_modeling/usNeedleTipPrebent.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>


usNeedleTipPrebent::usNeedleTipPrebent() :
    usNeedleTip(),
    _diameter(0.001),
    _length(0.01),
    _angle(0)
{

}

usNeedleTipPrebent::usNeedleTipPrebent(const usNeedleTipPrebent &tip):
    usNeedleTip(tip),
    _diameter(tip._diameter),
    _length(tip._length),
    _angle(tip._angle)
{
}

usNeedleTipPrebent::~usNeedleTipPrebent()
{
}

usNeedleTipPrebent& usNeedleTipPrebent::operator=(const usNeedleTipPrebent &tip)
{
    this->usNeedleTip::operator=(tip);

    _diameter = tip._diameter;
    _length = tip._length;
    _angle = tip._angle;

    return (*this);
}

usNeedleTipPrebent* usNeedleTipPrebent::clone() const
{
    return new usNeedleTipPrebent(*this);
}

void usNeedleTipPrebent::setDiameter(double diameter)
{
    if(diameter > 0) _diameter = diameter;
}

double usNeedleTipPrebent::getDiameter() const
{
    return _diameter;
}

void usNeedleTipPrebent::setLength(double l)
{
    if(l >= 0) _length = l;
}

double usNeedleTipPrebent::getLength() const
{
    return _length;
}

void usNeedleTipPrebent::setAngleRad(double angle)
{
    if(angle >= 0 && angle <= M_PI/2) _angle = angle;
}

double usNeedleTipPrebent::getAngleRad() const
{
    return _angle;
}

void usNeedleTipPrebent::setAngleDeg(double angle)
{
    if(angle >= 0 && angle <= 90) _angle = M_PI/180*angle;
}

double usNeedleTipPrebent::getAngleDeg() const
{
    return 180/M_PI*_angle;
}

std::ostream &operator<<(std::ostream &s, const usNeedleTipPrebent &tip)
{
    s << "usNeedleTipPrebent\n";
    s << (const usNeedleTip&)tip;

    s << tip._diameter << '\n';
    s << tip._length << '\n';
    s << tip._angle << '\n';

    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usNeedleTipPrebent &tip)
{
    std::string c;
    s >> c;
    if(c != "usNeedleTipPrebent")
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleTipPrebent data");
        throw e;
    }
    s >> (usNeedleTip&)tip;
    s >> tip._diameter;
    s >> tip._length;
    s >> tip._angle;
    s.get();
    
    return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleTipPrebent &tip)
{
    s.write("usNeedleTipPrebent",19);

    s <<= (const usNeedleTip&)tip;
    s.write((char*)&(tip._diameter), sizeof(double));
    s.write((char*)&(tip._length), sizeof(double));
    s.write((char*)&(tip._angle), sizeof(double));

    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usNeedleTipPrebent &tip)
{
    char c[19];
    s.read(c,19);
    if(strcmp(c,"usNeedleTipPrebent"))
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleTipPrebent data");
        throw e;
    }

    s >>= (usNeedleTip&)tip;
    s.read((char*)&(tip._diameter), sizeof(double));
    s.read((char*)&(tip._length), sizeof(double));
    s.read((char*)&(tip._angle), sizeof(double));
    return s;
}

void usNeedleTipPrebent::updateTipPose()
{
    vpHomogeneousMatrix H(0,_length*sin(_angle),_length*cos(_angle),-_angle,0,0);

    m_worldMtip = m_worldMbase*H;
    m_tipPose.buildFrom(m_worldMtip);
}
