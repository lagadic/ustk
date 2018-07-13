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

#include <visp3/ustk_needle_modeling/usNeedleTipSymmetric.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>


usNeedleTipSymmetric::usNeedleTipSymmetric() :
    usNeedleTip(),
    _diameter(0.001),
    _length(0.001)
{

}

usNeedleTipSymmetric::usNeedleTipSymmetric(const usNeedleTipSymmetric &tip):
    usNeedleTip(tip),
    _diameter(tip._diameter),
    _length(tip._length)
{
}

usNeedleTipSymmetric::~usNeedleTipSymmetric()
{
}

usNeedleTipSymmetric& usNeedleTipSymmetric::operator=(const usNeedleTipSymmetric &tip)
{
    this->usNeedleTip::operator=(tip);

    _diameter = tip._diameter;
    _length = tip._length;

    return (*this);
}

usNeedleTipSymmetric* usNeedleTipSymmetric::clone() const
{
    return new usNeedleTipSymmetric(*this);
}

void usNeedleTipSymmetric::setDiameter(double diameter)
{
    if(diameter > 0) _diameter = diameter;
}

double usNeedleTipSymmetric::getDiameter() const
{
    return _diameter;
}

void usNeedleTipSymmetric::setLength(double l)
{
    if(l >= 0) _length = l;
}

double usNeedleTipSymmetric::getLength() const
{
    return _length;
}

double usNeedleTipSymmetric::getAngle() const
{
    return atan2(_diameter/2,_length);
}

std::ostream &operator<<(std::ostream &s, const usNeedleTipSymmetric &tip)
{
    s << "usNeedleTipSymmetric\n";
    s << (const usNeedleTip&)tip;

    s << tip._diameter << '\n';
    s << tip._length << '\n';

    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usNeedleTipSymmetric &tip)
{
    std::string c;
    s >> c;
    if(c != "usNeedleTipSymmetric")
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleTipSymmetric data");
        throw e;
    }
    s >> (usNeedleTip&)tip;
    s >> tip._diameter;
    s >> tip._length;
    s.get();

    return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleTipSymmetric &tip)
{
    s.write("usNeedleTipSymmetric",21);

    s <<= (const usNeedleTip&)tip;
    s.write((char*)&(tip._diameter), sizeof(double));
    s.write((char*)&(tip._length), sizeof(double));

    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usNeedleTipSymmetric &tip)
{
    char c[21];
    s.read(c,21);
    if(strcmp(c,"usNeedleTipSymmetric"))
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleTipSymmetric data");
        throw e;
    }

    s >>= (usNeedleTip&)tip;
    s.read((char*)&(tip._diameter), sizeof(double));
    s.read((char*)&(tip._length), sizeof(double));
    return s;
}

void usNeedleTipSymmetric::updateTipPose()
{
    vpHomogeneousMatrix H(0,0,_length,0,0,0);

    m_worldMtip = m_worldMbase*H;
    m_tipPose.buildFrom(m_worldMtip);
}
