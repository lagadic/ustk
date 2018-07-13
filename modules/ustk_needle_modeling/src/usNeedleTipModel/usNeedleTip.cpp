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

#include <visp3/ustk_needle_modeling/usNeedleTip.h>

#include <visp3/core/vpException.h>


usNeedleTip::usNeedleTip() :
  m_basePose(),
  m_worldMbase(),
  m_tipPose(),
  m_worldMtip()
{

}

usNeedleTip::usNeedleTip(const usNeedleTip &tip):
    m_basePose(tip.m_basePose),
    m_worldMbase(tip.m_worldMbase),
    m_tipPose(tip.m_tipPose),
    m_worldMtip(tip.m_worldMtip)
{
}

usNeedleTip::~usNeedleTip()
{
}

usNeedleTip& usNeedleTip::operator=(const usNeedleTip &tip)
{
    m_basePose = tip.m_basePose;
    m_worldMbase = tip.m_worldMbase;
    m_tipPose = tip.m_tipPose;
    m_worldMtip = tip.m_worldMtip;

    return (*this);
}

usNeedleTip* usNeedleTip::clone() const
{
    return new usNeedleTip(*this);
}

void usNeedleTip::setBasePose(const vpPoseVector &pose)
{
    m_basePose = pose;
    m_worldMbase.buildFrom(m_basePose);
    
    this->updateTipPose();
}

vpPoseVector usNeedleTip::getBasePose() const
{
    return m_basePose;
}

void usNeedleTip::setWorldMbase(const vpHomogeneousMatrix &worldMbase)
{
    m_worldMbase = worldMbase;
    m_basePose.buildFrom(m_worldMbase);
    
    this->updateTipPose();
}

vpHomogeneousMatrix usNeedleTip::getWorldMbase() const
{
    return m_worldMbase;
}

void usNeedleTip::setBasePosition(const vpColVector &position)
{
    for(int i=0 ; i<3 ; i++) m_basePose[i] = position[i];
    m_worldMbase.buildFrom(m_basePose);
    
    this->updateTipPose();
}

vpColVector usNeedleTip::getBasePosition() const
{
    vpColVector p(3);
    
    for(int i=0 ; i<3 ; i++) p[i] = m_basePose[i];
    
    return p;
}

vpPoseVector usNeedleTip::getTipPose() const
{
    return m_tipPose;
}

vpHomogeneousMatrix usNeedleTip::getWorldMtip() const
{
    return m_worldMtip;
}

vpColVector usNeedleTip::getTipPosition() const
{
    return vpColVector(m_worldMtip.getCol(3),0,3);
}

vpColVector usNeedleTip::getTipDirection() const
{
    return vpColVector(m_worldMtip.getCol(2),0,3);
}

vpColVector usNeedleTip::getBaseAxisX() const
{
    return vpColVector(m_worldMbase.getCol(0),0,3);
}

vpColVector usNeedleTip::getBaseAxisY() const
{
    return vpColVector(m_worldMbase.getCol(1),0,3);
}

vpColVector usNeedleTip::getBaseAxisZ() const
{
    return vpColVector(m_worldMbase.getCol(2),0,3);
}

std::ostream &operator<<(std::ostream &s, const usNeedleTip &tip)
{
    s << "usNeedleTip\n";
    s << tip.m_basePose << '\n';
    s << tip.m_tipPose << '\n';
    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usNeedleTip &tip)
{
    std::string c;
    s >> c;
    if(c != "usNeedleTip")
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleTip data");
        throw e;
    }
    for(int i=0 ; i<6 ; i++) s >> tip.m_basePose[i];
    for(int i=0 ; i<6 ; i++) s >> tip.m_tipPose[i];
    s.get();
    tip.m_worldMbase.buildFrom(tip.m_basePose);
    tip.m_worldMtip.buildFrom(tip.m_tipPose);

    return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleTip &tip)
{
    s.write("usNeedleTip",12);
    for(int i=0 ; i<6 ; i++) s.write((char*)&(tip.m_basePose[i]), sizeof(double));
    for(int i=0 ; i<6 ; i++) s.write((char*)&(tip.m_tipPose[i]), sizeof(double));
    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usNeedleTip &tip)
{
    char c[12];
    s.read(c,12);
    if(strcmp(c,"usNeedleTip"))
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleTip data");
        throw e;
    }
    for(int i=0 ; i<6 ; i++) s.read((char*)&(tip.m_basePose[i]), sizeof(double));
    for(int i=0 ; i<6 ; i++) s.read((char*)&(tip.m_tipPose[i]), sizeof(double));
    tip.m_worldMbase.buildFrom(tip.m_basePose);
    tip.m_worldMtip.buildFrom(tip.m_tipPose);
    return s;
}

void usNeedleTip::updateTipPose()
{
    m_tipPose = m_basePose;
    m_worldMtip = m_worldMtip;
}
