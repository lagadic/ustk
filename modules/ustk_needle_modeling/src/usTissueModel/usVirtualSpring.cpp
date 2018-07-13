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

#include <visp3/ustk_needle_modeling/usVirtualSpring.h>


usVirtualSpring::usVirtualSpring():
    usOrientedPlane3D(),
    m_stiffness(0),

    m_IsActive(true),
    m_IsRemovable(true),
    m_AllowPositionUpdate(true),
    m_AllowDirectionUpdate(true),
    m_AllowStiffnessUpdate(true)
{

}

usVirtualSpring::usVirtualSpring(const usVirtualSpring &spring):
    usOrientedPlane3D(spring),

    m_stiffness(spring.m_stiffness),

    m_IsActive(spring.m_IsActive),
    m_IsRemovable(spring.m_IsRemovable),
    m_AllowPositionUpdate(spring.m_AllowPositionUpdate),
    m_AllowDirectionUpdate(spring.m_AllowDirectionUpdate),
    m_AllowStiffnessUpdate(spring.m_AllowStiffnessUpdate)
{

}

usVirtualSpring::usVirtualSpring(const vpColVector &p, const vpColVector &d, double K):
    usOrientedPlane3D(p,d),
    m_stiffness(0),

    m_IsActive(true),
    m_IsRemovable(true),
    m_AllowPositionUpdate(true),
    m_AllowDirectionUpdate(true),
    m_AllowStiffnessUpdate(true)
{
    this->setStiffness(K);
}

usVirtualSpring:: ~usVirtualSpring()
{

}

const usVirtualSpring &usVirtualSpring::operator=(const usVirtualSpring &spring)
{
    this->usOrientedPlane3D::operator=(spring);
    m_stiffness = spring.m_stiffness;

    m_IsActive = spring.m_IsActive;
    m_IsRemovable = spring.m_IsRemovable;
    m_AllowPositionUpdate = spring.m_AllowPositionUpdate;
    m_AllowDirectionUpdate = spring.m_AllowDirectionUpdate;
    m_AllowStiffnessUpdate = spring.m_AllowStiffnessUpdate;
    
    return *this;
}

void usVirtualSpring::setStiffness(double K)
{
    if(K>0) m_stiffness = K;
}

double usVirtualSpring::getStiffness() const
{
    return m_stiffness;
}

void usVirtualSpring::addStiffness(double dK)
{
    if(m_stiffness+dK>0) m_stiffness += dK;
}

void usVirtualSpring::Activate()
{
    m_IsActive = true;
}

void usVirtualSpring::Deactivate()
{
    m_IsActive = false;
}

bool usVirtualSpring::IsActive() const
{
    return m_IsActive;
}

void usVirtualSpring::AllowRemoval(bool flag)
{
    m_IsRemovable = flag;
}

bool usVirtualSpring::IsRemovable() const
{
    return m_IsRemovable;
}

void usVirtualSpring::AllowPositionUpdate(bool flag)
{
    m_AllowPositionUpdate = flag;
}

bool usVirtualSpring::IsPositionUpdateAllowed() const
{
    return m_AllowPositionUpdate;
}

void usVirtualSpring::AllowDirectionUpdate(bool flag)
{
    m_AllowDirectionUpdate = flag;
}

bool usVirtualSpring::IsDirectionUpdateAllowed() const
{
    return m_AllowDirectionUpdate;
}

void usVirtualSpring::AllowStiffnessUpdate(bool flag)
{
    m_AllowStiffnessUpdate = flag;
}

bool usVirtualSpring::IsStiffnessUpdateAllowed() const
{
    return m_AllowStiffnessUpdate;
}

std::ostream &operator<<(std::ostream &s, const usVirtualSpring &spg)
{
    s << "usVirtualSpring\n";

    s << *((usOrientedPlane3D*)&spg);

    s << spg.m_stiffness << '\n';
    s << spg.m_IsActive << '\n';
    s << spg.m_IsRemovable << '\n';
    s << spg.m_AllowPositionUpdate << '\n';
    s << spg.m_AllowDirectionUpdate << '\n';
    s << spg.m_AllowStiffnessUpdate << '\n';
    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usVirtualSpring &spg)
{
    std::string c;
    s >> c;
    if(c != "usVirtualSpring")
    {
        vpException e(vpException::ioError, "operator>>=(std::istream&, usVirtualSpring&): Stream does not contain usVirtualSpring data");
        throw e;
    }

    s >> *((usOrientedPlane3D*)&spg);

    s >> spg.m_stiffness;
    s >> spg.m_IsActive;
    s >> spg.m_IsRemovable;
    s >> spg.m_AllowPositionUpdate;
    s >> spg.m_AllowDirectionUpdate;
    s >> spg.m_AllowStiffnessUpdate;
    s.get();
    
    return s;
}

std::ostream &operator<<=(std::ostream &s, const usVirtualSpring &spg)
{
    s.write("usVirtualSpring",16);
    s <<= *((usOrientedPlane3D*)&spg);
    s.write((char*)&(spg.m_stiffness), sizeof(double));
    s.write((char*)&(spg.m_IsActive), sizeof(bool));
    s.write((char*)&(spg.m_IsRemovable), sizeof(bool));
    s.write((char*)&(spg.m_AllowPositionUpdate), sizeof(bool));
    s.write((char*)&(spg.m_AllowDirectionUpdate), sizeof(bool));
    s.write((char*)&(spg.m_AllowStiffnessUpdate), sizeof(bool));
    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usVirtualSpring &spg)
{
    char c[16];
    s.read(c,16);
    if(strcmp(c,"usVirtualSpring"))
    {
        vpException e(vpException::ioError, "operator>>=(std::istream&, usVirtualSpring&): Stream does not contain usVirtualSpring data");
        throw e;
    }
    s >>= *((usOrientedPlane3D*)&spg);

    s.read((char*)&(spg.m_stiffness), sizeof(double));
    s.read((char*)&(spg.m_IsActive), sizeof(bool));
    s.read((char*)&(spg.m_IsRemovable), sizeof(bool));
    s.read((char*)&(spg.m_AllowPositionUpdate), sizeof(bool));
    s.read((char*)&(spg.m_AllowDirectionUpdate), sizeof(bool));
    s.read((char*)&(spg.m_AllowStiffnessUpdate), sizeof(bool));
    return s;
}
