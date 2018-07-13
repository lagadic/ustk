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

#include <visp3/ustk_core/usOrientedPlane3D.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpThetaUVector.h>


usOrientedPlane3D::usOrientedPlane3D():
    m_direction(3,0),
    m_pose(0,0,0,0,0,0)
{
}

usOrientedPlane3D::usOrientedPlane3D(const usOrientedPlane3D &plane):
    m_direction(plane.m_direction),
    m_pose(plane.m_pose)
{
}

usOrientedPlane3D::usOrientedPlane3D(const vpPoseVector &pose): usOrientedPlane3D()
{
    this->setPose(pose);
}

usOrientedPlane3D::usOrientedPlane3D(const vpColVector &p, const vpColVector &d): usOrientedPlane3D()
{
    this->setPosition(p);
    this->setDirection(d);
}

usOrientedPlane3D:: ~usOrientedPlane3D()
{
}

const usOrientedPlane3D &usOrientedPlane3D::operator=(const usOrientedPlane3D &plane)
{
    m_direction = plane.m_direction;
    m_pose = plane.m_pose;
    
    return *this;
}

void usOrientedPlane3D::setPose(const vpPoseVector &pose)
{
    m_pose = pose;
    m_direction = ((vpMatrix)vpRotationMatrix(pose)).getCol(2,0,3).normalize();
}

vpPoseVector usOrientedPlane3D::getPose() const
{
    return m_pose;
}

void usOrientedPlane3D::setPosition(const vpColVector &P)
{
    if(P.size()==3)
    {
        for(int i=0 ; i<3 ; i++) m_pose[i] = P[i];
    }
}

vpColVector usOrientedPlane3D::getPosition() const
{
    vpColVector P(3);
    
    for(int i=0 ; i<3 ; i++) P[i] = m_pose[i];
    
    return P;
}

void usOrientedPlane3D::setDirection(const vpColVector &D)
{
    if(D.size()!=3) return;

    if(D.euclideanNorm()>std::numeric_limits<double>::epsilon())
    {
        if(m_direction.euclideanNorm()==0) m_direction[2] = 1;
        double cos = vpColVector::dotProd(m_direction, D);
        vpColVector u = vpColVector::crossProd(m_direction, D);
        double sin = u.euclideanNorm();
        double theta = atan2(sin, cos);
        u = theta * u.normalize();

        vpThetaUVector tu(vpRotationMatrix(u[0], u[1], u[2])*vpRotationMatrix(m_pose));
        for(int i=0 ; i<3 ; i++) m_pose[3+i] = tu[i];
        m_direction = D;
        m_direction.normalize();
    }
    else
    {
        m_direction = 0;
        for(int i=3 ; i<6 ; i++) m_pose[i] = 0;
    }
}

vpColVector usOrientedPlane3D::getDirection() const
{
    return m_direction;
}

void usOrientedPlane3D::moveInLocalFrame(const vpHomogeneousMatrix &H)
{
    this->setPose(vpPoseVector(vpHomogeneousMatrix(m_pose)*H));
}

void usOrientedPlane3D::moveInLocalFrame(double x, double y, double z, double tx, double ty, double tz)
{
    this->moveInLocalFrame(vpHomogeneousMatrix(x,y,z,tx,ty,tz));
}

void usOrientedPlane3D::moveInWorldFrame(const vpHomogeneousMatrix &H)
{
    this->setPose(vpPoseVector(H*vpHomogeneousMatrix(m_pose)));
}

void usOrientedPlane3D::moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz)
{
    this->moveInWorldFrame(vpHomogeneousMatrix(x,y,z,tx,ty,tz));
}

std::ostream &operator<<(std::ostream &s, const usOrientedPlane3D &plane)
{
    s << "usOrientedPlane3D\n";
    s << plane.m_pose[0];
    for(int i=1 ; i<6 ; i++) s  << " " << plane.m_pose[i];
    s << '\n';
    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usOrientedPlane3D &plane)
{
    std::string c;
    s >> c;
    if(c != "usOrientedPlane3D")
    {
        vpException e(vpException::ioError, "operator>>=(std::istream&, usOrientedPlane3D&): Stream does not contain usOrientedPlane3D data");
        throw e;
    }
    vpPoseVector pose;
    for(int i=0 ; i<6 ; i++) s >> pose[i];
    s.get();
    plane.setPose(pose);
    return s;
}

std::ostream &operator<<=(std::ostream &s, const usOrientedPlane3D &plane)
{
    s.write("usOrientedPlane3D",18);
    for(int i=0 ; i<6 ; i++) s.write((char*)&(plane.m_pose[i]), sizeof(double));
    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usOrientedPlane3D &plane)
{
    char c[18];
    s.read(c,18);
    if(strcmp(c,"usOrientedPlane3D"))
    {
        vpException e(vpException::ioError, "operator>>=(std::istream&, usOrientedPlane3D&): Stream does not contain usOrientedPlane3D data");
        throw e;
    }
    vpPoseVector pose;
    for(int i=0 ; i<3 ; i++) s.read((char*)&(pose[i]), sizeof(double));
    plane.setPose(pose);
    return s;
}
