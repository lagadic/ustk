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

#ifndef __usOrientedPlane3D_h_
#define __usOrientedPlane3D_h_

#include <iostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>


class VISP_EXPORT usOrientedPlane3D
{
protected:

    vpColVector m_direction;
    vpPoseVector m_pose;

public:

    //! Constructors, destructor

    usOrientedPlane3D();
    usOrientedPlane3D(const usOrientedPlane3D &plane);
    usOrientedPlane3D(const vpPoseVector &pose);
    usOrientedPlane3D(const vpColVector &p, const vpColVector &d);
    virtual ~usOrientedPlane3D();
    const usOrientedPlane3D &operator=(const usOrientedPlane3D &plane);

    //! Parameters setters and getters

    void setPose(const vpPoseVector &pose);
    vpPoseVector getPose() const;

    void setPosition(const vpColVector &P);
    vpColVector getPosition() const;

    void setDirection(const vpColVector &D);
    vpColVector getDirection() const;

    void moveInLocalFrame(const vpHomogeneousMatrix &H);
    void moveInLocalFrame(double x, double y, double z, double tx, double ty, double tz);
    void moveInWorldFrame(const vpHomogeneousMatrix &H);
    void moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz);

    //! Data saving

        //! Text
        friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usOrientedPlane3D &plane);
        friend VISP_EXPORT std::istream &operator>>(std::istream &s, usOrientedPlane3D &plane);
        //! Binary
        friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usOrientedPlane3D &plane);
        friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usOrientedPlane3D &plane);
};


VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usOrientedPlane3D &plane);
VISP_EXPORT std::istream &operator>>(std::istream &s, usOrientedPlane3D &plane);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usOrientedPlane3D &plane);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usOrientedPlane3D &plane);

#endif // __usOrientedPlane3D_h_
