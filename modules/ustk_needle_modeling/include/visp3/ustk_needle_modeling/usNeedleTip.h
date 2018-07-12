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

#ifndef __usNeedleTip_h
#define __usNeedleTip_h

#include <iostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>


class VISP_EXPORT usNeedleTip
{
protected:
    vpPoseVector m_basePose;
    vpHomogeneousMatrix m_worldMbase;
    vpPoseVector m_tipPose;
    vpHomogeneousMatrix m_worldMtip;
    
public:

    //! Constructors, destructors

    usNeedleTip();
    usNeedleTip(const usNeedleTip &needle);
    virtual ~usNeedleTip();
    virtual usNeedleTip &operator=(const usNeedleTip &needle);
    virtual usNeedleTip* clone() const;  // Polymorph copy method

    //! Parameters setters and getters
    
    void setBasePose(const vpPoseVector &pose);
    vpPoseVector getBasePose() const;
    void setWorldMbase(const vpHomogeneousMatrix &worldMbase);
    vpHomogeneousMatrix getWorldMbase() const;
    void setBasePosition(const vpColVector &position);
    vpColVector getBasePosition() const;
    
    vpPoseVector getTipPose() const;
    vpHomogeneousMatrix getWorldMtip() const;
    vpColVector getTipPosition() const;
    vpColVector getTipDirection() const;
    
    vpColVector getBaseAxisX() const;
    vpColVector getBaseAxisY() const;
    vpColVector getBaseAxisZ() const;

    //! Data saving

        //! Text
        friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleTip &tip);
        friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleTip &tip);
        //! Binary
        friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleTip &tip);
        friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleTip &tip);
        
private:
        
    virtual void updateTipPose();
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleTip &tip);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleTip &tip);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleTip &tip);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleTip &tip);

#endif // __usNeedleTip_h
