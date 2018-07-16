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

#ifndef __usNeedleInsertionModelKinematic_h
#define __usNeedleInsertionModelKinematic_h

#include <visp3/core/vpPoseVector.h>

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelInterface.h>
#include <visp3/ustk_needle_modeling/usNeedleModelBaseTip.h>


class VISP_EXPORT usNeedleInsertionModelKinematic : public usNeedleInsertionModelInterface
{

protected:

    usNeedleModelBaseTip m_needle;

    double m_naturalCurvature; //!< needle curvature parameter

public:

    //! Constructors, destructors
    usNeedleInsertionModelKinematic();
    usNeedleInsertionModelKinematic(const usNeedleInsertionModelKinematic &needle);
    ~usNeedleInsertionModelKinematic();
    usNeedleInsertionModelKinematic &operator=(const usNeedleInsertionModelKinematic &needle);
    virtual usNeedleInsertionModelKinematic* clone() const;// Polymorph copy method

    //! Parameters setters and getters

        //! Curvature

        void setNaturalCurvature(double naturalCurvature);
        double getNaturalCurvature() const;

        //! Needle

        const usNeedleModelBaseTip &accessNeedle() const;
        usNeedleModelBaseTip &accessNeedle();

    //! Control of the needle

    using usNeedleInsertionModelInterface::moveBase;
    bool moveBase(double vz, double wz, double time);
    bool moveBase(double controlCurvature, double vz, double wz, double time);

    bool setBasePose(const vpPoseVector &pose);
    vpPoseVector getBasePose() const;
};


#endif // __usNeedleInsertionModelKinematic_h
