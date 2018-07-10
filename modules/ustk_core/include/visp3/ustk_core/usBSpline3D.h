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

#ifndef usBSpline3D_h
#define usBSpline3D_h

#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

#include <visp3/ustk_core/usPolynomialCurve3D.h>


class VISP_EXPORT usBSpline3D
{
protected:

    //! Polynomials container

    std::vector<usPolynomialCurve3D> m_spline;

public:

    //! Constructors, destructor

    usBSpline3D();
    usBSpline3D(const usBSpline3D &spline);
    virtual ~usBSpline3D();
    const usBSpline3D &operator=(const usBSpline3D &spline);

    virtual usBSpline3D* clone() const; // Polymorph copy method

    //! Parameters setters and getters

        int getNbSegments() const;
        double getParametricLength() const;
        double getLength(int nbSubSeg=50) const;

    //! Spline definition

        void addSegment(const usPolynomialCurve3D &seg);
        void insertSegment(int i, const usPolynomialCurve3D &seg);
        void setSegment(int i, const usPolynomialCurve3D &poly);
        void removeLastSegment();
        void removeSegment(int i);
        void removeSegments(int i, int j);
        void clear();

        void defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &lengths, int order = 3);

        const usPolynomialCurve3D &accessSegment(int i) const;
        const usPolynomialCurve3D &accessLastSegment() const;
        usPolynomialCurve3D &accessSegment(int i);
        usPolynomialCurve3D &accessLastSegment();
        usBSpline3D getSubSpline(double a, double b) const;

    //! Move

        bool move(const vpHomogeneousMatrix &H);
        bool move(double x, double y, double z, double tx, double ty, double tz);

    //! Measure curve information

        //! Position

        vpColVector getPoint(double param) const;
        vpColVector getTangent(double param) const;
        double getDistanceFromPoint(const vpColVector &point, double start = 0, double stop = -1, double threshold = 1e-5) const;
        bool getParametersFromLength(double l, int &index, double &param) const;

        //! Curvature

        double getCurvatureFromShape(double start, double end, vpColVector &center3D, vpColVector &direction3D) const;

        friend std::ostream &operator<<(std::ostream &s, const usBSpline3D &needle);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usBSpline3D &needle);

#endif // usBSpline3D_h
