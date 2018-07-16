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

#ifndef __usGeometryTools_h
#define __usGeometryTools_h

#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>

#include <visp3/ustk_core/usBSpline3D.h>
#include <visp3/ustk_core/usOrientedPlane3D.h>
#include <visp3/ustk_core/usPolynomialCurve3D.h>


namespace usGeometryTools
{
    VISP_EXPORT double getPointPlaneDistance(const vpColVector &point, const usOrientedPlane3D &plane);
    VISP_EXPORT bool IsPointInFrontOfPlane(const vpColVector &point, const usOrientedPlane3D &plane);
    VISP_EXPORT bool DoesSegmentCrossPlane(const usPolynomialCurve3D &poly, const usOrientedPlane3D &plane);
    VISP_EXPORT bool DoesSegmentCrossPlane(const usBSpline3D &spline, const usOrientedPlane3D &plane);
    VISP_EXPORT bool DoesSegmentCrossPlaneDirect(const usPolynomialCurve3D &poly, const usOrientedPlane3D &plane);
    VISP_EXPORT bool DoesSegmentCrossPlaneDirect(const usBSpline3D &spline, const usOrientedPlane3D &plane);
    VISP_EXPORT vpColVector getPlaneCurveCrossingPoint(const usPolynomialCurve3D&poly, const usOrientedPlane3D &plane, double threshold, double *t = nullptr);
    VISP_EXPORT vpColVector getPlaneCurveCrossingPoint(const usBSpline3D &spline, const usOrientedPlane3D &plane, double threshold, double *t = nullptr);
    VISP_EXPORT vpColVector projectPointOnPlane(const vpColVector &point, const usOrientedPlane3D &plane, const vpColVector &direction = vpColVector());
    VISP_EXPORT vpColVector projectPointOnCurve(const vpColVector &point, const usPolynomialCurve3D &poly, double threshold = -1, double *t = nullptr);
    VISP_EXPORT vpColVector projectPointOnCurve(const vpColVector &point, const usBSpline3D &spline, double threshold = -1, int *index = nullptr, double *t = nullptr);
    VISP_EXPORT usOrientedPlane3D getNormalPlane(const usPolynomialCurve3D &p, double l);

    VISP_EXPORT usPolynomialCurve3D convertBSplineToPolynomial(const usBSpline3D &spline, int order);
    VISP_EXPORT usBSpline3D convertPolynomialToBSpline(const usPolynomialCurve3D &poly, int nbSegments, int order);

    VISP_EXPORT vpPoseVector findPointCloudRelativePose(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, double *res = nullptr); // return transform from P1 to P2 (P2 = H * P1) (here P1 and P2 have the same size and are already matched one by one)
    VISP_EXPORT vpTranslationVector findPointCloudRelativePosition(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, const vpRotationMatrix &worldRprobe, double *res = nullptr); // return translation from P1 to P2 (P2 = P1 + T) (here P1 and P2 have the same size and are already matched one by one)
    VISP_EXPORT vpRotationMatrix findPointCloudRelativeRotation(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, double *res = nullptr); // return rotation from P1 to P2 (P2 = R * P1) (here P1 and P2 have the same size and are already matched one by one)

    VISP_EXPORT vpPoseVector ICPPointCloudRelativePose(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, double *res = nullptr, vpPoseVector *initialGuess = nullptr); // return transform from P1 to P2 (P2 = H * P1) (P1 and P2 can have different sizes)
    VISP_EXPORT vpTranslationVector ICPPointCloudRelativePosition(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, const vpRotationMatrix &worldRprobe, double *res = nullptr, vpTranslationVector *initialGuess = nullptr); // return translation from P1 to P2 (P2 = P1 + T) (P1 and P2 can have different sizes)
    VISP_EXPORT vpRotationMatrix ICPPointCloudRelativeRotation(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, double *res = nullptr, vpRotationMatrix *initialGuess = nullptr); // return rotation from P1 to P2 (P2 = R * P1) (P1 and P2 can have different sizes)
    
    VISP_EXPORT bool fitCircleTo2DPointCloud(const std::vector<vpColVector> &points, vpColVector &center, double &r);
    VISP_EXPORT bool fitSphereTo3DPointCloud(const std::vector<vpColVector> &points, vpColVector &center, double &r);

} //namespace usGeometryTools

#endif // __usGeometryTools_h
