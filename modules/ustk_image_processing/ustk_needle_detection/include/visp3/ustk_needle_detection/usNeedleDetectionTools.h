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
 * Authors:
 * Pierre Chatelain
 * Alexandre Krupa
 *
 *****************************************************************************/

#ifndef __usNeedleDetectionTools_h_
#define __usNeedleDetectionTools_h_

#include <algorithm>
#include <float.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

// ViSP
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpList.h>
#include <visp3/core/vpMatrix.h>

// UsTK
#include <visp3/ustk_core/usConfig.h>

/**
 * @namespace usNeedleDetectionTools
 * @brief Basic tools for needle detection.
 * @ingroup module_ustk_needle_detection
 *
 * This class implements various tools for 3D point cloud processing.
 *
 */
namespace usNeedleDetectionTools
{
/**
   * Compute the principal angle between two vectors.
   */
double angle(double *p1, double *p2, double *q1, double *q2);

/**
   * Approximate curvilinear coordinates.
   *
   * @param X The points for which to compute the coordinates.
   * @param MSS The control points of the curve.
   * @param s The order of the polynomial curve.
   * @return The Vandermonde matrix built from the approximate curvilinear coordinates of X.
   *
   * Computes the approximate curvilinear coordinates of the points X for the model MSS.
   *
   * Given \f$X\in\mathbb{R}^{n\times d}\f$, and \f$MSS\in\mathbb{R}^{s\times d}\f$,
   * the approximate coordinate of \f$X_i\f$ for \f$i\in[1,n]\f$ is defined as
   * \f$\hat{a}_i=\frac{(X_i-MSS_1)\cdot \mathbf{k}_0}{||\mathbf{k}_0||}\f$,
   * where \f$k_0\f$ is the principal direction vector of the needle.
   *
   * The result is a Vandermonde matrix built from the coordinates of all points in \f$X\f$:
   * \f[
   * A = \left[
   * \begin{array}{ccc}
   * 1 & \cdots & 1\\
   * \hat{a}_1 & \cdots & \hat{a}_n\\
   * \vdots & & \vdots\\
   * \hat{a}_1^{s-1} & \cdots & \hat{a}_n^{s-1}
   * \end{array}
   * \right]
   * \f]
   */
vpMatrix approximateCoordinates(vpMatrix X, vpMatrix MSS, unsigned int s);

/**
   * Compute the mean (center of mass) of a set of d-dimensional points.
   * @param points The input points.
   * @param mean Pointer to hold the result.
   * @param npts Number of points.
   * @param d Dimension.
   */
void arithmeticMean(const vpMatrix points, double *mean, unsigned int npts, unsigned int d);

void computeQuantile(unsigned char *data, unsigned int dataSize, unsigned int nDesired, unsigned int &q,
                     unsigned int &nThresholded);
void computeQuantile(short *data, unsigned int dataSize, unsigned int nDesired, unsigned int &q,
                     unsigned int &nThresholded);
void computeQuantile(const unsigned int *data, unsigned int dataSize, unsigned int nDesired, unsigned int &q,
                     unsigned int &nThresholded);

void computeControlPoints(const vpMatrix &model, vpMatrix &mss, double **points, const unsigned int &nPoints);

/**
   * Compute the euclidean distance between two 3D points.
   */
double dist3(double *x, double *y);

bool findEntry(const vpMatrix &model, double *entry, unsigned int nPoints, const vpColVector &origin,
               const vpColVector &entryPlane, int VOI[6]);

/**
   * Compute the geometric median of a set of d-dimensional points.
   * @param points The input points.
   * @param npts Number of points.
   * @param d Dimension.
   */
vpColVector geometricMedian(const vpMatrix points, unsigned int npts, unsigned int d);

bool inside(const vpColVector point, int *VOI);

/**
   * Get the intersection between a plane and a straight line.
   */
bool intersectionLinePlane(vpMatrix line, const vpColVector &plane, double offset, double &x, double &y, double &z);

/**
   * Performs linear regression on a set of 3D points.
   */
void linearRegression(vpMatrix &points, double &x, double &y, double &z, double &u, double &v, double &w);
void linearRegression(vpMatrix &points, double &a, double &b, double &u, double &v, double &w);

short quantile(short *data, unsigned int num, unsigned int n);

/**
   * Compute the variance of a set of d-dimensional points.
   */
double variance(vpMatrix points, unsigned int npts, unsigned int d);
}

#endif // US_NEEDLE_DETECTION_TOOLS_H
