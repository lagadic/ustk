/****************************************************************************
 *
 * This file is part of the UsNeedleDetection software.
 * Copyright (C) 2013 - 2016 by Inria. All rights reserved.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Pierre Chatelain
 * Alexandre Krupa
 *
 *****************************************************************************/

#ifndef US_NEEDLE_DETECTION_TOOLS_H
#define US_NEEDLE_DETECTION_TOOLS_H

#include <stdio.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <float.h>


// ViSP
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpList.h>


// VTK
#if defined USNEEDLEDETECTION_HAVE_VTK
#include <vtkDataSet.h>
#include <vtkDataArray.h>
#include <vtkShortArray.h>
#include <vtkPointData.h>
#include <vtkImageData.h>
#include <vtkUnsignedIntArray.h>
#endif

/**
 * @namespace usNeedleDetectionTools
 * @brief Basic tools for needle detection.
 * @author Pierre Chatelain
 * @date 2013-09-09
 *
 * This class implements diverse tools for 3D point cloud processing.
 *
 */
namespace usNeedleDetectionTools
{
  /**
   * Compute the mean (center of mass) of a set of d-dimensional points.
   * @param points The input points.
   * @param mean Pointer to hold the result.
   * @param npts Number of points.
   * @param d Dimension.
   */
  void arithmeticMean(const vpMatrix points, double *mean, unsigned int npts,
			     unsigned int d);

  /**
   * Compute the geometric median of a set of d-dimensional points.
   * @param points The input points.
   * @param npts Number of points.
   * @param d Dimension.
   */
  vpColVector geometricMedian(const vpMatrix points, unsigned int npts,
			      unsigned int d);

  /**
   * Compute the variance of a set of d-dimensional points.
   */
  double variance(vpMatrix points, unsigned int npts, unsigned int d);

  short quantile(short *data, unsigned int num, unsigned int n);

#if defined USNEEDLEDETECTION_HAVE_VTK
  short quantile(vtkDataSet *data, unsigned int num);
#endif

  void computeQuantile(unsigned char *data, unsigned int dataSize,
		       unsigned int nDesired, unsigned int &q,
		       unsigned int &nThresholded);
  void computeQuantile(short *data, unsigned int dataSize,
		       unsigned int nDesired, unsigned int &q,
		       unsigned int &nThresholded);
  void computeQuantile(const unsigned int *data, unsigned int dataSize,
		       unsigned int nDesired, unsigned int &q,
		       unsigned int &nThresholded);
  bool findEntry(const vpMatrix &model, double *entry, unsigned int nPoints,
		 const vpColVector &origin, const vpColVector &entryPlane, int VOI[6]);

#if defined USNEEDLEDETECTION_HAVE_VTK
  int findTip(vtkDataArray *data, unsigned int dataSize,
			      double gap);
  bool findTip(vtkImageData *image, const vpMatrix &model, int *VOI, double *tip, unsigned int nPoints, double gap);
  bool findTip(vtkImageData *image, const vpMatrix &model, int *VOI, double *tip, unsigned int nPoints, double gap, double length);
  bool findTipUsingMeanValues(vtkImageData *image, const vpMatrix &model, int *VOI, double *tip, unsigned int nPoints, double gap, double length);
#endif

  bool inside(const vpColVector point, int *VOI);

#if defined USNEEDLEDETECTION_HAVE_VTK
  unsigned int findTip(vtkDataArray *data, unsigned int dataSize,
			      double threshL, double threshU);
  unsigned int findTip(vtkDataArray *data, unsigned int dataSize,
			      double threshL, double threshU, int prediction);
  vpList<double*> getThresholdedIds(vtkImageData *image,
				    double threshold);
#endif

#if defined USNEEDLEDETECTION_HAVE_VTK
  /**
   * Get the coordinates of the points of highest intensity.
   *
   * @param image The image.
   * @param points [out] The points of intensity higher than the threshold.
   * @param nDesired The desired number of points.
   *
   * @return The threshold.
   */
  void getThresholdedCoordinates(vtkImageData *image, vpMatrix &points, double threshold);
#endif
#if 0
  /**
   * Get the coordinates of the points of highest intensity.
   *
   * @param V The volume.
   * @param points [out] The points of intensity higher than the threshold.
   * @param nDesired The desired number of points.
   *
   * @return The threshold.
   */
  double getThresholdedCoordinates(const usVolume<unsigned int> &V,
				   vpMatrix &points,
				   unsigned int nDesired);
#endif
#if defined USNEEDLEDETECTION_HAVE_VTK
  /**
   * Get the coordinates of the points of highest intensity.
   *
   * @param image The image.
   * @param points [out] The points of intensity higher than the threshold.
   * @param nDesired The desired number of points.
   *
   * @return The threshold.
   */
  double getThresholdedCoordinates(vtkImageData *image,
				   vpMatrix &points,
				   unsigned int nDesired);
#endif

  /**
   * Compute the euclidean distance between two 3D points.
   */
  double dist3(double *x, double *y);

  /**
   * Compute the principal angle between two vectors.
   */
  double angle(double *p1, double *p2, double *q1, double *q2);

  /**
   * Performs linear regression on a set of 3D points.
   */
  void linearRegression(vpMatrix &points, double &x, double &y, double &z,
			double &u, double &v, double &w);
  void linearRegression(vpMatrix &points, double &a, double &b, double &u, double &v, double &w);
  void computeControlPoints(const vpMatrix &model, vpMatrix &mss, double **points,
			    const unsigned int &nPoints);
		
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
   * Get the intersection between a plane and a straight line.
   */
  bool intersectionLinePlane(vpMatrix line, const vpColVector &plane, double offset,
			     double &x, double &y, double &z);
};

#endif // US_NEEDLE_DETECTION_TOOLS_H
