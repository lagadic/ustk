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

#ifndef __usPolynomialCurve2D_h_
#define __usPolynomialCurve2D_h_

#include <visp3/core/vpMatrix.h>

#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>

/**
 * This class represents a needle modeled as a 2D polynomial curve.
 *
 * @brief 2D needle model
 *
 */
class VISP_EXPORT usPolynomialCurve2D {
 public:
  /**
   * Default constructor. Requires a call to setOrder().
   */
  usPolynomialCurve2D();

  /**
   * Copy constructor.
   */
  usPolynomialCurve2D(const usPolynomialCurve2D &needle);

  /**
   * Constructor.
   *
   * @param order The order of the polynomial curve representing the needle.
   */
  usPolynomialCurve2D(unsigned int order);

  /**
   * Change the order of the polynomial representing the needle.
   */
  usPolynomialCurve2D changePolynomialOrder(unsigned int newOrder);

  /**
   * Compute the distance between two needles.
   */
  static double curveDistance(const usPolynomialCurve2D &n1, const usPolynomialCurve2D &n2);

  /**
   * Get the control points.
   */
  vpMatrix getControlPoints() const;

  /**
   * Get the needle curvature.
   */
  double getCurvature();

  /**
   * Get the needle length.
   */
  double getLength() const;

  /**
   * Get the needle model.
   */
  const vpMatrix *getModel() const;

  /**
   * Get the number of lines used to display the needle.
   */
  unsigned int getNumberOfRenderingLines() const;

  /**
   * Get the order of the polynomial curve representing the needle.
   */
  unsigned int getOrder() const;

  /**
   * Get the needle coordinates at a given curvilinear abscissa.
   */
  vpColVector getPoint(double abscissa) const;

  /**
   * Get the rendering points.
   */
  vpMatrix getRenderingPoints() const;

  /**
   * Get the tangent vector at a given curvilinear abscissa.
   */
  vpColVector getTangent(double abscissa) const;

  /**
   * Set the control points.
   * The input matrix has to be of size nx3, where n is the order of the model.
   *
   * @param controlPoints Reference to the desired control points.
   */
  void setControlPoints(const vpMatrix &controlPoints);

  /**
   * Set the control points.
   */
  void setControlPoints(double **controlPoints);

  /**
   * Set the needle model.
   */
  void setModel(const vpMatrix &model);

  /**
   * Set the number of lines used to diplay the needle.
   */
  void setNumberOfRenderingLines(unsigned int nRenderingLines);

  /**
   * Set the order of the polynomial curve representing the needle.
   */
  void setOrder(unsigned int order);

 private:
  unsigned int m_order;
  unsigned int m_nRenderingLines;
  vpMatrix m_controlPoints;
  vpMatrix m_model;
  vpMatrix m_renderingPoints;
};

#endif // USPOLYNOMIALCURVE2D_H
