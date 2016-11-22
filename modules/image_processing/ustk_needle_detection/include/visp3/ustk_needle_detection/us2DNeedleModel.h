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

#ifndef US2DNEEDLEMODEL_H
#define US2DNEEDLEMODEL_H

#include <visp/vpMatrix.h>

#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>

/**
 * This class represents a needle modeled as a 2D polynomial curve.
 *
 * @brief 2D needle model
 * @author Pierre Chatelain
 * @date 2013-09-11
 *
 */
class VISP_EXPORT us2DNeedleModel {
 public:
  /**
   * Default constructor. Requires a call to setOrder().
   */
  us2DNeedleModel();

  /**
   * Copy constructor.
   */
  us2DNeedleModel(const us2DNeedleModel &needle);

  /**
   * Constructor.
   *
   * @param order The order of the polynomial curve representing the needle.
   */
  us2DNeedleModel(unsigned int order);

  /**
   * Set the order of the polynomial curve representing the needle.
   */
  void setOrder(unsigned int order);

  /**
   * Get the order of the polynomial curve representing the needle.
   */
  unsigned int getOrder() const;

  /**
   * Set the number of lines used to diplay the needle.
   */
  void setNumberOfRenderingLines(unsigned int nRenderingLines);

  /**
   * Get the number of lines used to display the needle.
   */
  unsigned int getNumberOfRenderingLines() const;

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
   * Get the control points.
   */
  vpMatrix getControlPoints() const;

  /**
   * Get the rendering points.
   */
  vpMatrix getRenderingPoints() const;

  /**
   * Set the needle model.
   */
  void setModel(const vpMatrix &model);

  /**
   * Get the needle model.
   */
  vpMatrix *getModel();

  /**
   * Get the needle coordinates at a given curvilinear abscissa.
   */
  vpColVector getPoint(double abscissa) const;

  /**
   * Get the tangent vector at a given curvilinear abscissa.
   */
  vpColVector getTangent(double abscissa) const;

  /**
   * Get the needle length.
   */
  double getLength() const;

  /**
   * Get the needle curvature.
   */
  double getCurvature();

  /**
   * Compute the distance between two needles.
   */
  static double curveDistance(us2DNeedleModel &n1, us2DNeedleModel &n2);

  /**
   * Change the order of the polynomial representing the needle.
   */
  us2DNeedleModel changePolynomialOrder(unsigned int newOrder);

 private:
  unsigned int m_order;
  unsigned int m_nRenderingLines;
  vpMatrix m_controlPoints;
  vpMatrix m_model;
  vpMatrix m_renderingPoints;
};

#endif // US2DNEEDLEMODEL_H
