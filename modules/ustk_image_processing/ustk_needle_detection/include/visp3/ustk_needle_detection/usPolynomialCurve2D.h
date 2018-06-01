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
 * Jason Chevrie
 *
 *****************************************************************************/

#ifndef __usPolynomialCurve2D_h_
#define __usPolynomialCurve2D_h_

#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

/**
 * @class usPolynomialCurve2D
 * @brief 2D curve model
 * @ingroup module_ustk_needle_detection
 *
 * This class represents a 2D polynomial curve.
 *
 */
class VISP_EXPORT usPolynomialCurve2D
{
public:
  /**
   * Default constructor. Requires a call to setOrder().
   */
  usPolynomialCurve2D();

  /**
   * Copy constructor.
   */
  usPolynomialCurve2D(const usPolynomialCurve2D &curve);
  
  /**
   * Assignment operator.
   */
  const usPolynomialCurve2D &operator=(const usPolynomialCurve2D &curve);
  
  /**
   * Destructor.
   */
  virtual ~usPolynomialCurve2D();

  /**
   * Constructor.
   *
   * @param order The order of the polynomial curve.
   */
  usPolynomialCurve2D(unsigned int order);

  /**
   * Change the order of the polynomial curve.
   */
  void setOrder(unsigned int order);

  /**
   * Get the order of the polynomial curve.
   */
  unsigned int getOrder() const;

  /**
   * Set the starting value of the parametric variable of the polynomial curve.
   */
  void setStartParameter(double startParameter);
  
  /**
   * Get the starting value of the parametric variable of the polynomial curve.
   */
  double getStartParameter() const;
  
  /**
   * Set the ending value of the parametric variable of the polynomial curve.
   */
  void setEndParameter(double endParameter);
  
  /**
   * Get the ending value of the parametric variable of the polynomial curve.
   */
  double getEndParameter() const;

  /**
   * Set the starting and ending values of the parametric variable of the polynomial curve.
   * If the starting value is higher than the ending value, these values are inverted as well as the direction of displacement along the curve.
   */
  void setBoundaries(double startParameter, double endParamter);
  
  /**
   * Set the length of the curve in the parametric variable space.
   */
  void setParametricLength(double length);
  
  /**
   * Get the length of the curve in the parametric variable space.
   */
  double getParametricLength() const;
  
  /**
   * Set the curve length in metric space.
   */
  void setLength(double length, double precision=1e-4);
  
  /**
   * Get the curve length in metric space.
   */
  double getLength(int nbCountSeg=50) const;

  /**
   * Set the curve polynomial coefficients.
   */
  void setPolynomialCoefficients(const vpMatrix &polynomialCoefficients);
  
  /**
   * Get the polynomial coefficients.
   */
  vpMatrix getPolynomialCoefficients() const;
    
  /**
   * Get the polynomial curve point at a given parametric value.
   */
  vpColVector getPoint(double parameter) const;
  
  /**
   * Get the polynomial curve points at given parametric values.
   */
  vpMatrix getPoints(vpColVector parameters) const;

  /**
   * Get the starting extremity of the polynomial curve.
   */
  vpColVector getStartPoint() const;
  
  /**
   * Get the ending extremity of the polynomial curve.
   */
  vpColVector getEndPoint() const;
  
  /**
   * Get the tangent vector at a given parametric value.
   */
  vpColVector getTangent(double parameter) const;
  
  /**
   * Get the tangent vector at the starting extremity of the polynomial curve.
   */
  vpColVector getStartTangent() const;
  
  /**
   * Get the tangent vector at the starting extremity of the polynomial curve.
   */
  vpColVector getEndTangent() const;
  
  /**
   * Get the derivative of the polynmial curve at a given parametric value.
   */
  vpColVector getDerivative(double parameter, unsigned int order) const;
  
  /**
   * Define the polynomial curve to fit as best as possible a set of control points at given parametric values.
   *
   * @param points The desired control points.
   * @param param The desired parametric values.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, unsigned int order=0);
    
  /**
   * Define the polynomial curve to fit as best as possible a set of control points at given parametric values.
   *
   * @param points The desired control points.
   * @param param The desired parametric values.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromPoints(const vpMatrix points, const vpColVector &param, unsigned int order=0);
    
  /**
   * Define the polynomial curve to fit as best as possible a set of control points.
   * Parametric values for the different points are automatically computed.
   *
   * @param points The desired control points.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromPointsAuto(const std::vector<vpColVector> &points, unsigned int order=0);
  
  /**
   * Define the polynomial curve to fit as best as possible a set of control points.
   * Parametric values for the different points are automatically computed.
   *
   * @param points The desired control points.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromPointsAuto(const vpMatrix &points, unsigned int order=0);
    
  /**
   * Define the polynomial curve to fit as best as possible a set of weighted control points at given parametric values.
   * Parametric values for the different points are automatically computed according to their position along a given straight direction.
   * 
   * @param points The desired control points.
   * @param direction The direction used to compute the parametric values associated to each point.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromPointsAuto(const std::vector<vpColVector> &points, const vpColVector &direction, unsigned int order=0);
  
  /**
   * Define the polynomial curve to fit as best as possible a set of control points at given parametric values.
   * Parametric values for the different points are automatically computed according to their position along a given straight direction.
   * 
   * @param points The desired control points.
   * @param direction The direction used to compute the parametric values associated to each point.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromPointsAuto(const vpMatrix &points, const vpColVector &direction, unsigned int order=0);
  
  /**
   * Define the polynomial curve to fit as best as possible a set of weighted control points at given parametric values.
   *
   * @param points The desired control points.
   * @param param The desired parametric values.
   * @param weights Set of weights describing the importance of fitting each point with the curve.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromWeightedPoints(const std::vector<vpColVector> &points, const std::vector<double> &param, const std::vector<double> &weights, unsigned int order=0);
  
  /**
   * Define the polynomial curve to fit as best as possible a set of weighted control points at given parametric values.
   *
   * @param points The desired control points.
   * @param param The desired parametric values.
   * @param weights Set of weights describing the importance of fitting each point with the curve.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromWeightedPoints(const vpMatrix &points, const vpColVector &param, const vpColVector &weights, unsigned int order=0);
  
  /**
   * Define the polynomial curve to fit as best as possible a set of control points at given parametric values.
   * Parametric values for the different points are automatically computed.
   * 
   * @param points The desired control points.
   * @param weights Set of weights describing the importance of fitting each point with the curve.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromWeightedPointsAuto(const std::vector<vpColVector> &points, const std::vector<double> &weights, unsigned int order=0);
  
  /**
   * Define the polynomial curve to fit as best as possible a set of control points at given parametric values.
   * Parametric values for the different points are automatically computed.
   * 
   * @param points The desired control points.
   * @param weights Set of weights describing the importance of fitting each point with the curve.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromWeightedPointsAuto(const vpMatrix &points, const vpColVector &weights, unsigned int order=0);
  
  /**
   * Define the polynomial curve to fit as best as possible a set of control points at given parametric values.
   * Parametric values for the different points are automatically computed according to their position along a given straight direction.
   * 
   * @param points The desired control points.
   * @param weights Set of weights describing the importance of fitting each point with the curve.
   * @param direction The direction used to compute the parametric values associated to each point.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromWeightedPointsAuto(const std::vector<vpColVector> &points, const std::vector<double> &weights, const vpColVector &direction, unsigned int order=0);
  
  /**
   * Define the polynomial curve to fit as best as possible a set of control points at given parametric values.
   * Parametric values for the different points are automatically computed according to their position along a given straight direction.
   * 
   * @param points The desired control points.
   * @param weights Set of weights describing the importance of fitting each point with the curve.
   * @param direction The direction used to compute the parametric values associated to each point.
   * @param order The order of the resulting polynomial curve (keep the current order of the curve if given value is <1 (default))
   */
  void defineFromWeightedPointsAuto(const vpMatrix &points, const vpColVector &weights, const vpColVector &direction, unsigned int order=0);
  
  /**
   * Get the curvature of the curve at a specific point.
   * 
   * @param param Parametric value where the curvature should be computed.
   */
  double getCurvature(double param) const;
  
  /**
   * Get the mean deviation of the polynomial with respect to the straight axis between the two extremities.
   * 
   * @param nbCountSeg number of segments used to approximate the deviation.
  */
  double getMeanAxisDeviation(int nbCountSeg=50) const;
    
  /**
   * Set the control points.
   * The input matrix has to be of size 2xn, where n is the order of the polynomial curve.
   *
   * @param controlPoints Reference to the desired control points.
   */
  void setControlPoints(const vpMatrix &controlPoints);

  /**
   * Set the control points.
   */
  void setControlPoints(double **controlPoints);
  
  /**
   * Get the control points. (to remove)
   */
  vpMatrix getControlPoints() const;
  
  /**
   * Get the rendering points. (to remove)
   */
  vpMatrix getRenderingPoints() const;
  
  /**
   * Compute the distance between two curves. (to remove)
   */
  static double curveDistance(const usPolynomialCurve2D &n1, const usPolynomialCurve2D &n2);
  
  /**
   * Get new curve starting at different parametric coefficients .
   */
  usPolynomialCurve2D getSubPolynomialCurve(double startParameter, double endParameter) const;
  
  /**
   * Get new curve with new polynomial order.
   */
  usPolynomialCurve2D getNewOrderPolynomialCurve(unsigned int order) const;
  
  /**
   * Modify the polynomial coefficients such that the parametric variable now goes from two different boundaries, without changing the shape of the curve in space.
   */
  void changeCoefficientsToFitBoundaries(double startParameter, double endParameter);
  
  /**
   * Invert the direction of displacement along the curve with respect to the parametric variable.
   */
  void reverse();
  
  /**
   * Modify the polynomial coefficients such that the metric length of the curve corresponds to the parametric length.
   */
  void changeCoefficientsToFitMetricLength();
  
  /**
   * Apply an homogeneous transformation to the polynomial curve.
   */
  void move(double x, double y, double tz);
  
  /**
   * Scale the polynomial curve.
   */
  void scale(double s);

private:
  unsigned int m_order; // Order of the polynomial curve
  double m_startParameter; // starting value of the parametric variable
  double m_endParameter; // ending value of the parametric variable
  vpMatrix m_polynomialCoefficients; // coefficients of the polynomial curve
};

#endif // __usPolynomialCurve2D_h_
