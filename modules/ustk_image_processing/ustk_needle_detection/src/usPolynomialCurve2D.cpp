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

#include <visp3/ustk_needle_detection/usPolynomialCurve2D.h>
#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>

usPolynomialCurve2D::usPolynomialCurve2D() {
  m_order = 0;
  m_nRenderingLines = 0;
  m_controlPoints = vpMatrix();
  m_model = vpMatrix();
  m_renderingPoints = vpMatrix();
}

usPolynomialCurve2D::usPolynomialCurve2D(const usPolynomialCurve2D &needle) {
  m_order = needle.getOrder();
  m_nRenderingLines = needle.getNumberOfRenderingLines();
  m_controlPoints = needle.getControlPoints();
  m_model = needle.m_model;
  m_renderingPoints = needle.getRenderingPoints();
}

usPolynomialCurve2D::usPolynomialCurve2D(unsigned int order) {
  m_order = order;
  if (m_order < 2) {
    std::cerr << "Error: the order of the polynomial curve should be at least 2." << std::endl;
    m_order = 2;
    m_nRenderingLines = 1;
  }
  else if (m_order == 2)
    m_nRenderingLines = 1;
  else
    m_nRenderingLines = 10;
  m_controlPoints = vpMatrix(2, m_order);
  m_model = vpMatrix(2, order);
  m_renderingPoints = vpMatrix(2, m_nRenderingLines+1);
}

void usPolynomialCurve2D::setOrder(unsigned int order) {
  m_order = order;
  if (m_order < 2)
    std::cerr << "Error: the order of the polynomial curve should be at least 2." << std::endl;
  else if (m_order == 2)
    m_nRenderingLines = 1;
  else
    m_nRenderingLines = 10;
  m_controlPoints = vpMatrix(2, m_order);
  m_renderingPoints = vpMatrix(2, m_nRenderingLines+1);
  m_model = vpMatrix(2, order);
}

unsigned int usPolynomialCurve2D::getOrder() const { return m_order; }

void usPolynomialCurve2D::setNumberOfRenderingLines(unsigned int nRenderingLines) {
  m_nRenderingLines = nRenderingLines;
}

unsigned int usPolynomialCurve2D::getNumberOfRenderingLines() const { return m_nRenderingLines; }

void usPolynomialCurve2D::setControlPoints(const vpMatrix &controlPoints) {
  if (controlPoints.getCols() != 2) {
    std::cerr << "Error: in usPolynomialCurve2D::setControlPoints(): "
	      << "Input matrix should of size nx2." << std::endl;
    exit(EXIT_FAILURE);
  }
  if (controlPoints.getRows() != m_order) {
    setOrder(controlPoints.getRows());
  }

  m_controlPoints = controlPoints.t();
  
  if ((m_controlPoints.getCol(m_order-1) - m_controlPoints.getCol(0)).euclideanNorm() == 0.0) {
    std::cerr << "Warning: in usPolynomialCurve2D::setControlPoints(): needle model is degenerate."
	      << std::endl;
    for (unsigned int i=0; i<2; ++i) {
      m_model[i][0] = m_controlPoints[i][0];
      m_model[i][1] = 0.0;
      m_model[i][2] = 0.0;
    }
  } else {
    if (m_order!=6) {
      vpMatrix tapprox = usNeedleDetectionTools::approximateCoordinates(m_controlPoints.t(),
									m_controlPoints.t(),
									m_order);
      m_model = m_controlPoints * tapprox.inverseByLU();
    }
    else {
      double norm = (m_controlPoints.getCol(2) - m_controlPoints.getCol(0)).euclideanNorm();
      if (norm == 0.0) {
  std::cerr << "Warning: in usPolynomialCurve2D::setControlPoints(): needle model is degenerate."
		  << std::endl;
	norm = 1;
	for (unsigned int i=0; i<2; ++i) {
	  m_model[i][0] = m_controlPoints[i][0];
	  m_model[i][1] = 0.0;
	  m_model[i][2] = 0.0;
	}
      } else {
	double t = (m_controlPoints.getCol(1) - m_controlPoints.getCol(0)).euclideanNorm() / norm;
	if (t == 0.0) {
    std::cerr << "Warning: in usPolynomialCurve2D::setControlPoints(): needle model is degenerate."
		    << std::endl;
	  for (unsigned int i=0; i<3; ++i) {
	    m_model[i][0] = m_controlPoints[i][0];
	    m_model[i][1] = 0.0;
	    m_model[i][2] = 0.0;
	  }
	} else {
	  for (unsigned int i=0; i<2; ++i) {
	    m_model[i][0] = m_controlPoints[i][0];
	    m_model[i][2] = (t * (m_controlPoints[i][2]-m_model[i][0]) - m_controlPoints[i][1])
	      / (t * (1.0 - t) + 1e-10);
	    m_model[i][1] = m_controlPoints[i][2] - m_model[i][0] - m_model[i][2];
	  }
	}
      }
    }
  }
  vpMatrix coords = vpMatrix(m_order, m_nRenderingLines+1);
  for (unsigned int j=0; j<m_nRenderingLines+1; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / static_cast<double>(m_nRenderingLines);
    for (unsigned int i=1; i<m_order; ++i) 
      coords[i][j]  = coords[i-1][j] * t;
  }
  m_renderingPoints = m_model * coords;
}

void usPolynomialCurve2D::setControlPoints(double **controlPoints) {
  for (unsigned int i=0; i<2; ++i)
    for (unsigned int j=0; j<m_order; ++j)
      m_controlPoints[i][j] = controlPoints[j][i];

  vpMatrix tapprox = usNeedleDetectionTools::approximateCoordinates(m_controlPoints.t(),
								    m_controlPoints.t(),
								    m_order);
  m_model = m_controlPoints * tapprox.inverseByLU();
  vpMatrix coords = vpMatrix(m_order, m_nRenderingLines+1);
  
  for (unsigned int j=0; j<m_nRenderingLines+1; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / static_cast<double>(m_nRenderingLines);
    for (unsigned int i=1; i<m_order; ++i) 
      coords[i][j]  = coords[i-1][j] * t;
  }
  m_renderingPoints = m_model * coords;
}

vpMatrix usPolynomialCurve2D::getControlPoints() const { return m_controlPoints; }

vpMatrix usPolynomialCurve2D::getRenderingPoints() const
{
  vpMatrix points = m_renderingPoints;
  return points;
}

void usPolynomialCurve2D::setModel(const vpMatrix &model) {
  m_model = model;
  vpMatrix coords(m_order,m_order);
  for (unsigned int j=0; j<m_order; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / static_cast<double>(m_order-1);
    for (unsigned int i=1; i<m_order; ++i) 
      coords[i][j]  = coords[i-1][j] * t;
  }
  m_controlPoints = m_model * coords;
  coords = vpMatrix(m_order, m_nRenderingLines+1);
  for (unsigned int j=0; j<m_nRenderingLines+1; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / static_cast<double>(m_nRenderingLines);
    for (unsigned int i=1; i<m_order; ++i) 
      coords[i][j]  = coords[i-1][j] * t;
  }
  m_renderingPoints = m_model * coords;
}

const vpMatrix *usPolynomialCurve2D::getModel() const { return &m_model; }

vpColVector usPolynomialCurve2D::getPoint(double t) const {
  vpColVector T(m_order);
  T[0] = 1.0;
  for (unsigned int i=1; i<m_order; ++i)
    T[i] = T[i-1] * t;
  return m_model * T;
}

vpColVector usPolynomialCurve2D::getTangent(double t) const {
  vpColVector T(m_order);
  double tt = 1.0;
  T[0] = 0.0;
  for (unsigned int i=1; i<m_order; ++i) {
    T[i] = i * tt;
    tt *= t;
  }
  return m_model * T;
}

double usPolynomialCurve2D::getLength() const {
  vpMatrix coords(m_order, 50);
  for (unsigned int j=0; j<50; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / 49.0;
    for (unsigned int i=1; i<m_order; ++i) 
      coords[i][j]  = coords[i-1][j] * t;
  }
  vpMatrix points = m_model * coords;
  double length = 0.0;
  for (unsigned int i=0; i<49; ++i)
    length += (points.getCol(i) - points.getCol(i+1)).euclideanNorm();
  return length;
};

double usPolynomialCurve2D::curveDistance(const usPolynomialCurve2D &n1, const usPolynomialCurve2D &n2) {
  unsigned int order1 = n1.getOrder();
  unsigned int order2 = n2.getOrder();
  vpMatrix coords1(order1, 50);
  vpMatrix coords2(order2, 50);
  for (unsigned int j=0; j<50; ++j) {
    coords1[0][j] = 1.0;
    coords2[0][j] = 1.0;
    double t = static_cast<double>(j) / 49.0;
    for (unsigned int i=1; i<order1; ++i) 
      coords1[i][j]  = coords1[i-1][j] * t;
    for (unsigned int i=1; i<order2; ++i) 
      coords2[i][j]  = coords2[i-1][j] * t;
  }
  vpMatrix p1 = (*n1.getModel()) * coords1;
  vpMatrix p2 = (*n2.getModel()) * coords2;
  double distance = 0.0;
  for (unsigned int i=0; i<50; ++i)
    distance += (p1.getCol(i) - p2.getCol(i)).euclideanNorm();
  distance /= 50;
  return distance;
}

double usPolynomialCurve2D::getCurvature() {
  vpMatrix coords(m_order, 50);
  for (unsigned int j=0; j<50; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / 49.0;
    for (unsigned int i=1; i<m_order; ++i) 
      coords[i][j]  = coords[i-1][j] * t;
  }
  vpMatrix p1 = m_model * coords;
  vpColVector p1p2 = m_controlPoints.getCol(m_order-1) - m_controlPoints.getCol(0);
  vpColVector p2;
  double curvature = 0.0;
  for (unsigned int i=0; i<50; ++i) {
    p2 = m_controlPoints.getCol(0) + static_cast<double>(i) / 49.0 * p1p2;
    curvature += (p1.getCol(i) - p2).euclideanNorm();
  }
  curvature /= 50;
  return curvature;
}

usPolynomialCurve2D usPolynomialCurve2D::changePolynomialOrder(unsigned int newOrder) {
  usPolynomialCurve2D newNeedleModel(newOrder);
  vpMatrix newControlPoints;
  vpMatrix coords(m_order,newOrder);
  for (unsigned int j=0; j<newOrder; ++j) {
    coords[0][j] = 1.0;
    double t = static_cast<double>(j) / static_cast<double>(newOrder-1);
    for (unsigned int i=1; i<m_order; ++i) 
      coords[i][j]  = coords[i-1][j] * t;
  }
  //m_model.print(std::cerr, 5, "MODEL");
  //coords.print(std::cerr, 5, "T");
  newControlPoints = m_model * coords;
  //newControlPoints.print(std::cerr, 5, "CP");
  newNeedleModel.setControlPoints(newControlPoints.t());
  return newNeedleModel;
}
