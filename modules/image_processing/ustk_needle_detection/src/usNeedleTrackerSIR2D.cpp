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

#include <visp3/ustk_needle_detection/usNeedleTrackerSIR2D.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/ustk_core/usImageMathematics.h>

usNeedleTrackerSIR2D::usNeedleTrackerSIR2D()
{
  m_needleModel = NULL;
  m_particles = NULL;
  m_weights = NULL;
  m_noise.setSigmaMean(1.0, 0.0);
  m_noise.seed(42);
  m_sample = vpUniRand(42);
  m_sigma.eye(2);
}

usNeedleTrackerSIR2D::~usNeedleTrackerSIR2D()
{
  if (m_needleModel) {
    delete m_needleModel;
    m_needleModel = NULL;
  }
  if (m_particles) {
    for (unsigned int i = 0; i < m_nParticles; ++i) {
      if (m_particles[i]) {
	delete m_particles[i];
	m_particles[i] = NULL;
      }
      delete [] m_particles;
      m_particles = NULL;
    }
  }
  if (m_weights) {
    delete [] m_weights;
    m_weights = NULL;
  }
}

void usNeedleTrackerSIR2D::init(unsigned int dims[2],
				unsigned int nPoints, unsigned int nParticles,
        const usPolynomialCurve2D &needle)
{
  m_dims[0] = dims[0];
  m_dims[1] = dims[1];
  m_nPoints = nPoints;
  m_nPointsCurrent = needle.getOrder();
  m_nParticles = nParticles;
  m_needleModel = new usPolynomialCurve2D(needle);
  m_particles = new usPolynomialCurve2D*[m_nParticles];
  for (unsigned int i = 0; i < m_nParticles; ++i)
    m_particles[i] = new usPolynomialCurve2D(needle);
  m_weights = new double[m_nParticles];
  for (unsigned int i = 0; i < m_nParticles; ++i)
    m_weights[i] = 1.0 / m_nParticles;
  m_lengthThreshold = 50.0;
}

void usNeedleTrackerSIR2D::init(const vpImage<unsigned char>& I, unsigned int nPoints,
        unsigned int nParticles, const usPolynomialCurve2D &needle) {
  unsigned int dims[2];
  dims[0] = I.getHeight();
  dims[1] = I.getWidth();
  m_fgMean = 0.0;

  vpColVector point;
  unsigned int c = 0;
  double length = needle.getLength();
  for (double t = 0; t <= 1.0; t += 1.0 / length) {
    point = needle.getPoint(t);
    unsigned int x = vpMath::round(point[0]);
    unsigned int y = vpMath::round(point[1]);
    if ((3 <= x) && (x < dims[0] - 3) && (3 <= y) && (y < dims[1] - 3)) {
      m_fgMean += vpImageFilter::gaussianFilter(I, x, y);
    }
    ++c;
  }
  m_fgMean /= c;
  m_bgMean = usImageMathematics::sum(I) / I.getSize();

  init(dims, nPoints, nParticles, needle);
}

usPolynomialCurve2D *usNeedleTrackerSIR2D::getNeedle() { return m_needleModel; }

usPolynomialCurve2D *usNeedleTrackerSIR2D::getParticle(unsigned int i) {
  if (i >= m_nParticles) {
    std::cerr << "Error: in usNeedleTrackerSIR2D::getParticle(): "
	      << "Particle index is out of range." << std::endl;
    exit(EXIT_FAILURE);
  }
  return m_particles[i];
}

double usNeedleTrackerSIR2D::getWeight(unsigned int i) {
  if (i >= m_nParticles) {
    std::cerr << "Error: in usNeedleTrackerSIR2D::getWeight(): "
	      << "Particle index is out of range." << std::endl;
    exit(EXIT_FAILURE);
  }
  return m_weights[i];
}

void usNeedleTrackerSIR2D::setSigma(double s) {
  m_noise.setSigmaMean(s, 0.0);  
}

void usNeedleTrackerSIR2D::setSigma1(double s) {
  m_sigma[0][0] = s;
}

void usNeedleTrackerSIR2D::setSigma2(double s) {
  m_sigma[1][1] = s;
}

void usNeedleTrackerSIR2D::run(vpImage<unsigned char>& I, double v)
{
  vpMatrix controlPoints;
  vpMatrix meanControlPoints;
  vpColVector direction;
  vpMatrix S_noise_tip;
  vpMatrix S_noise_inter;
  vpColVector noise(2);
  vpMatrix U(2,2);

  // Sample particles
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    controlPoints = m_particles[i]->getControlPoints();

    // Get needle direction
    direction = m_particles[i]->getTangent(1.0);
    direction /= direction.euclideanNorm();

    // Compute eigen vectors
    for (unsigned int j = 0; j < 2; ++j)
      U[j][0] = direction[j];
    U[0][1] = - direction[1];
    U[1][1] = direction[0];

    // Compute noise covariance
    S_noise_tip = U * m_sigma * U.t();
    S_noise_inter.eye(2);
    S_noise_inter *= m_sigma[1][1];

    // Compute tip velocity vector
    direction *= v;

    // Entry point
    //controlPoints[0][0] += m_noise();
    //controlPoints[1][0] += m_noise();

    // Intermediate control points
    for (unsigned int j = 1; j < m_nPointsCurrent-1; ++j) {

      // Generate noise
      for (unsigned int k = 0; k < 2; ++k)
	noise[k] = m_noise();
      noise = S_noise_inter * noise;

      // Update control points
      for (unsigned int k = 0; k < 2; ++k)
	controlPoints[k][j] += direction[k] / (m_nPointsCurrent-1) + noise[k] / 4.0;
    }

    // Tip

    // Generate noise
    for (unsigned int k = 0; k < 2; ++k)
      noise[k] = m_noise();
    noise = S_noise_tip * noise;
    
    // Update control points
    for (unsigned int k = 0; k < 2; ++k)
      controlPoints[k][m_nPointsCurrent-1] += direction[k] + noise[k];

    m_particles[i]->setControlPoints(controlPoints.t());
  }

  // Compute weights
  double sumWeights = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    m_weights[i] *= computeLikelihood(m_particles[i], I);
    sumWeights += m_weights[i];
  }
  
  // Normalize the weights and estimate the effective number of particles
  double sumSquare = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    m_weights[i] /= sumWeights;
    sumSquare += vpMath::sqr(m_weights[i]);
  }
  
  double n_eff = 1.0 / sumSquare;
  //std::cout << "Neff = " << n_eff << std::endl;

  // Resampling
  if (n_eff < m_nParticles / 2.0) {
    resample();
  }

  // Compute mean
  meanControlPoints.resize(2, m_nPointsCurrent);
  meanControlPoints = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    meanControlPoints += m_weights[i] * m_particles[i]->getControlPoints();
  }
 
  m_needleModel->setControlPoints(meanControlPoints.t());

  if ((m_needleModel->getLength()) > m_lengthThreshold && (m_nPoints != m_nPointsCurrent)) {
    std::cout << "Changing polynomial order from " << m_nPointsCurrent
        << " to " << (m_nPointsCurrent+1) << std::endl;
    usPolynomialCurve2D *newModel =
      new usPolynomialCurve2D(m_needleModel->changePolynomialOrder(m_nPointsCurrent));
    delete m_needleModel;
    m_needleModel = newModel;
    for (unsigned int i = 0; i < m_nParticles; ++i) {
      usPolynomialCurve2D *newModel =
  new usPolynomialCurve2D(m_particles[i]->changePolynomialOrder(m_nPointsCurrent));
      delete m_particles[i];
    m_particles[i] = newModel;
    }
    m_lengthThreshold *= 2.0;
  }
}
/*
double usNeedleTrackerSIR2D::computeLikelihood(usPolynomialCurve2D *model, vpImage<unsigned char> &I)
{
  double ll = 0.0;
  vpColVector point, dir;
  unsigned int c = 0;
  double length = model->getLength();
  double intensity, l;

  for (double t = 0; t <= 1.0; t += 1.0 / length) {
    point = model->getPoint(t);
    dir = model->getTangent(t);
    int x = vpMath::round(point[0]);
    int y = vpMath::round(point[1]);
    if ((3 <= x) && (x < m_dims[0] - 3) && (3 <= y) && (y < m_dims[1] - 3)) {
      ++c;
      intensity = vpImageFilter::gaussianFilter(I, x, y);
      if (intensity >= m_bgMean)
	return 0.0;
      if (intensity <= m_fgMean)
	continue;
      l = (intensity - m_bgMean) / (m_fgMean - m_bgMean);
      ll += log(l);
    }
  }

  if (c == 0)
    return 0.0;

  ll /= c;

  point = model->getPoint(1.0 + 1.0 / length);
  int x = vpMath::round(point[0]);
  int y = vpMath::round(point[1]);
  if ((0 <= x) && (x < m_dims[0]) && (0 <= y) && (y < m_dims[1])) {
    intensity = vpImageFilter::gaussianFilter(I,x,y);
    if (intensity >= m_bgMean)
      return exp(ll);
    if (intensity <= m_fgMean)
      return 0.0;
    l = 1.0 - (intensity - m_bgMean) / (m_fgMean - m_bgMean);
    ll += log(l);
  }

  return exp(ll);
}
*/

double usNeedleTrackerSIR2D::computeLikelihood(usPolynomialCurve2D *model, vpImage<unsigned char> &I)
{
	double l = 0.0;
	vpColVector point;
	unsigned int c = 0;
  double length = model->getLength();
	double intensity;

	for (double t = 0; t <= 1.0; t += 1.0 / length) {
    point = model->getPoint(t);
    unsigned int x = vpMath::round(point[0]);
    unsigned int y = vpMath::round(point[1]);
		if ((3 <= x) && (x < m_dims[0] - 3) && (3 <= y) && (y < m_dims[1] - 3)) {
			++c;
			intensity = vpImageFilter::gaussianFilter(I, x, y);
			l += intensity;
		}
	}

	if (c == 0)
		return 0.0;

	l /= c;

  point = model->getPoint(1.0);
  unsigned int x = vpMath::round(point[0]);
  unsigned int y = vpMath::round(point[1]);
	if ((3 <= x) && (x < m_dims[0] - 3) && (3 <= y) && (y < m_dims[1] - 3)) {
		intensity = vpImageFilter::gaussianFilter(I, x, y);
		l += intensity;
	}

  point = model->getPoint(1.0 + 1.0 / length);
	x = vpMath::round(point[0]);
	y = vpMath::round(point[1]);
	if ((3 <= x) && (x < m_dims[0] - 3) && (3 <= y) && (y < m_dims[1] - 3)) {
		intensity = vpImageFilter::gaussianFilter(I, x, y);
		l -= intensity;
	}

	return l;
}

void usNeedleTrackerSIR2D::resample() {
  // std::cout << "Resampling..." << std::endl;
  int *idx = new int[m_nParticles];
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    double x = m_sample();
    double sumWeights = 0.0;
    for (unsigned int j = 0; j < m_nParticles; ++j) {
      if (x < sumWeights + m_weights[j]) {
        idx[i] = j;
        break;
      }
      sumWeights += m_weights[j];
    }
  }
  usPolynomialCurve2D **oldParticles = new usPolynomialCurve2D*[m_nParticles];
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    oldParticles[i] = new usPolynomialCurve2D(*m_particles[i]);
  }
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    delete m_particles[i];
    m_particles[i] = new usPolynomialCurve2D(*oldParticles[idx[i]]);
  }
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    delete oldParticles[i];
    oldParticles[i] = NULL;
  }
  delete [] oldParticles;
  delete [] idx;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    m_weights[i] = 1.0 / m_nParticles;
  }
}
