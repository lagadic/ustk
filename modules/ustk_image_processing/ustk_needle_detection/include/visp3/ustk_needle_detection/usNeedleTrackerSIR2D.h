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

#ifndef __usNeedleTrackerSIR2D_h_
#define __usNeedleTrackerSIR2D_h_

// ViSP
#include <visp3/core/vpNoise.h>
#include <visp3/core/vpImage.h>

// UsNeedleDetection
#include <visp3/ustk_needle_detection/usPolynomialCurve2D.h>

/**
 * @brief Needle detection in 2D images based on particle filtering.
 *
 * This class defines the particle-based needle detection pipeline for 2D images.
 */
class VISP_EXPORT usNeedleTrackerSIR2D
{
 public:
  /**
   * Default constructor.
   */
  usNeedleTrackerSIR2D();
  
  /**
   * Destructor.
   */
  ~usNeedleTrackerSIR2D();
 
  /**
   * Computing the likelihood of the model for a given observation.
   *
   * @param model The needle model.
   * @param I The input image.
   */
  double computeLikelihood(usPolynomialCurve2D *model, vpImage<unsigned char> &I);

  /**
   * Returns the detected needle model.
   */
  usPolynomialCurve2D *getNeedle();

  /**
   * Get a specific particle.
   */
  usPolynomialCurve2D *getParticle(unsigned int i);

  /**
   * Get a specific particle's weight.
   */
  double getWeight(unsigned int i);

  /**
   * Initialization method.
   *
   * @param dims The image dimensions (height, width).
   * @param nPoints The number of control points.
   * @param nParticles The number of particles.
   * @param needle The initial needle model.
   *
   * Initializes the particle filter.
   * Should be called before to start the detection.
   */
  void init(unsigned int dims[2], unsigned int nPoints, unsigned int nParticles,
      const usPolynomialCurve2D &needle);

  /**
   * Initialization method.
   *
   * Initializes all attributes with default values and the given image resolution.
   * Should be called before to start the detection.
   * @param I Reference to the image.
   * @param nPoints The number of control points.
   * @param nParticles The number of particles.
   * @param needle The initial needle model.
   */
  void init(const vpImage<unsigned char>& I, unsigned int nPoints, unsigned int nParticles,
      const usPolynomialCurve2D &needle);

  /**
   * Resample the particles proportionnaly to their weights.
   */
  void resample();

  /**
   * Runs the needle detection process with the current post-scan volume.
   *
   * @param I The input image.
   * @param v The insertion velocity.
   */
  void run(vpImage<unsigned char>& I, double v);

/**
   * Set the standard deviation for the update noise.
   */
  void setSigma(double s);

  /**
   * Set the standard deviation along the insertion direction.
   */
  void setSigma1(double s);

  /**
   * Set the standard deviation orthogonal to the insertion direction.
   */
  void setSigma2(double s);

 private:
  unsigned int m_nParticles;
  unsigned int m_nPoints;
  unsigned int m_nPointsCurrent;
  usPolynomialCurve2D *m_needleModel;
  usPolynomialCurve2D **m_particles;
  double *m_weights;
  vpGaussRand m_noise;
  vpUniRand m_sample;
  vpMatrix m_sigma;
  double m_lengthThreshold;

  // Image parameters
  unsigned int m_dims[2];
  double m_fgMean;
  double m_bgMean;
};

#endif // US_NEEDLE_TRACKER_SIR_2D
