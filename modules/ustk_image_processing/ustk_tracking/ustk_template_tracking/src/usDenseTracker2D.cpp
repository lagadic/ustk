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
 *
 *****************************************************************************/

#include <visp3/ustk_template_tracking/usDenseTracker2D.h>

#include <visp3/ustk_core/usRectangle.h>
#include <visp3/ustk_core/usImageMathematics.h>

#include <visp3/core/vpImageFilter.h>

/**
 * @brief Initialisation of the tracker : to call to set the region to track (R) in the image (I) before starting the tracking.
 * @param I Image containing a region to track.
 * @param R Region of interest (in the image pxiel coordinates).
 */
void usDenseTracker2D::init(const vpImage<unsigned char> &I, const usRectangle &R)
{
  usImageMathematics::extract(I, m_template, R);
  usImageMathematics::extract(I, m_region, R);
  m_target = R;
  m_height = m_template.getHeight();
  m_width = m_template.getWidth();
  m_size = m_template.getSize();
  s_desired.resize(m_size);
  s_current.resize(m_size);
  m_LI.resize(m_size, 3);
  
  //compute gradients
  vpImageFilter::getGradY(m_template, m_gradX);
  vpImageFilter::getGradX(m_template, m_gradY);

  //center of the target
  double u0 = m_target.getHeight() / 2.0;
  double v0 = m_target.getWidth() / 2.0;

  //filling desired col vector with pixels values of template region
  //filling interaction matrix
  for (unsigned int u = 0; u < m_height; ++u)
    for (unsigned int v = 0; v < m_width; ++v)
    {
      s_desired[u * m_width + v] = m_template[u][v];
      s_current[u * m_width + v] = m_region[u][v];
      m_LI[u * m_width + v][0] = m_gradX[u][v];
      m_LI[u * m_width + v][1] = m_gradY[u][v];
      m_LI[u * m_width + v][2] = (static_cast<double>(u) - u0) * m_gradY[u][v]
          - (static_cast<double>(v) - v0) * m_gradX[u][v];
    }
  //pseudo inverse of interaction matrix
  m_LI_inverse = m_LI.pseudoInverse();
}

/**
 * @brief Tracking method, to call at every new frame to track.
 * @param I The new image.
 */
void usDenseTracker2D::update(const vpImage<unsigned char> &I)
{
  double gain = 0.6;

  unsigned int max_iter = 40;
  unsigned int i = 0;
  double rms = 1.0;
  double rms0 = 0.0;
  double drms = 1.0e-5;

  while ((i < max_iter) && (std::abs(rms - rms0) > drms))
  {
    //extract new region from previous target rectangle
    usImageMathematics::extract(I, m_region, m_target);

    //filling current features colVector 
    for (unsigned int u = 0; u < m_height; ++u)
      for (unsigned int v = 0; v < m_width; ++v)
        s_current[u * m_width + v] = m_region[u][v];

    //compute error and velocity command
    vpColVector e = s_current - s_desired;
    vpColVector v = - gain * m_LI_inverse * e;

    rms0 = rms;
    rms = e.euclideanNorm() / m_size;

    //extract deplacements to apply to follow the region
    double alpha = m_target.getOrientation();
    double dx = v[0] * cos(alpha) + v[1] * sin(alpha);
    double dy = v[1] * cos(alpha) - v[0] * sin(alpha);
    double da = - v[2];

    //update target with old values and deplacements previously comuted
    m_target.setCenter(m_target.getCx() + gain * dx, m_target.getCy() + gain * dy);
    m_target.setOrientation(alpha + gain * da);

    ++i;
  }

  //std::cout << "Converged in " << i << " iterations with rms = " << rms
  //	    << "and drms = " << std::abs(rms - rms0) << std::endl;
}

/**
 * @brief To call after update() at each new frame, to get the position of the ROI in the last acquired frame.
 * @return The rectangle pixel coordinates in the new frame.
 */
usRectangle usDenseTracker2D::getTarget() const
{
  return m_target;
}

vpImage<unsigned char> &usDenseTracker2D::getTemplate()
{
  return m_template;
}

vpImage<unsigned char> &usDenseTracker2D::getRegion()
{
  return m_region;
}

