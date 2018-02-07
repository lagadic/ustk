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
 * Pedro Patlan Rosales
 * Marc Pouliquen
 *
 *****************************************************************************/

#ifndef USELASTOGRAPHY_H
#define USELASTOGRAPHY_H

/**
 * @file usElastography.h
 * @brief Base class to compute an elastography on RF images.
 */
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <visp3/core/vpImageTools.h>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_elastography/usConvolution2d.h>
#include <visp3/ustk_elastography/usMotionEstimation.h>
#include <visp3/ustk_elastography/usSignalProcessing.h>

/**
 * @class usElastography
 * @brief Computation of a strain map using two sucessive RF images acquired at different compressions of the probe.
 * @ingroup module_ustk_elastography
 *
 * This class produces a strain map based on tissues displacements between 2 input RF images.
 *
 * The typical use of this class is :
 *  - Set the ROI in the RF image, with setROI()
 *  - Set the pre and post compressed images, with setPreCompression() and setPostCompression() or with updateRF().
 *  - Run the process, using run()
 */
class VISP_EXPORT usElastography
{
public:
  enum MotionEstimator { OF, BMA_TAYLOR };
  usElastography();
  usElastography(usImageRF2D<short int> &Pre, usImageRF2D<short int> &Post);
  virtual ~usElastography();

  double getFPS(void) { return m_FPS; }
  double getfs(void) { return m_fs; }

  vpImage<unsigned char> run();

  void setFPS(double fps);
  void setfs(double fs);
  void setLSQpercentage(double per);
  void setMotionEstimator(MotionEstimator t_mest) { m_mEstimatior = t_mest; }
  void setPostCompression(const usImageRF2D<short> &Post);
  void setPreCompression(const usImageRF2D<short> &Pre);
  void setROI(int tx, int ty, int tw, int th);

  void updateRF(const usImageRF2D<short int> &image);
  void updateROIPos(int tx, int ty);

private:
  usImageRF2D<short int> usImageRF_ROI(const usImageRF2D<short int> &M, uint r, uint c, uint nrows, uint ncols);

  vpImage<unsigned char> m_StrainMap;
  usImageRF2D<short int> m_Precomp;
  usImageRF2D<short int> m_Postcomp;

  double m_Lsqper;
  bool m_isloadPre;
  bool m_isloadPost;
  double m_FPS;
  double m_fs;
  double m_c;
  double m_min_str;
  double m_max_str;
  double m_max_abs;
  int m_h_m;
  int m_w_m;

  int m_ix;
  int m_iy;
  int m_rw;
  int m_rh;
  bool m_setROI;
  usImageRF2D<short int> m_PreROI;
  usImageRF2D<short int> m_PostROI;

  std::vector<usConvolution2d *> cC;

  // Motion estimation
  MotionEstimator m_mEstimatior;
  usMotionEstimation m_ME;

  vpMatrix U;
  vpMatrix V;
};

#endif // FFTW
#endif // USELASTOGRAPHY_H
