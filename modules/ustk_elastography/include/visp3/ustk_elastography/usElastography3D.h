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
 * Marc Pouliquen
 *
 *****************************************************************************/

#ifndef __usElastography3D_h_
#define __usElastography3D_h_

/**
 * @file usElastography3D.h
 * @brief Base class to compute an elastography on 3D RF images.
 */
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <visp3/ustk_elastography/usElastography.h>

#include <visp3/ustk_core/usImage3D.h>
#include <visp3/ustk_core/usImageRF3D.h>

/**
 * @class usElastography3D
 * @brief Computation of a strain map using two sucessive RF volulmes acquired at different compressions of the probe.
 * @ingroup module_ustk_elastography
 *
 * This class produces a strain map based on tissues displacements between 2 input 3D RF images.
 *
 * The typical use of this class is :
 *  - Set the ROI in the RF image, with setROI()
 *  - Set the pre and post compressed volumes, with setPreCompression() and setPostCompression() or with updateRF().
 *  - Run the process, using run()
 */
class VISP_EXPORT usElastography3D
{
public:
  usElastography3D();
  usElastography3D(usImageRF3D<short int> &Pre, usImageRF3D<short int> &Post);
  virtual ~usElastography3D();

  double getFPS(void);
  double getfs(void);

  usImage3D<unsigned char> run();

  void setDecimationFactor(unsigned int decimationFactor);
  void setFPS(double fps);
  void setfs(double fs);
  void setLSQpercentage(double per);
  void setMotionEstimator(usElastography::MotionEstimator t_mest);
  void setPostCompression(const usImageRF3D<short> &Post);
  void setPreCompression(const usImageRF3D<short> &Pre);
  void setROI(int tx, int ty, int tz, int tw, int th, int tf);

  void updateRF(const usImageRF3D<short int> &image);
  void updateROIPos(int tx, int ty, int tz);

private:
  usElastography m_elastography2DProcessor;

  usImage3D<unsigned char> m_StrainMap;
  usImageRF3D<short int> m_Precomp;
  usImageRF3D<short int> m_Postcomp;

  bool m_isloadPre;
  bool m_isloadPost;

  bool m_setROI;
  int m_framesInROI;
  int m_frameBeginROI;
};

#endif // FFTW
#endif // __usElastography3D_h_
