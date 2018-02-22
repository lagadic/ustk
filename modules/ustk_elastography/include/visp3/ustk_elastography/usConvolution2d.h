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

/**
 * @file usConvolution2d.h
 * @brief Convolution process for elastography puropse, based on fftw thirdparty library.
 */

#ifndef US_CONVOLUTION_2D_H
#define US_CONVOLUTION_2D_H

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <fftw3.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>

/**
 * @class usConvolution2d
 * @brief Convolution process for elastography puropse, based on fftw thirdparty library.
 * @ingroup module_ustk_elastography
 *
 * This class performs 2D convolutions on RF images.
 */
class VISP_EXPORT usConvolution2d
{
public:
  usConvolution2d();
  virtual ~usConvolution2d();
  void init(const vpMatrix &matrix1, const vpMatrix &matrix2);
  vpMatrix run(const vpMatrix &matrix1, const vpMatrix &matrix2);

private:
  fftw_complex *outa;
  fftw_complex *outb;
  fftw_complex *outc;
  fftw_complex *out;
  fftw_complex *ad;
  fftw_complex *bd;
  fftw_plan p1;
  fftw_plan p2;
  fftw_plan p3;

  bool m_init;

  vpMatrix m_M1;
  vpMatrix m_M2;
  vpMatrix m_R;
  unsigned int h_dst;
  unsigned int w_dst;
  unsigned int hf;
  unsigned int wf;
  unsigned int Am; // A row number
  unsigned int An; // A col number
  unsigned int Bm; // B row number
  unsigned int Bn; // B col number

  void padding_zeros(void);
};

#endif // USTK_HAVE_FFTW
#endif // US_CONVOLUTION_2D_H
