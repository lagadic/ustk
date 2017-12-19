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

#ifndef US_CONVOLUTION_2D_H
#define US_CONVOLUTION_2D_H

#include <fftw3.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>

class VISP_EXPORT usConvolution2d
{
public:
  usConvolution2d();
  virtual ~usConvolution2d();
  void init(vpMatrix t_M1, vpMatrix t_M2);
  void update(vpMatrix t_M1, vpMatrix t_M2);
  void run();
  vpMatrix getConvolution(void);

private:
  fftw_plan p1;
  fftw_plan p2;
  fftw_plan p3;
  vpMatrix m_M1;
  vpMatrix m_M2;
  vpMatrix m_R;
  uint h_dst;
  uint w_dst;
  uint hf;
  uint wf;
  fftw_complex *outa;
  fftw_complex *outb;
  fftw_complex *outc;
  fftw_complex *out;
  fftw_complex *ad;
  fftw_complex *bd;
  unsigned int Am; // A row number
  unsigned int An; // A col number
  unsigned int Bm; // B row number
  unsigned int Bn; // B col number
  bool m_computed;
  void padding_zeros(void);
};

#endif // US_CONVOLUTION_2D_H
