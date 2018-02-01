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

#ifndef US_MOTION_ESTIMATION_H
#define US_MOTION_ESTIMATION_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <visp3/core/vpMatrix.h>
#include <visp3/ustk_core/usImageRF2D.h>

// Armadillo
#include <armadillo>

using namespace arma;

class VISP_EXPORT usMotionEstimation
{
public:
  usMotionEstimation();
  virtual ~usMotionEstimation();
  void init(mat M1, mat M2, int blk_w, int blk_h, int sr_w, int sr_h);
  void init(usImageRF2D<short int> usM1, usImageRF2D<short int> usM2, int blk_w, int blk_h, int sr_w, int sr_h);
  void run();
  vec FullSearch(mat block, int xc, int yc, int sr_w, int sr_h);
  vec TaylorApp(mat B1, mat B2);
  mat xDifferential(mat input);
  mat yDifferential(mat input);
  mat MedianFilt2D(mat M, int kw, int kh);
  mat getU(void) { return m_U; }
  mat getV(void) { return m_V; }
  vpMatrix getU_vp(void) { return convert_mat2vpMatrix(m_U); }
  vpMatrix getV_vp(void) { return convert_mat2vpMatrix(m_V); }
  void saveU(const char *t_s);
  void saveV(const char *t_s);
  mat convert_usImageRF2mat(usImageRF2D<short int> vI);
  vpMatrix convert_mat2vpMatrix(mat vI);

private:
  mat m_M1;
  mat m_M2;
  mat m_U;
  mat m_V;
  int m_Wblock;
  int m_Hblock;
  int m_WSregion;
  int m_HSregion;
  ivec m_blk_rngX;
  ivec m_blk_rngY;
  ivec m_xcRng;
  ivec m_ycRng;
  int m_Lx;
  int m_Ly;
};

#endif // US_MOTION_ESTIMATION_H
