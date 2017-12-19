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

#ifndef US_SIGNAL_PROCESSING_H
#define US_SIGNAL_PROCESSING_H

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpSubMatrix.h>
#include <visp3/ustk_core/usImageRF2D.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <omp.h>
#include <typeinfo>
#include <vector>

/// DEFINITIONS
#define PI M_PI
typedef unsigned int uint;
using namespace std;
typedef complex<double> dcomplex;

class VISP_EXPORT usSignalProcessing
{
public:
  /// Constructor
  usSignalProcessing();
  /// Virtual destructor
  virtual ~usSignalProcessing();
  /// FIR1: Lowpass filter
  static vpMatrix m_FIR1(unsigned N, double Wn);
  /// Gaussian Kernel generator
  static vpMatrix GaussianFilter(int ny, int nx, double sigma);
  /// Computation of gradients
  static vpMatrix GetGx(vpMatrix F);
  static vpMatrix GetGy(vpMatrix F);
  static vpMatrix GetGx(usImageRF2D<short int> F);
  static vpMatrix GetGy(usImageRF2D<short int> F);
  // Diference between usImageRF2D
  static vpMatrix Difference(usImageRF2D<short int> A, usImageRF2D<short int> B);
  /// Bilinear Interpolation
  static vpMatrix BilinearInterpolation(vpMatrix In, uint newW, uint newH);
  /// Element-wise product
  static vpMatrix HadamardProd(vpMatrix in_array1, vpMatrix in_array2);
  /// Normalize Matrix to 1
  static vpMatrix Normalize_1(vpMatrix &in_array);
  static vpMatrix SubMatrix(const vpMatrix &M, uint r, uint c, uint nrows, uint ncols);
};

#endif // US_SIGNAL_PROCESSING_H
