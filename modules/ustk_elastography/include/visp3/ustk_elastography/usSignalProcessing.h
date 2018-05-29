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

#ifndef __usSignalProcessing_h_
#define __usSignalProcessing_h_

/**
 * @file usSignalProcessing.h
 * @brief Class containing a set of static methods to compute various processes on RF images (gradients, product,
 * interpolation).
 */

#include <visp3/core/vpMatrix.h>
#include <visp3/ustk_core/usImageRF2D.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <typeinfo>
#include <vector>

/// DEFINITIONS
#define PI M_PI
typedef unsigned int uint;
using namespace std;
typedef complex<double> dcomplex;

/**
 * @class usSignalProcessing.h
 * @brief Class containing a set of static methods to compute various processes on RF images (gradients, product,
 * interpolation).
 * @ingroup module_ustk_elastography
 */
class VISP_EXPORT usSignalProcessing
{
public:
  /// Constructor
  usSignalProcessing();
  /// Virtual destructor
  virtual ~usSignalProcessing();
  /// Gaussian Kernel generator
  static vpMatrix GaussianFilter(int height, int width, double sigma);
  /// Computation of gradients
  static vpMatrix getXGradient(const usImageRF2D<short> &image);
  static vpMatrix getYGradient(const usImageRF2D<short int> &image);
  // Diference between usImageRF2D
  static vpMatrix Difference(const usImageRF2D<short int> &A, const usImageRF2D<short int> &B);
  /// Bilinear Interpolation
  static vpMatrix BilinearInterpolation(vpMatrix In, uint newW, uint newH);
  /// Element-wise product
  static vpMatrix HadamardProd(vpMatrix matrix1, vpMatrix matrix2);
};

#endif // __usSignalProcessing_h_
