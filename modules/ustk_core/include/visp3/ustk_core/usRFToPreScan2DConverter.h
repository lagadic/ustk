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
 * Pedro Patlan
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usRFToPreScan2DConverter.h
 * @brief 2D scan-converter
 */

#ifndef __usRFToPreScan2DConverter_h_
#define __usRFToPreScan2DConverter_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

//external includes
#include <fftw3.h>

// std includes
#include <vector>
#include <cmath>
#include <complex>

// visp/ustk includes
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usLogCompressor.h>

/**
 * @class usRFToPreScan2DConverter
 * @brief 2D conversion from RF signal to pre-scan image
 * @ingroup module_ustk_core
 *
 * This class allows to convert 2D RF ultrasound images to pre-scan.
 *
 */
class VISP_EXPORT usRFToPreScan2DConverter
{
 public:

  usRFToPreScan2DConverter();

  ~usRFToPreScan2DConverter();

  void convert(const usImageRF2D<short int> &rfImage, usImagePreScan2D<unsigned char> &preScanImage);

private:
  std::vector<std::complex<double> > HilbertTransform(const short *s, int size);
  void sqrtAbsv(std::vector<std::complex<double> > cv, double* out);

  usLogCompressor m_logCompressor;
};

#endif // USTK_HAVE_FFTW
#endif // __usRFToPreScan2DConverter_h_
