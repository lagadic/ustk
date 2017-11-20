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

/**
 * @file usLogCompressor.h
 * @brief Log-compression filter.
 */

#ifndef __usLogCompressor_h_
#define __usLogCompressor_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <fftw3.h>

/**
 * @class usLogCompressor
 * @brief Log-compression filter.
 * @author Pierre Chatelain
 *
 * This class performs log-compression of an array of data.
 * The filter should be initialized through init() and then applied through run().
 */
class usLogCompressor
{
public:
  /// Constructor
  usLogCompressor();

  /// Destructor
  ~usLogCompressor();

  void init(double alpha);

  void run(unsigned char *dst, const double *src, unsigned int size);

private:
  double m_alpha;                    /// Contrast parameter
  unsigned char *m_compressionTable; /// Compression table
};

#endif // have fftw
#endif // __usLogCompressor_h_
