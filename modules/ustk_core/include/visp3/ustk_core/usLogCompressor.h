/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
  double m_alpha; /// Contrast parameter
  unsigned char *m_compressionTable; /// Compression table
  
};

#endif // have fftw
#endif // __usLogCompressor_h_
