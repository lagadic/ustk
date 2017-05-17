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
 * @author Pierre Chatelain
 */

#ifndef US_LOG_COMPRESSOR_H
#define US_LOG_COMPRESSOR_H

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

  /**
   * Initialize the log-compression filter.
   * @param alpha The contrast parameter.
   */
  void init(double alpha);

  /**
   * Run the log-compression filter.
   * @param[out] dst Pointer to the destination data array.
   * @param[in] src Pointer to the source data array.
   * @param[in] size The size of the data array.
   *
   * Both \e dst and \e src must point to an array of size \e size.
   */
  void run(unsigned char *dst, const double *src, unsigned int size);

 private:
  double m_alpha; /// Contrast parameter
  unsigned char *m_compressionTable; /// Compression table
  
};

#endif // US_LOG_COMPRESSOR_H
