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

#include <visp3/ustk_core/usLogCompressor.h>

#if defined(USTK_HAVE_FFTW)

#include <visp3/core/vpMath.h>

/**
* Constructor.
*/
usLogCompressor::usLogCompressor()
{
  m_alpha = 0.5;
  m_compressionTable = new unsigned char[65536];
  double denom = exp(m_alpha * log(65535.0)) / 255.0;
  for (unsigned int i = 0; i < 65536; ++i)
    m_compressionTable[i] = (unsigned char)(exp(m_alpha * log(i)) / denom);
}

/**
* Destructor.
*/
usLogCompressor::~usLogCompressor()
{
  if (m_compressionTable) {
    delete[] m_compressionTable;
    m_compressionTable = NULL;
  }
}

/**
* Initialize the log-compression filter.
* @param alpha The contrast parameter.
*/
void usLogCompressor::init(double alpha)
{
  if (alpha != m_alpha) {
    m_alpha = alpha;
    double denom = exp(m_alpha * log(65535.0)) / 255.0;
    for (unsigned int i = 0; i < 65536; ++i)
      m_compressionTable[i] = (unsigned char)(exp(m_alpha * log(i)) / denom);
  }
}

/**
* Run the log-compression filter.
* @param[out] dst Pointer to the destination data array.
* @param[in] src Pointer to the source data array.
* @param[in] size The size of the data array.
*
* Both \e dst and \e src must point to an array of size \e size.
*/
void usLogCompressor::run(unsigned char *dst, const double *src, unsigned int size)
{
  for (unsigned int i = 0; i < size; ++i)
    dst[i] = m_compressionTable[vpMath::round(src[i])];
}

#endif
