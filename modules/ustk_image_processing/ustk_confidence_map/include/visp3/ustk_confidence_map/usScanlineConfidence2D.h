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
 * Alexandre Krupa
 *
 *****************************************************************************/

#ifndef __usScanlineConfidence2D_h_
#define __usScanlineConfidence2D_h_

#include <cfloat>
#include <iostream>

#include <visp3/ustk_core/usImagePreScan2D.h>

/**
* @class usScanlineConfidence2D
* @brief Process a pre-scan image to determine the confidence map.
* @ingroup module_ustk_confidence_map
*/
class VISP_EXPORT usScanlineConfidence2D {

 public:

  usScanlineConfidence2D();

  virtual ~usScanlineConfidence2D();

  void run(usImagePreScan2D<unsigned char> &preScanConfidence, const usImagePreScan2D<unsigned char> &preScanImage);
};

#endif // US_SCANLINE_CONF_2D_H
