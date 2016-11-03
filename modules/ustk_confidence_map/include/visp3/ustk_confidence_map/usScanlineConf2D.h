/****************************************************************************
 *
 * This file is part of the UsConfidenceMaps software.
 * Copyright (C) 2013 - 2016 by Inria. All rights reserved.
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
 * Authors:
 * Pierre Chatelain
 * Alexandre Krupa
 *
 *****************************************************************************/

#ifndef US_SCANLINE_CONF_2D_H
#define US_SCANLINE_CONF_2D_H

#include <cfloat>
#include <iostream>
#include <visp/vpImage.h>


class VISP_EXPORT usScanlineConf2D {

 public:

enum usScanLineConfidenceType {
  US_CONF_MAX, US_CONF_INTEGRATION
};
  /**
   * Default constructor.
   */
  usScanlineConf2D();

  /**
   * Initialize the confidence map processor.
   * Sets the boundary conditions.
   */
  void init(usScanLineConfidenceType type);

  /**
   * Run the confidence map processor.
   */
  void run(vpImage<unsigned char> &Dst, const vpImage<unsigned char> &Src);

 private:
  usScanLineConfidenceType m_type;
};

#endif // US_SCANLINE_CONF_2D_H
