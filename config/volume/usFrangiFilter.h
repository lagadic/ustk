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
 * @file usFrangiFilter.h
 * @brief 3D vessel detector
 * @author Pierre Chatelain
 */

#ifndef US_FRANGI_FILTER_H
#define US_FRANGI_FILTER_H

#include <UsTk/usVolume.h>

class usFrangiFilter {

 public:
  usFrangiFilter();
  
  void init();

  void setInput(usVolume<unsigned char> *Src);

  double getOutput(unsigned int x, unsigned int y, unsigned int z);

  double computeFrangi(unsigned int x, unsigned int y, unsigned int z);

 private:
  double m_a;
  double m_b;
  double m_c;
  double m_minScale;
  double m_maxScale;
  unsigned int m_dimx;
  unsigned int m_dimy;
  unsigned int m_dimz;
  usVolume<unsigned char> *m_input;
  usVolume<double> m_output;
  usVolume<double> m_Fxx;
  usVolume<double> m_Fyy;
  usVolume<double> m_Fzz;
  usVolume<double> m_Fxy;
  usVolume<double> m_Fxz;
  usVolume<double> m_Fyz;
  usVolume<double> m_Hxx;
  usVolume<double> m_Hyy;
  usVolume<double> m_Hzz;
  usVolume<double> m_Hxy;
  usVolume<double> m_Hxz;
  usVolume<double> m_Hyz;
  usVolume<bool> m_available;
};

#endif
