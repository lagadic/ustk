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

#include "usDataPostscan3D.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>

usDataPostscan3D::usDataPostscan3D()
  : usVolume<unsigned char>()
{
  setOrigin(0.0, 0.0, 0.0);
  setMode(POSTSCAN_3D);
}

usDataPostscan3D::usDataPostscan3D(unsigned int dimx, unsigned int dimy, unsigned int dimz,
				   float esx, float esy, float esz)
  : usVolume<unsigned char>(dimx, dimy, dimz, esx, esy, esz)
{
  setOrigin(0.0, 0.0, 0.0);
  setMode(POSTSCAN_3D);
}

usDataPostscan3D::usDataPostscan3D(unsigned int dimx, unsigned int dimy, unsigned int dimz,
				   float esx, float esy, float esz, float ox, float oy, float oz)
  : usVolume<unsigned char>(dimx, dimy, dimz, esx, esy, esz)
{
  setOrigin(ox, oy, oz);
  setMode(POSTSCAN_3D);
}

usDataPostscan3D::usDataPostscan3D(const usDataPostscan3D &volume, const bool copy)
  : usVolume<unsigned char>(volume, copy)
{
  setOrigin(volume.getOriginX(), volume.getOriginY(), volume.getOriginZ());
  setMode(POSTSCAN_3D);
}

void usDataPostscan3D::printInfo()
{
  std::cout << "mode: " << m_mode << std::endl
	    << "scanner: " << m_scannerType << std::endl
	    << "probe: " << m_probeType << std::endl
	    << "data index: " << m_dataIdx << std::endl
	    << "timestamp: " << m_timestamp << std::endl
	    << "dimensions: " << getDimX() << "x" << getDimY() << "x" << getDimZ() << std::endl
	    << "origin: (" << m_originX << ", " << m_originY << ", " << m_originZ << ")" << std::endl;
}
