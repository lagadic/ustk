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
 * @file usDataPostscan3D.h
 * @brief Volume handling.
 * @author Pierre Chatelain
 */

#ifndef US_DATA_POSTSCAN_3D_H
#define US_DATA_POSTSCAN_3D_H

#include <UsTk/usTkConfig.h>
#include <UsTk/usData.h>
#include <UsTk/usVolume.h>

/**
 * @class usDataPostscan3D
 * @brief Representation of a physical data volume.
 *
 * This class is used to represent 3D data with physical information such as element spacing.
 */
class USTK_EXPORT usDataPostscan3D : public usData, public usVolume<unsigned char>
{
public:
  /** Constructor  */
  usDataPostscan3D();

  /** Constructor.
   * Set the dimensions and element spacing of the volume.
   */
  usDataPostscan3D(unsigned int dimx, unsigned int dimy, unsigned int dimz, float esx, float esy, float esz);

  /** Constructor.
   * Set the dimensions, element spacing, and origin of the volume.
   */
  usDataPostscan3D(unsigned int dimx, unsigned int dimy, unsigned int dimz, float esx, float esy,
		   float esz, float ox, float oy, float oz);

  /** Copy constructor */
  usDataPostscan3D(const usDataPostscan3D &volume, const bool copy);

  /** Print information */
  void printInfo();
  
};

#endif // US_DATA_POSTSCAN_3D_H
