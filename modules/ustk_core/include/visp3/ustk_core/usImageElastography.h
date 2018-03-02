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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usImageElastography.h
 * @brief Elastography image : contains a 2D B-Mode image (pre-scan or post-scan), with an overlaying colored sub-image
 * to display the strain map of the sub-image region.
 */

#ifndef __usImageElastography_h_
#define __usImageElastography_h_

#include <visp3/core/vpRGBa.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>

/*!
  @class usImageElastography
  @brief Elastography image : contains a 2D B-Mode image (pre-scan or post-scan), with an overlaying colored sub-image
to display the strain map of the sub-image region.
  @ingroup module_ustk_core

  This class contains in fact 2 images : the ultrasound image (pre-scan or post-scan), and a second image corresponding
to the tissue strain map of a region of interest in the ultrasound image.
  The strain map image is displayed as a colored overlay of the grayscale ultrasound image.

  @warning For internal process reason the ultrasound image has to be set first. And then when the strain map is set,
the "mix" of the 2 image is done.

  The following figure represents the output of usImageElastography.
  \image html img-usImageElastography.png
 */

class VISP_EXPORT usImageElastography
{

public:
  usImageElastography();
  usImageElastography(const vpImage<unsigned char> &ultrasoundImage, const vpImage<unsigned char> &strainMap,
                      unsigned int heightPosition, unsigned int widthPosition);
  virtual ~usImageElastography();

  vpImage<vpRGBa> getElastoImage();

  void setStrainMap(const vpImage<unsigned char> &strainMap, unsigned int heightPosition, unsigned int widthPosition);
  void setUltrasoundImage(const vpImage<unsigned char> &ultrasoundImage);

private:
  void computeElastographyImage();

  vpImage<unsigned char> m_ultrasoundImage;
  vpImage<unsigned char> m_strainMap;
  vpImage<vpRGBa> m_elastoImage;

  unsigned int m_heigthPosition;
  unsigned int m_widthPosition;
};
#endif // __usImageElastography_h_
