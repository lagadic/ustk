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

#include <visp3/ustk_core/usImageElastography.h>

/*!
  \brief Constructor.
*/
usImageElastography::usImageElastography()
  : m_ultrasoundImage(), m_strainMap(), m_elastoImage(), m_heigthPosition(0), m_widthPosition(0)
{
}

/**
* Initializing constructor.
* @param ultrasoundImage Ultrasound image on which elastography was performed.
* @param strainMap Resulting image of elastography process.
* @param heightPosition Offset (in px) between top-left corner of the strain map and top-left corner of the ultrasound
* image in height direction.
* @param widthPosition Offset (in px) between top-left corner of the strain map and top-left corner of the ultrasound
* image in width direction.
*/
usImageElastography::usImageElastography(const vpImage<unsigned char> &ultrasoundImage,
                                         const vpImage<unsigned char> &strainMap, unsigned int heightPosition,
                                         unsigned int widthPosition)
  : m_ultrasoundImage(ultrasoundImage), m_strainMap(strainMap), m_elastoImage(), m_heigthPosition(heightPosition),
    m_widthPosition(widthPosition)
{
  computeElastographyImage();
}

/**
* Destructor.
*/
usImageElastography::~usImageElastography() {}

/**
* Get the resulting image, combinig ultrasound and elastography.
* @return The image resulting of the mix of ultrasound image and the strain map of a region of interest.
*/
vpImage<vpRGBa> usImageElastography::getElastoImage() { return m_elastoImage; }

/**
* Strain map image setter.
* @warning Make sure you already set the ultrasound image with setUltrasoundImage() before calling this method.
* @param m_strainMap
* @param heightPosition
* @param widthPosition
*/
void usImageElastography::setStrainMap(const vpImage<unsigned char> &strainMap, unsigned int heightPosition,
                                       unsigned int widthPosition)
{
  // image dimension check
  if (m_ultrasoundImage.getHeight() < strainMap.getHeight() + heightPosition ||
      m_ultrasoundImage.getWidth() < strainMap.getWidth() + widthPosition) {
    throw(
        vpException(vpException::dimensionError,
                    "usImageElastography::setStrainMap : the strainMap position is out of ultrasound image bounds !"));
  }

  m_strainMap = strainMap;
  m_heigthPosition = heightPosition;
  m_widthPosition = widthPosition;
  computeElastographyImage();
}

/**
* Ultrasound image setter.
* @param ultrasoundImage Ultrasound image : base image of elastography.
*/
void usImageElastography::setUltrasoundImage(const vpImage<unsigned char> &ultrasoundImage)
{
  m_ultrasoundImage = ultrasoundImage;
  m_elastoImage.resize(m_ultrasoundImage.getHeight(), m_ultrasoundImage.getWidth(), vpRGBa(0, 0, 0, 255));
}

void usImageElastography::computeElastographyImage()
{
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (unsigned int i = 0; i < m_elastoImage.getHeight(); i++) {
    for (unsigned int j = 0; j < m_elastoImage.getWidth(); j++) {
      vpRGBa newColor;
      // inside of the strainMap ROI
      if (i >= m_heigthPosition && i < m_heigthPosition + m_strainMap.getHeight() && j >= m_widthPosition &&
          j < m_widthPosition + m_strainMap.getWidth()) {
        newColor.R = m_strainMap[i - m_heigthPosition][j - m_widthPosition];
      } else { // oustide
        newColor.R = m_ultrasoundImage[i][j];
        newColor.G = m_ultrasoundImage[i][j];
        newColor.B = m_ultrasoundImage[i][j];
      }
      m_elastoImage[i][j] = newColor;
    }
  }
}
