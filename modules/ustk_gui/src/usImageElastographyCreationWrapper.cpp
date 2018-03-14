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
* @file usImageElastographyCreationWrapper.cpp
* @brief Qt wrapper for colored elastography image creation (pre-scan image with a colored rectangle overlay for
 * elastograpy).
*/

#include <visp3/ustk_gui/usImageElastographyCreationWrapper.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_ELASTOGRAPHY)

/**
* Constructor.
*/
usImageElastographyCreationWrapper::usImageElastographyCreationWrapper()
  : QObject(), m_elastography(), m_elastographyImageCreation(), m_RFConverter(), m_preScanImage(), m_heighPositionROI(),
    m_widthPositionROI()
{
  connect(&m_elastography, SIGNAL(elastoReady(vpImage<unsigned char>)), this,
          SLOT(strainMapReadySlot(vpImage<unsigned char>)));
}

/**
* Destructor.
*/
usImageElastographyCreationWrapper::~usImageElastographyCreationWrapper() {}

/**
* ROI setter, coordinates are set in RF image pixel coordinates.
* @param i Top left row of the ROI.
* @param j Top left column of the ROI.
* @param height ROI height in px.
* @param width ROI width in px.
*/
void usImageElastographyCreationWrapper::setROI(unsigned int i, unsigned int j, unsigned int height, unsigned int width)
{
  m_elastography.setROI(j, i, width, height);
  m_heighPositionROI = i;
  m_widthPositionROI = j;
}

/**
* Slot to update the RF frame. This slot manages RF conversion and elastography process.
* @param img New RF image input for elastography computation and RF conversion.
*/
void usImageElastographyCreationWrapper::updateFrame(usImageRF2D<short int> &img)
{
  m_RFConverter.convert(img, m_preScanImage);
  m_elastographyImageCreation.setUltrasoundImage(m_preScanImage);
  m_elastography.updateFrame(img);
  qApp->processEvents();
}

/**
* Private slot called when the strain map is ready.
* @param image New strain map computed by usElastographyQtWrapper.
*/
void usImageElastographyCreationWrapper::strainMapReadySlot(vpImage<unsigned char> image)
{
  m_elastographyImageCreation.setStrainMap(image, m_heighPositionROI, m_widthPositionROI);
  emit(elastographyImageReady(m_elastographyImageCreation.getElastoImage()));
}
#endif
