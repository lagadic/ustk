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
* @file usElastographyQtWrapper.cpp
* @brief Qt wrapper for elastography computation.
*/

#include <visp3/ustk_gui/usElastographyQtWrapper.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(USTK_HAVE_ELASTOGRAPHY)

/**
* Constructor.
*/
usElastographyQtWrapper::usElastographyQtWrapper() : QObject(), m_elastography() {}

/**
* Destructor.
*/
usElastographyQtWrapper::~usElastographyQtWrapper() {}

/**
* ROI setter, coordinates are set in RF image pixel coordinates.
* @param tx Top left column of the ROI.
* @param ty Top left row of the ROI.
* @param tw ROI width in px.
* @param th ROI height in px.
*/
void usElastographyQtWrapper::setROI(int tx, int ty, int tw, int th) { m_elastography.setROI(tx, ty, tw, th); }

/**
* Slot to update the RF frame as input for elastography. The elastography is computed between the previous frame and the
* new one.
* @param img New RF image input for elastography computation.
*/
void usElastographyQtWrapper::updateFrame(const usImageRF2D<short int> &img)
{
  m_elastography.updateRF(img);
  vpImage<unsigned char> elasto = m_elastography.run();
  if (elasto.getHeight() != 0 && elasto.getWidth() != 0)
    emit elastoReady(elasto);
}

#endif
