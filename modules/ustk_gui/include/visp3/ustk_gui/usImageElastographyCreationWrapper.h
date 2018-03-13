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
 * @file usImageElastographyCreationWrapper.h
 * @brief Qt wrapper for elastography image creation (pre-scan image with a colored rectangle overlay for elastograpy).
 */

#ifndef __usImageElastographyCreationWrapper_h_
#define __usImageElastographyCreationWrapper_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_ELASTOGRAPHY)

#include <QApplication>
#include <QImage>
#include <QObject>
#include <visp3/ustk_core/usImageElastography.h>
#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_gui/usElastographyQtWrapper.h>

/**
 * @class usImageElastographyCreationWrapper
 * @brief Qt wrapper for colored elastography image creation (pre-scan image with a colored rectangle overlay for
 * elastograpy).
 * @ingroup module_ustk_gui
 */
class VISP_EXPORT usImageElastographyCreationWrapper : public QObject
{
  Q_OBJECT
public:
  usImageElastographyCreationWrapper();
  ~usImageElastographyCreationWrapper();

public slots:
  void updateFrame(usImageRF2D<short int> &img);
  void setROI(unsigned int i, unsigned int j, unsigned int height, unsigned int width);

private slots:
  void strainMapReadySlot(vpImage<unsigned char>);

signals:
  void elastographyImageReady(vpImage<vpRGBa>);

private:
  // processing objects
  usElastographyQtWrapper m_elastography;
  usImageElastography m_elastographyImageCreation;
  usRFToPreScan2DConverter m_RFConverter;

  // data objects
  usImagePreScan2D<unsigned char> m_preScanImage;

  unsigned int m_heighPositionROI;
  unsigned int m_widthPositionROI;
};
#endif // QT && ELASTO
#endif // __usImageElastographyCreationWrapper_h_
