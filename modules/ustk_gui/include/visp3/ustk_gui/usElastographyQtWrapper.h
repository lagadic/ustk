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
 * @file usElastographyQtWrapper.h
 * @brief Qt wrapper for usElastography class.
 */

#ifndef __usElastographyQtWrapper_h_
#define __usElastographyQtWrapper_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_ELASTOGRAPHY)

#include <QObject>
#include <visp3/ustk_elastography/usElastography.h>

/**
 * @class usElastographyQtWrapper
 * @brief Qt wrapper for usElastography class.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usElastographyQtWrapper : public QObject
{
  Q_OBJECT
public:
  usElastographyQtWrapper();
  ~usElastographyQtWrapper();

  void setROI(int tx, int ty, int tw, int th);

public slots:
  void updateFrame(const usImageRF2D<short int> &img);

signals:
  void elastoReady(vpImage<unsigned char>);

private:
  usElastography m_elastography;
};
#endif // QT && FFTW
#endif // __usElastographyQtWrapper_h_
