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
 * @file usNetworkGrabberPreScan.h
 * @brief Grabber used to grab pre-scan frames from ultrasonix station, using a tcp connection.
 */

#ifndef __usNetworkViewerPreScan_h_
#define __usNetworkViewerPreScan_h_

#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_HAVE_QT4) || defined(USTK_HAVE_QT5)

#include <visp3/ustk_grabber/usNetworkGrabberPreScan.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/gui/vpDisplayX.h>

class VISP_EXPORT usNetworkViewerPreScan : public QObject
{
  Q_OBJECT
public:

  explicit usNetworkViewerPreScan(QObject *parent = 0);
  ~usNetworkViewerPreScan();

  void setGrabber(usNetworkGrabberPreScan* grabber);

public slots :
  void updateDisplay(usImagePreScan2D<unsigned char>* newFrame);

private:
  //grabbed image
  usImagePreScan2D<unsigned char> m_grabbedImage;
  vpDisplayX display;
};

#endif // QT4 || QT5
#endif // __usNetworkGrabberPreScan_h_
