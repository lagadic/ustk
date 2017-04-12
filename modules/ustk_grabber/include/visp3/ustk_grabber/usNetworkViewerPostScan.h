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
 * @file usNetworkGrabberPostScan.h
 * @brief Grabber used to grab pre-scan frames from ultrasonix station, using a tcp connection.
 */

#ifndef __usNetworkViewerPostScan_h_
#define __usNetworkViewerPostScan_h_

#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5) && defined(VISP_HAVE_DISPLAY)

#include <visp3/ustk_grabber/usNetworkGrabberPostScan.h>
#include <visp3/ustk_core/usImagePostScan2D.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>

/**
 * @class usNetworkViewerPostScan
 * @brief Specific class to display pre-scan frames using Qt signals when a new frame is arrived on the network.
 * @ingroup module_ustk_grabber
 */
class VISP_EXPORT usNetworkViewerPostScan : public QObject
{
  Q_OBJECT
public:

  explicit usNetworkViewerPostScan(int frameWith=1, int frameHeight=1, QObject *parent = 0);
  ~usNetworkViewerPostScan();

public slots :
  void updateDisplay(usImagePostScan2D<unsigned char>* newFrame);

private:
  //grabbed image
  usImagePostScan2D<unsigned char> m_grabbedImage;

  //vpDisplay
#if defined(VISP_HAVE_X11)
  vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
  vpDisplayX display;
#elif defined(VISP_HAVE_GTK)
  vpDisplayGTK display;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV display;
#endif
};

#endif // QT5 && X11
#endif // __usNetworkViewerPostScan_h_
