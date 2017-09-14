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
 * @file usNetworkGrabberPostScan2D.h
 * @brief Grabber used to grab post-scan frames from ultrasonix station, using a tcp connection.
 */

#ifndef __usNetworkGrabberPostScan2D_h_
#define __usNetworkGrabberPostScan2D_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)

#include <vector>

#include <visp3/ustk_grabber/usNetworkGrabber.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_grabber/usFrameGrabbedInfo.h>

/**
 * @class usNetworkGrabberPostScan2D
 * @brief Specific class to grab post-scan frames from the ultrasound station on the network.
 * @ingroup module_ustk_grabber
 *
 * The following figure details the network communication process and summarizes the steps to follow to acquire ultrasound images :
 * \image html img-usNetworkGrabber.png
 *
 * This grabber manages a buffer system to avoid multiple copy of the frames.
 * The acquire() method returns a pointer on a new frame, you can acess and modify the frame (it is thread-safe).
 * Acquire() can be blocking, the behaviour depends on how often you call it :
 * - If you call acquire() faster than the frames are arriving on the network, it is blocking to wait next frame coming.
 * - If you call it slower you will loose frames, but you will get the last frame available.
 */
class VISP_EXPORT usNetworkGrabberPostScan2D : public usNetworkGrabber
{
  Q_OBJECT
public:

  explicit usNetworkGrabberPostScan2D(usNetworkGrabber *parent = 0);
  ~usNetworkGrabberPostScan2D();

  usFrameGrabbedInfo<usImagePostScan2D<unsigned char> > * acquire();

  void dataArrived();

  bool isFirstFrameAvailable() {return m_firstFrameAvailable;}

signals:
  void newFrameAvailable();

private:
  // Output images
  std::vector<usFrameGrabbedInfo<usImagePostScan2D<unsigned char> > *> m_outputBuffer;
  bool m_firstFrameAvailable;

  //to manage ptrs switch init
  bool m_swichOutputInit;
};

#endif // QT4 || QT5
#endif // __usNetworkGrabberPostScan2D_h_
