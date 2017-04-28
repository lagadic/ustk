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
 * @file usNetworkGrabberRF.h
 * @brief Grabber used to grab pre-scan frames from ultrasonix station, using a tcp connection.
 */

#ifndef __usNetworkGrabberRF_h_
#define __usNetworkGrabberRF_h_

#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5)

#include <vector>

#include <visp3/ustk_grabber/usNetworkGrabber.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_grabber/usDataGrabbed.h>

/**
 * @class usNetworkGrabberRF
 * @brief Specific class to grab pre-scan frames on the network.
 * @ingroup module_ustk_grabber
 */
class VISP_EXPORT usNetworkGrabberRF : public usNetworkGrabber
{
  typedef enum {
    OUTPUT_FRAME_POSITION_IN_VEC = 0,
    MOST_RECENT_FRAME_POSITION_IN_VEC = 1,
    CURRENT_FILLED_FRAME_POSITION_IN_VEC = 2,
  }DataPositionInBuffer;
  Q_OBJECT
public:

  explicit usNetworkGrabberRF(usNetworkGrabber *parent = 0);
  ~usNetworkGrabberRF();

  usDataGrabbed<usImageRF2D<unsigned char> > * acquire();

  void dataArrived();

  bool isFirstFrameAvailable() {return m_firstFrameAvailable;}

signals:
  void newFrameAvailable();

protected:
  void invertRowsCols();

private:
  //grabbed image
  usDataGrabbed<usImageRF2D<unsigned char> > m_grabbedImage;

  // Output images : we have to invert (i <-> j) in the image grabbed
  std::vector<usDataGrabbed<usImageRF2D<unsigned char> > *> m_outputBuffer;
  bool m_firstFrameAvailable;

  //to manage ptrs switch init
  bool m_swichOutputInit;
};

#endif // QT4 || QT5
#endif // __usNetworkGrabberRF_h_
