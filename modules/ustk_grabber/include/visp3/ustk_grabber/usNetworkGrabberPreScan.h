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

#ifndef __usNetworkGrabberPreScan_h_
#define __usNetworkGrabberPreScan_h_

#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5)

#include <visp3/ustk_grabber/usNetworkGrabber.h>
#include <visp3/ustk_core/usImagePreScan2D.h>

/**
 * @class usNetworkGrabberPreScan
 * @brief Specific class to grab pre-scan frames on the network.
 * @ingroup module_ustk_grabber
 */
class VISP_EXPORT usNetworkGrabberPreScan : public usNetworkGrabber
{
  Q_OBJECT
public:

  explicit usNetworkGrabberPreScan(usNetworkGrabber *parent = 0);
  ~usNetworkGrabberPreScan();

  void dataArrived();

signals:
  void newFrameArrived(usImagePreScan2D<unsigned char>*);

protected:
  void invertRowsCols();

private:
  //grabbed image
  usImagePreScan2D<unsigned char> m_grabbedImage;
  usImagePreScan2D<unsigned char> m_outputImage; //we have to invert (i-j) in the image grabbed
};

#endif // QT4 || QT5
#endif // __usNetworkGrabberPreScan_h_
