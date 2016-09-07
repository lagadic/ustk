/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Authors:
 * Pierre Chatelain
 *
 *****************************************************************************/

/**
 * @file usGrabber.h
 * @brief Generic ultrasound data grabber.
 * @author Pierre Chatelain
 */

#ifndef US_GRABBER_H
#define US_GRABBER_H

#include <string>
#include <visp3/ustk_data/usData.h>

/**
 * Storage element type.
 */
enum usGrabberType {
  US_GRABBER_ULTRASONIX,
  US_GRABBER_V4L2
};

/**
 * @class usGrabber
 * @brief Generic ultrasound data grabber.
 * @author Pierre Chatelain
 *
 * Generic class for ultrasound data grabber.
 */
class usGrabber {
 public:
  /**
   * Constructor.
   */
  usGrabber();

  /**
   * Destructor.
   */
  virtual ~usGrabber();

  /**
   * Initialization method.
   */
  virtual void start() =0;

  /**
   * Grab new data.
   *
   * @return true if there is new data available.
   */
  virtual void grab() =0;

  virtual void grabFrame();

  /**
   * Close input connection.
   */
  virtual void stop() =0;

  /**
   * Get the current data.
   */
  usData *getData();

  /**
   * Get the imaging mode.
   */
  std::string getMode() const;

  double getResolution() const;
  
 protected:
  usData *m_data;
  std::string m_mode;

  double m_resolution;

};

#endif // US_DATA_H
