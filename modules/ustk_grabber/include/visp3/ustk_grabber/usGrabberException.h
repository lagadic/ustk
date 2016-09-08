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

#ifndef US_GRABBER_EXCEPTION_H
#define US_GRABBER_EXCEPTION_H

/**
 * @file usGrabberException.h
 * @brief Exception that can be emitted by usGrabber and its derivates.
 * @author Pierre Chatelain
 */

#include <visp/vpException.h>
#include <iostream>
#include <string>

/**
 * @class usGrabberException
 * @brief Error that can be emited by the vpRobot class and its derivates.
 * @author Pierre Chatelain
 */
class VISP_EXPORT usGrabberException : public vpException
{
  public:
  /**
   * @brief Lists the possible error than can be emitted while calling usGrabber member.
   */
    enum errorGrabberCode
    {

      /** Error while calling accept(). */
      acceptError,

      /** Error while calling bind(). */
      bindError,

      /** Error while calling listen(). */
      listenError,

      /** Error while calling recv(). */
      recvError,

      /** Error while calling recvfrom(). */
      recvfromError,

      /** Error while calling socket(). */
      socketError,

      /** Error opening file. */
      fileError,

      /** End of sequence */
      endOfSequence,

      /** Missing configuration variable */
      missingConfigVar,
      
      /** Unsupported grabber */
      unsupportedGrabber,

      /** Index error */
      indexError
    } ;

  public:
    usGrabberException (const int id,  const char* format, ...)
    {
      this->code = id;
      va_list args;
      va_start(args, format);
      setMessage(format, args);
      va_end (args);
    }

    usGrabberException (const int id, const std::string & msg)
      : vpException(id, msg)
    {
    }

    usGrabberException (const int id)
      : vpException(id)
    {
    }

};

#endif
