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
 * @file usGrabberFile.h
 * @brief Offline ultrasound data grabber (reading from file).
 * @author Pierre Chatelain
 */

#ifndef US_GRABBER_FILE_H
#define US_GRABBER_FILE_H

#include <string>
#include <UsTk/usGrabber.h>

/**
 * @class usGrabberFile
 * @brief Offline ultrasound data grabber (reading from file).
 * @author Pierre Chatelain
 *
 * Generic class for ultrasound data grabber.
 */
class usGrabberFile : public usGrabber {
 public:
  /**
   * Constructor.
   */
  usGrabberFile();

  /**
   * Initialization method.
   */
  void start();

  /**
   * Grab new data.
   *
   * @return true if there is new data available.
   */
  void grab();

  /**
   * Close input connection.
   */
  void stop();

  /**
   * Get the current data.
   */
  //usData *getData();

  /**
   * Set the sequence file name.
   */
  void setFileName(std::string dir, std::string file);

  /**
   * Get the sequence file name.
   */
  std::string getFileName();

  /**
   * Get the length of the image sequence.
   */
  int getSequenceLength();

 private:
  std::string getFileExtension(const std::string &filename);
  std::string getDirPath(const std::string &filename);
  std::string getShortFilename(const std::string &filename);
  int m_digits;

 protected:
  int m_idx;
  int m_sequenceLength;
  int m_dimensionality;
  size_t m_dataSize;
  FILE* m_pFile;
  std::string m_sequenceFileName;
  std::string m_dirName;
  std::string m_baseName;
  std::string m_extension;

};

#endif // US_GRABBER_FILE_H
