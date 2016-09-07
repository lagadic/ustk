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
 * @file usData.h
 * @brief Generic ultrasound data.
 * @author Pierre Chatelain
 */

#ifndef US_DATA_H
#define US_DATA_H

//#include <UsTk/usTkConfig.h>

/**
 * Storage element type.
 */
enum usElementType {
  US_UNKNOWN, US_CHAR, US_UCHAR, US_SHORT, US_USHORT, US_LONG, US_ULONG, US_INT, US_UINT,
  US_FLOAT, US_DOUBLE, US_VECTOR
};

/**
 * Ultasound scanner type.
 *
 * Defines the ultrasound scanner used for acquisition.
 */
enum usScannerType {
  UNKNOWN_SCANNER, SONIX_RP, SONIX_TOUCH, SONOSITE
};

/**
 * Ultasound probe type.
 *
 * Defines the ultrasound probe used for acquisition.
 *
 * @todo Add more probe types.
 */
enum usProbeType {
  UNKNOWN_PROBE, US_4DC7, SS_C60
};

/**
 * Ultrasound data mode.
 *
 * Defines the ultrasound data mode (prescan, postscan, dimensionality).
 */
enum usDataMode {
  UNKNOWN_MODE, PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D, POSTSCAN_3D, RF_2D, RF_3D
};

/**
 * @class usData
 * @brief Generic class for ultrasound data.
 * @author Pierre Chatelain
 *
 * This class represents a generic ultrasound image.
 */
class /*USTK_EXPORT*/ usData {
 public:
  /**
   * Constructor.
   */
  usData();

  /**
   * Destructor.
   */
  virtual ~usData();

  /**
   * Assignment operator.
   */
  usData& operator=(const usData& other);

  /**
   * Get the data modality (PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D, POSTSCAN_3D).
   */
  usDataMode getMode() const;

  /**
   * Get the scanner type.
   */
  usScannerType getScannerType() const;

  /**
   * Get the probe type.
   */
  usProbeType getProbeType() const;

  /**
   * Get the data index.
   */
  unsigned int getDataIdx() const;

  /**
   * Set the data timestamp.
   */
  double getTimestamp() const;

  /**
   * Get the x-coordinate of the volume origin.
   */
  double getOriginX() const;

  /**
   * Get the y-coordinate of the volume origin.
   */
  double getOriginY() const;

  /**
   * Get the z-coordinate of the volume origin.
   */
  double getOriginZ() const;

  /**
   * Set the data modality (PRESCAN_2D, PRESCAN_3D, POSTSCAN_2D, POSTSCAN_3D).
   */
  void setMode(usDataMode mode);

  /**
   * Set the scanner type (SONIX_RP, SONIX_TOUCH, SONOSITE).
   */
  void setScannerType(usScannerType scannerType);

  /**
   * Set the probe type (US_4DC7, SS_C60).
   */
  void setProbeType(usProbeType probeType);

  /**
   * Set the data index.
   */
  void setDataIdx(unsigned int idx);

  /**
   * Set the data timestamp.
   */
  void setTimestamp(double timestamp);

  /**
   * Set the x-coordinate of the volume origin.
   */
  void setOriginX(double originX);

  /**
   * Set the y-coordinate of the volume origin.
   */
  void setOriginY(double originY);

  /**
   * Set the z-coordinate of the volume origin.
   */
  void setOriginZ(double originZ);

  /**
   * Set the coordinates of the volume origin.
   */
  void setOrigin(double originX, double originY, double originZ);

  /**
   * Print data information.
   */
  virtual void printInfo();

 protected:
  usDataMode m_mode;
  usScannerType m_scannerType;
  usProbeType m_probeType;
  unsigned int m_dataIdx;
  double m_timestamp;
  double m_originX;
  double m_originY;
  double m_originZ;

};

#endif // US_DATA_H
