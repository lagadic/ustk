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
 * @file usDataPrescan2D.h
 * @brief 2D prescan ultrasound data.
 * @author Pierre Chatelain
 */

#ifndef US_DATA_PRESCAN_2D_H
#define US_DATA_PRESCAN_2D_H

#include <visp3/core/vpImage.h>

#include <visp3/ustk_data/usData.h>

/**
 * @class usDataPrescan2D
 * @brief 2D prescan ultrasound data.
 * @author Pierre Chatelain
 *
 * This class represents a 2D ultrasound prescan frame.
 */
class VISP_EXPORT usDataPrescan2D : public usData, public vpImage<unsigned char> {
 public:
  /**
   * Constructor.
   */
  usDataPrescan2D();

  /**
   * Initializing constructor.
   */
  usDataPrescan2D(unsigned int AN, unsigned int LN);

  /**
   * Initializing constructor.
   */
  usDataPrescan2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
		  float resolution, float BSampleFreq, float probeElementPitch);

  /**
   * Copy constructor.
   */
  usDataPrescan2D(const usDataPrescan2D &other, const bool copy=true);

  /**
   * Destructor.
   */
  ~usDataPrescan2D();

  void copyFrom(const vpImage<unsigned char> &I);

  /**
   * Get the number of A-samples in a line.
   */
  unsigned int getAN() const;

  /**
   * Get the number of lines.
   */
  unsigned int getLN() const;

  /**
   * Get the probe radius (m).
   */
  float getProbeRadius() const;

  /**
   * Get the line angle (rad).
   */
  float getLineAngle() const;

  /**
   * Get the linear resolution (m).
   */
  float getResolution() const;

  /**
   * Get the B-sample frequence (Hz).
   */
  float getBSampleFreq() const;

  /**
   * Get the probe element pitch.
   */
  float getProbeElementPitch() const;

  void setProbeRadius(float probeRadius);
  void setLineAngle(double angle);
  void setResolution(float resolution);
  void setBSampleFreq(double freq);
  void setProbeElementPitch(float probeElementPitch);

 private:
  float m_probeRadius;
  float m_lineAngle;
  float m_resolution;
  float m_BSampleFreq;
  float m_probeElementPitch;

};

#endif // US_DATA_PRESCAN_2D_H
