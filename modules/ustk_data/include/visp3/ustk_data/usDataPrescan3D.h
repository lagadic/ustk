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
 * @file usDataPrescan3D.h
 * @brief 3D prescan ultrasound data.
 * @author Pierre Chatelain
 */

#ifndef US_DATA_PRESCAN_3D_H
#define US_DATA_PRESCAN_3D_H

#include <iostream>
#include <vector>
#include <algorithm>

//#include <UsTk/usTkConfig.h>
#include <visp3/ustk_data/usData.h>
#include <visp3/ustk_volume/usVolume.h>

/**
 * @class usDataPrescan3D
 * @brief 3D prescan ultrasound data.
 *
 * This class represents a 3D prescan ultrasound volume.
 */
class /*USTK_EXPORT*/ usDataPrescan3D : public usData, public usVolume<unsigned char>
{
public:
  /**
   * Constructor.
   */
  usDataPrescan3D();

  /**
   * Constructor.
   * Sets the dimensions of the volume.
   */
  usDataPrescan3D(unsigned int AN, unsigned int LN, unsigned int FN);

  /**
   * Constructor.
   * Sets all parameters of the volume.
   */
  usDataPrescan3D(unsigned int AN, unsigned int LN, unsigned int FN,
		  float probeRadius, float motorRadius,
		  float lineAngle, float frameAngle, float resolution,
		  float BSampleFreq, float probeElementPitch);
  
  /**
   * Copy constructor. By default performs a deep copy.
   */
  usDataPrescan3D(const usDataPrescan3D& other, const bool copy=true);

  /**
   * Assignment operator.
   */
  usDataPrescan3D& operator=(const usDataPrescan3D& other);

  /**
   * Get the number of elements per A-line.
   */
  unsigned int getAN() const { return getDimX(); }

  /**
   * Get the number of A-lines per frame.
   */
  unsigned int getLN() const { return getDimY(); }

  /**
   * Get the number of frames per volume.
   */
  unsigned int getFN() const { return getDimZ(); }

  /**
   * Get the probe radius.
   */
  float getProbeRadius() const { return m_probeRadius; }

  /**
   * Set the probe radius.
   */
  void setProbeRadius(float probeRadius) { m_probeRadius = probeRadius; }

  /**
   * Get the motor radius.
   */
  float getMotorRadius() const { return m_motorRadius; }

  /**
   * Set the motor radius.
   */
  void setMotorRadius(float motorRadius) { m_motorRadius = motorRadius; }

  /**
   * Get the line angle.
   */
  float getLineAngle() const { return m_lineAngle; }

  /**
   * Set the line angle.
   */
  void setLineAngle(float lineAngle) { m_lineAngle = lineAngle; }

  /**
   * Get the frame angle.
   */
  float getFrameAngle() const { return m_frameAngle; }

  /**
   * Set the frame angle.
   */
  void setFrameAngle(float frameAngle) { m_frameAngle = frameAngle; }

  /**
   * Get the resolution.
   */
  float getResolution() const { return m_resolution; }

  /**
   * Set the resolution.
   */
  void setResolution(float resolution) { m_resolution = resolution; }

  /**
   * Get the B-sample frequency.
   */
  float getBSampleFreq() const { return m_BSampleFreq; }

  /**
   * Set the B-sample frequency.
   */
  void setBSampleFreq(float BSampleFreq) { m_BSampleFreq = BSampleFreq; }

  /**
   * Get the probe element pitch.
   */
  float getProbeElementPitch() const { return m_probeElementPitch; }

  /**
   * Set the probe element pitch.
   */
  void setProbeElementPitch(float probeElementPitch) { m_probeElementPitch = probeElementPitch; }
  
protected:
  float m_probeRadius; /**< Probe radius (m). */
  float m_motorRadius; /**< Motor radius (m). */
  float m_lineAngle; /**< Angle b/w two neighboring lines (rad). */
  float m_frameAngle; /**< Angle b/w two neighboring frames (rad). */
  float m_resolution; /**< Distance b/w two neighboring elements in a A-line (m). */
  float m_BSampleFreq; /**< B-sample frequency (Hz). */
  float m_probeElementPitch; /**< Probe element pitch (m). */
};

#endif // US_DATA_PRESCAN_3D_H
