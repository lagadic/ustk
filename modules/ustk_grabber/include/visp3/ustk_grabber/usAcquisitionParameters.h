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
 * @file usAcquisitionParameters.h
 * @brief Class to store acquisition parameters for ultrasound station.
 */

#ifndef __usAcquisitionParameters_h_
#define __usAcquisitionParameters_h_

#include <visp3/core/vpConfig.h>

/**
 * @class usAcquisitionParameters
 * @brief Class to store acquisition parameters for ultrasound station.
 * @ingroup module_ustk_grabber
 */
class VISP_EXPORT usAcquisitionParameters
{
public:

  /// For 4DC7 3D probe motor movement
  typedef enum
    {
      /// motor not moving (2D case)
      US_4DC7_STATIC_MOTOR = 0,

      /// 4 motor steps per frame = 0.7317 degrees
      US_4DC7_SMALL_ANGLE_PITCH = 4,

      /// 8 motor steps per frame = 1.4634 degrees
      US_4DC7_BIG_ANGLE_PITCH = 8
    } us4DC7Angles;


  usAcquisitionParameters();
  ~usAcquisitionParameters();

  //values
  bool getActivateMotor() const {return m_activateMotor;}

  int getAnglePerFrame () const {return m_anglePerFrame;}
  int getAnglePerFrameMax () const {return m_anglePerFrameMax;}
  int getAnglePerFrameMin () const {return m_anglePerFrameMin;}

  int getFramesPerVolume () const {return m_framesPerVolume;}
  int getFramesPerVolumeMax () const {return m_framesPerVolumeMax;}
  int getFramesPerVolumeMin () const {return m_framesPerVolumeMin;}

  int getImageDepth () const {return m_imageDepth;}
  int getImageDepthMax () const {return m_imageDepthMax;}
  int getImageDepthMin () const {return m_imageDepthMin;}


  int getImagingMode () const {return m_imagingMode;}
  int getImagingModeMax () const {return m_imagingModeMax;}
  int getImagingModeMin () const {return m_imagingModeMin;}

  int getMotorPosition () const {return m_motorPosition;}
  int getMotorPositionMax () const {return m_motorPositionMax;}
  int getMotorPositionMin () const {return m_motorPositionMin;}

  int getPostScanHeigh () const {return m_postScanHeigh;}

  bool getPostScanMode () const {return m_postScanMode;}

  int getPostScanWidth () const {return m_postScanWidth;}

  int getSamplingFrequency () const {return m_samplingFrequency;}
  int getSamplingFrequencyMax () const {return m_samplingFrequencyMax;}
  int getSamplingFrequencyMin () const {return m_samplingFrequencyMin;}

  int getSector () const {return m_sector;}
  int getSectorMax () const {return m_sectorMax;}
  int getSectorMin () const {return m_sectorMin;}

  int getTransmitFrequency () const {return m_transmitFrequency;}
  int getTransmitFrequencyMax () const {return m_transmitFrequencyMax;}
  int getTransmitFrequencyMin () const {return m_transmitFrequencyMin;}

  //setters
  void setActivateMotor(bool activateMotor);

  void setAnglePerFrame(int anglePerFrame);
  void setAnglePerFrameMax(int anglePerFrameMax);
  void setAnglePerFrameMin(int anglePerFrameMin);

  void setFramesPerVolume(int framesPerVolume);
  void setFramesPerVolumeMax(int framesPerVolumeMax);
  void setFramesPerVolumeMin(int framesPerVolumeMin);

  void setImageDepth(int imageDepth);
  void setImageDepthMax(int imageDepthMax);
  void setImageDepthMin(int imageDepthMin);


  void setImagingMode(int imagingMode);
  void setImagingModeMax(int imagingModeMax);
  void setImagingModeMin(int imagingModeMin);

  void setMotorPosition(int motorPosition);
  void setMotorPositionMax(int motorPositionMax);
  void setMotorPositionMin(int motorPositionMin);

  void setPostScanHeigh(int postScanHeigh);

  void setPostScanMode(bool postScanMode);

  void setPostScanWidth(int postScanWidth);

  void setSamplingFrequency(int samplingFrequency);
  void setSamplingFrequencyMax(int samplingFrequencyMax);
  void setSamplingFrequencyMin(int samplingFrequencyMin);

  void setSector(int sector);
  void setSectorMax(int sectorMax);
  void setSectorMin(int sectorMin);

  void setTransmitFrequency(int transmitFrequency);
  void setTransmitFrequencyMax(int transmitFrequencyMax);
  void setTransmitFrequencyMin(int transmitFrequencyMin);

private :
  //frequencies
  int m_transmitFrequency;
  int m_samplingFrequency;

  //image type
  int m_imagingMode; // see ImagingModes.h
  bool m_postScanMode; //performs scan conversion on ultrasound station if true
  int m_postScanHeigh; // if post-scan mode, height of the frame (px)
  int m_postScanWidth; // if post-scan mode, width of the frame (px)

  int m_imageDepth; //in mm
  int m_sector; // in %

  //motor settings
  bool m_activateMotor; //to sweep the motor permanently

  // position of the motor in degrees : 0° = side of the fixation system for 4DC7 probe
  int m_motorPosition; // (used if activateMotor = false)

  // motor movement parameters
  int m_framesPerVolume; // (must be odd : always a central frame)
  int m_anglePerFrame; // angle between two frames in degrees

  // min values
  int m_transmitFrequencyMin;
  int m_samplingFrequencyMin;
  int m_imagingModeMin;
  int m_imageDepthMin;
  int m_sectorMin;
  int m_motorPositionMin;
  int m_framesPerVolumeMin;
  int m_anglePerFrameMin;

  // max values
  int m_transmitFrequencyMax;
  int m_samplingFrequencyMax;
  int m_imagingModeMax;
  int m_imageDepthMax;
  int m_sectorMax;
  int m_motorPositionMax;
  int m_framesPerVolumeMax;
  int m_anglePerFrameMax;
};
#endif // __usAcquisitionParameters_h_
