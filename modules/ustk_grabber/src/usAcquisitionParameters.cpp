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

#include <visp3/ustk_grabber/usAcquisitionParameters.h>
#include <visp3/core/vpException.h>


/**
* Constructor.
*/
usAcquisitionParameters::usAcquisitionParameters() :
  m_transmitFrequency(0), m_samplingFrequency(0), m_imagingMode(0),m_postScanMode(false), m_postScanHeigh(0), m_postScanWidth(0),
  m_imageDepth(0), m_sector(0), m_activateMotor(false), m_motorPosition(0),m_framesPerVolume(0), m_degreesPerFrame(0),
  m_transmitFrequencyMin(0), m_samplingFrequencyMin(0), m_imagingModeMin(0), m_imageDepthMin(0), m_sectorMin(0),
  m_motorPositionMin(0), m_framesPerVolumeMin(0), m_degreesPerFrameMin(0), m_transmitFrequencyMax(0), m_samplingFrequencyMax(0),
  m_imagingModeMax(0), m_imageDepthMax(0), m_sectorMax(0), m_motorPositionMax(0), m_framesPerVolumeMax(0), m_degreesPerFrameMax(0)
{

}

/**
* Destructor.
*/
usAcquisitionParameters::~usAcquisitionParameters()
{

}

/**
* Setter for motor activation.
* @param activateMotor Boolean to activate the motor sweep (true) or not (false).
*/
void usAcquisitionParameters::setActivateMotor(bool activateMotor) {
  m_activateMotor =  activateMotor;
}

/**
* Setter for degrees per frame.
* @param degreesPerFrame Angle in degrees between 2 sucessive frames.
*/
void usAcquisitionParameters::setDegreesPerFrame(int degreesPerFrame) {
  m_degreesPerFrame =  degreesPerFrame;
}

/**
* Setter for degreesPerFrame max.
* @param degreesPerFrameMax Max angle in degrees between 2 sucessive frames.
*/
void usAcquisitionParameters::setDegreesPerFrameMax(int degreesPerFrameMax) {
  m_degreesPerFrameMax =  degreesPerFrameMax;
}

/**
* Setter for degreesPerFrameMin.
* @param degreesPerFrameMin Min angle in degrees between 2 sucessive frames.
*/
void usAcquisitionParameters::setDegreesPerFrameMin(int degreesPerFrameMin) {
  m_degreesPerFrameMin =  degreesPerFrameMin;
}

/**
* Setter for framesPerVolume.
* @param framesPerVolume Number of frames in a volume acquisition.
*/
void usAcquisitionParameters::setFramesPerVolume(int framesPerVolume) {
  m_framesPerVolume =  framesPerVolume;
}

/**
* Setter for framesPerVolume.
* @param framesPerVolumeMax Maximum number of frames in a volume acquisition.
*/
void usAcquisitionParameters::setFramesPerVolumeMax(int framesPerVolumeMax) {
  m_framesPerVolumeMax =  framesPerVolumeMax;
}

/**
* Setter for framesPerVolumeMin.
* @param framesPerVolumeMin Minimum number of frames in a volume acquisition.
*/
void usAcquisitionParameters::setFramesPerVolumeMin(int framesPerVolumeMin) {
  m_framesPerVolumeMin =  framesPerVolumeMin;
}

/**
* Setter for imageDepth.
* @param imageDepth Depth of the echo image, in millimeters.
*/
void usAcquisitionParameters::setImageDepth(int imageDepth) {
  m_imageDepth =  imageDepth;
}

/**
* Setter for imageDepthMax.
* @param imageDepthMax Max depth of the probe for the echo image, in millimeters.
*/
void usAcquisitionParameters::setImageDepthMax(int imageDepthMax) {
  m_imageDepthMax =  imageDepthMax;
}

/**
* Setter for imageDepthMin.
* @param imageDepthMin Min depth of the probe for the echo image, in millimeters.
*/
void usAcquisitionParameters::setImageDepthMin(int imageDepthMin) {
  m_imageDepthMin =  imageDepthMin;
}

/**
* Setter for imagingMode.
* @param imagingMode Code for imaging mode (0 = pre-scan B-MODE, RF mode = 12)
*/
void usAcquisitionParameters::setImagingMode(int imagingMode) {
  m_imagingMode =  imagingMode;
}

/**
* Setter for imagingModeMax.
* @param imagingModeMax Code for imaging mode max.
*/
void usAcquisitionParameters::setImagingModeMax(int imagingModeMax) {
  m_imagingModeMax =  imagingModeMax;
}

/**
* Setter for imagingModeMin.
* @param imagingModeMin Code for imaging mode min.
*/
void usAcquisitionParameters::setImagingModeMin(int imagingModeMin) {
  m_imagingModeMin =  imagingModeMin;
}

/**
* Setter for motorPosition, to set a fixed motor position.
* @param motorPosition Motor position in degrees.
*/
void usAcquisitionParameters::setMotorPosition(int motorPosition) {
  m_motorPosition =  motorPosition;
}

/**
* Setter for motorPositionMax.
* @param motorPosition Max motor position in degrees of the current probe.
*/
void usAcquisitionParameters::setMotorPositionMax(int motorPositionMax) {
  m_motorPositionMax =  motorPositionMax;
}

/**
* Setter for motorPositionMin.
* @param motorPositionMin Min motor position in degrees of the current probe.
*/
void usAcquisitionParameters::setMotorPositionMin(int motorPositionMin) {
  m_motorPositionMin =  motorPositionMin;
}

/**
* Setter for postScanHeigh.
* @param postScanHeigh Height of the post-scan image in px.
*/
void usAcquisitionParameters::setPostScanHeigh(int postScanHeigh) {
  m_postScanHeigh =  postScanHeigh;
}

/**
* Setter for postScanMode.
* @param postScanMode Boolean to acquire scan-converted images (true), or not (false).
*/
void usAcquisitionParameters::setPostScanMode(bool postScanMode) {
  m_postScanMode =  postScanMode;
}

/**
* Setter for postScanWidth.
* @param postScanWidth Width of the post-scan image in px.
*/
void usAcquisitionParameters::setPostScanWidth(int postScanWidth) {
  m_postScanWidth =  postScanWidth;
}

/**
* Setter for samplingFrequency.
* @param samplingFrequency Sampling frequency of the acquisition (Hz).
*/
void usAcquisitionParameters::setSamplingFrequency(int samplingFrequency) {
  m_samplingFrequency =  samplingFrequency;
}

/**
* Setter for samplingFrequencyMax.
* @param samplingFrequencyMax Max sampling frequency of the acquisition (Hz).
*/
void usAcquisitionParameters::setSamplingFrequencyMax(int samplingFrequencyMax) {
  m_samplingFrequencyMax =  samplingFrequencyMax;
}

/**
* Setter for samplingFrequencyMin.
* @param samplingFrequencyMin Min sampling frequency of the acquisition (Hz).
*/
void usAcquisitionParameters::setSamplingFrequencyMin(int samplingFrequencyMin) {
  m_samplingFrequencyMin =  samplingFrequencyMin;
}

/**
* Setter for sector.
* @param sector Percentage of the total width of the transducers used to acquire an image.
*/
void usAcquisitionParameters::setSector(int sector) {
  m_sector =  sector;
}

/**
* Setter for sectorMax.
* @param sectorMax Max percentage of the width of the transducers used to acquire an image.
*/
void usAcquisitionParameters::setSectorMax(int sectorMax) {
  m_sectorMax =  sectorMax;
}

/**
* Setter for sectorMin.
* @param sectorMax Min percentage of the width of the transducers used to acquire an image.
*/
void usAcquisitionParameters::setSectorMin(int sectorMin) {
  m_sectorMin =  sectorMin;
}

/**
* Setter for transmitFrequency.
* @param transmitFrequency Acoustic frequency sent with the transducers (Hz).
*/
void usAcquisitionParameters::setTransmitFrequency(int transmitFrequency) {
  m_transmitFrequency =  transmitFrequency;
}

/**
* Setter for transmitFrequencyMax.
* @param transmitFrequency Max acoustic frequency sent with the transducers (Hz).
*/
void usAcquisitionParameters::setTransmitFrequencyMax(int transmitFrequencyMax) {
  m_transmitFrequencyMax =  transmitFrequencyMax;
}

/**
* Setter for transmitFrequencyMin.
* @param transmitFrequency Max acoustic frequency sent with the transducers (Hz).
*/
void usAcquisitionParameters::setTransmitFrequencyMin(int transmitFrequencyMin) {
  m_transmitFrequencyMin =  transmitFrequencyMin;
}


