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


/**
* Constructor.
*/
usAcquisitionParameters::usAcquisitionParameters()
{

}

/**
* Destructor.
*/
usAcquisitionParameters::~usAcquisitionParameters()
{

}

/**
*
* @param
*/
void usAcquisitionParameters::setAcousticPower(int acousticPower)
{
  m_acousticPower = acousticPower;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBImageDepth(int bImageDepth)
{
  m_bImageDepth = bImageDepth;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBBImageDepth (int bBImageDepth)
{
  m_bBImageDepth = bBImageDepth;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBImageDepthA (int bImageDepthA)
{
  m_bImageDepthA = bImageDepthA;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBImageDepthB(int bImageDepthB)
{
  m_bImageDepthB = bImageDepthB;
}

/**
*
* @param
*/
void usAcquisitionParameters::setZoom(int zoom)
{
  m_zoom = zoom;
}

/**
*
* @param
*/
void usAcquisitionParameters::setTGC(int TGC)
{
  m_tGC = TGC;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBrightness(int brightness)
{
  m_brightness = brightness;
}

/**
*
* @param
*/
void usAcquisitionParameters::setContrast(int contrast)
{
  m_contrast = contrast;
}

/**
*
* @param
*/
void usAcquisitionParameters::setGamma(int gamma)
{
  m_gamma = gamma;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBMap(int bMap)
{
  m_bMap = bMap;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBSector(int bSector)
{
  m_bSector = bSector;
}

/**
*
*/
void usAcquisitionParameters::setBBSector(int bBSector)
{
  m_bBSector = bBSector;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBSectorA(int bSectorA)
{
  m_bSectorA = bSectorA;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBSectorB(int bSectorB)
{
  m_bSectorB = bSectorB;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBPersist(int bPersist)
{
  m_bPersist = bPersist;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBDynRange(int bDynRange)
{
  m_bDynRange = bDynRange;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBSteer(int bSteer)
{
  m_bSteer = bSteer;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBGain(int bGain)
{
  m_bGain = bGain;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBTxFreq(int bTxFreq)
{
  m_bTxFreq = bTxFreq;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBFocusDepth(int bFocusDepth)
{
  m_bFocusDepth = bFocusDepth;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBFocusCount(int bFocusCount)
{
  m_bFocusCount = bFocusCount;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBFocusSpacing(int bFocusSpacing)
{
  m_bFocusSpacing = bFocusSpacing;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBImageOpt(int bImageOpt)
{
  m_bImageOpt = bImageOpt;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBMru(int bMru)
{
  m_bMru = bMru;
}

/**
*
* @param
*/
void usAcquisitionParameters::setDualActiveDisplay(int dualActiveDisplay)
{
  m_dualActiveDisplay = dualActiveDisplay;
}

/**
*
* @param
*/
void usAcquisitionParameters::setQuadActiveDisplay(int quadActiveDisplay)
{
  m_quadActiveDisplay = quadActiveDisplay;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMMode(int mMode)
{
  m_mMode = mMode;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMZoom(int mZoom)
{
  m_mZoom = mZoom;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMPos(int mPos)
{
  m_mPos = mPos;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMDepth(int mDepth)
{
  m_mDepth = mDepth;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMSweep(int mSweep)
{
  m_mSweep = mSweep;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMSteer(int mSteer)
{
  m_mSteer = mSteer;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwActiveDisplay(int pwActiveDisplay)
{
  m_pwActiveDisplay = pwActiveDisplay;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwGatePos(int pwGatePos)
{
  m_pwGatePos = pwGatePos;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwGateDepth(int pwGateDepth)
{
  m_pwGateDepth = pwGateDepth;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwGateSize(int pwGateSize)
{
  m_pwGateSize = pwGateSize;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwGain(int pwGain)
{
  m_pwGain = pwGain;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwInvert(int pwInvert)
{
  m_pwInvert = pwInvert;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPWSteer(int pWSteer)
{
  m_pWSteer = pWSteer;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwSweep(int pwSweep)
{
  m_pwSweep = pwSweep;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwTxFreq(int pwTxFreq)
{
  m_pwTxFreq = pwTxFreq;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwBaseline(int pwBaseline)
{
  m_pwBaseline = pwBaseline;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPwPrp(int pwPrp)
{
  m_pwPrp = pwPrp;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorBox(int colorBox)
{
  m_colorBox = colorBox;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorSteer(int colorSteer)
{
  m_colorSteer = colorSteer;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorPersist(int colorPersist)
{
  m_colorPersist = colorPersist;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorGain(int colorGain)
{
  m_colorGain = colorGain;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorInvert(int colorInvert)
{
  m_colorInvert = colorInvert;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorTxFreq(int colorTxFreq)
{
  m_colorTxFreq = colorTxFreq;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorEnsemble(int colorEnsemble)
{
  m_colorEnsemble = colorEnsemble;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorMode(int colorMode)
{
  m_colorMode = colorMode;
}

/**
*
* @param
*/
void usAcquisitionParameters::setColorPrp(int colorPrp)
{
  m_colorPrp = colorPrp;
}

/**
*
* @param
*/
void usAcquisitionParameters::setTriplexActiveDisplay(int triplexActiveDisplay)
{
  m_triplexActiveDisplay = triplexActiveDisplay;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBSamplingFreq(int bSamplingFreq)
{
  m_bSamplingFreq = bSamplingFreq;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBLineDensity(int bLineDensity)
{
  m_bLineDensity = bLineDensity;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPowerPositive(int powerPositive)
{
  m_powerPositive = powerPositive;
}

/**
*
* @param
*/
void usAcquisitionParameters::setPowerNegative(int powerNegative)
{
  m_powerNegative = powerNegative;
}

/**
*
* @param
*/
void usAcquisitionParameters::setBSampleSize(int bSampleSize)
{
  m_bSampleSize = bSampleSize;
}

/**
*
* @param
*/
void usAcquisitionParameters::setRfMode(int rfMode)
{
  m_rfMode = rfMode;
}

/**
*
* @param
*/
void usAcquisitionParameters::setRfDecim(int rfDecim)
{
  m_rfDecim = rfDecim;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMotorFrames(int motorFrames)
{
  m_motorFrames = motorFrames;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMotorSteps(int motorSteps)
{
  m_motorSteps = motorSteps;
}

/**
*
* @param
*/
void usAcquisitionParameters::setMotorStatus(int motorStatus)
{
  m_motorStatus = motorStatus;
}
