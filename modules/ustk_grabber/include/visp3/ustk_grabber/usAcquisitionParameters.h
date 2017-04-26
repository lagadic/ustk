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
 * @brief Class to store all acquisition parameters for ultrasound acquisition.
 */

#ifndef __usAcquisitionParameters_h_
#define __usAcquisitionParameters_h_

#include <visp3/core/vpConfig.h>

/**
 * @class usAcquisitionParameters
 * @brief Class to store all acquisition parameters for ultrasound acquisition.
 * @ingroup module_ustk_grabber
 */
class VISP_EXPORT usAcquisitionParameters
{
public:

  usAcquisitionParameters();
  ~usAcquisitionParameters();

  //values
  int getAcousticPower() {return m_acousticPower;}
  int getBImageDepth() {return m_bImageDepth;}
  int getBBImageDepth () {return m_bBImageDepth;}
  int getBImageDepthA () {return m_bImageDepthA;}
  int getBImageDepthB() {return m_bImageDepthB;}
  int getZoom() {return m_zoom;}
  int getTGC() {return m_tGC;}
  int getBrightness() {return m_brightness;}
  int getContrast() {return m_contrast;}
  int getGamma() {return m_gamma;}
  int getBMap() {return m_bMap;}
  int getBSector() {return m_bSector;}
  int getBBSector() {return m_bBSector;}
  int getBSectorA() {return m_bSectorA;}
  int getBSectorB() {return m_bSectorB;}
  int getBPersist() {return m_bPersist;}
  int getBDynRange() {return m_bDynRange;}
  int getBSteer() {return m_bSteer;}
  int getBGain() {return m_bGain;}
  int getBTxFreq() {return m_bTxFreq;}
  int getBFocusDepth() {return m_bFocusDepth;}
  int getBFocusCount() {return m_bFocusCount;}
  int getBFocusSpacing() {return m_bFocusSpacing;}
  int getBImageOpt() {return m_bImageOpt;}
  int getBMru() {return m_bMru;}
  int getDualActiveDisplay() {return m_dualActiveDisplay;}
  int getQuadActiveDisplay() {return m_quadActiveDisplay;}
  int getMMode() {return m_mMode;}
  int getMZoom() {return m_mZoom;}
  int getMPos() {return m_mPos;}
  int getMDepth() {return m_mDepth;}
  int getMSweep() {return m_mSweep;}
  int getMSteer() {return m_mSteer;}
  int getPwActiveDisplay() {return m_pwActiveDisplay;}
  int getPwGatePos() {return m_pwGatePos;}
  int getPwGateDepth() {return m_pwGateDepth;}
  int getPwGateSize() {return m_pwGateSize;}
  int getPwGain() {return m_pwGain;}
  int getPwInvert() {return m_pwInvert;}
  int getPWSteer() {return m_pWSteer;}
  int getPwSweep() {return m_pwSweep;}
  int getPwTxFreq() {return m_pwTxFreq;}
  int getPwBaseline() {return m_pwBaseline;}
  int getPwPrp() {return m_pwPrp;}
  int getColorBox() {return m_colorBox;}
  int getColorSteer() {return m_colorSteer;}
  int getColorPersist() {return m_colorPersist;}
  int getColorGain() {return m_colorGain;}
  int getColorInvert() {return m_colorInvert;}
  int getColorTxFreq() {return m_colorTxFreq;}
  int getColorEnsemble() {return m_colorEnsemble;}
  int getColorMode() {return m_colorMode;}
  int getColorPrp() {return m_colorPrp;}
  int getTriplexActiveDisplay() {return m_triplexActiveDisplay;}
  int getBSamplingFreq() {return m_bSamplingFreq;}
  int getBLineDensity() {return m_bLineDensity;}
  int getPowerPositive() {return m_powerPositive;}
  int getPowerNegative() {return m_powerNegative;}
  int getBSampleSize() {return m_bSampleSize;}
  int getRfMode() {return m_rfMode;}
  int getRfDecim() {return m_rfDecim;}
  int getMotorFrames() {return m_motorFrames;}
  int getMotorSteps() {return m_motorSteps;}
  int getMotorStatus() {return m_motorStatus;}

  //min values
  int getAcousticPowerMin() {return m_acousticPowerMin;}
  int getBImageDepthMin() {return m_bImageDepthMin;}
  int getBBImageDepthMin() {return m_bBImageDepthMin;}
  int getBImageDepthAMin() {return m_bImageDepthAMin;}
  int getBImageDepthBMin() {return m_bImageDepthBMin;}
  int getZoomMin() {return m_zoomMin;}
  int getTGCMin() {return m_tGCMin;}
  int getBrightnessMin() {return m_brightnessMin;}
  int getContrastMin() {return m_contrastMin;}
  int getGammaMin() {return m_gammaMin;}
  int getBMapMin() {return m_bMapMin;}
  int getBSectorMin() {return m_bSectorMin;}
  int getBBSectorMin() {return m_bBSectorMin;}
  int getBSectorAMin() {return m_bSectorAMin;}
  int getBSectorBMin() {return m_bSectorBMin;}
  int getBPersistMin() {return m_bPersistMin;}
  int getBDynRangeMin() {return m_bDynRangeMin;}
  int getBSteerMin() {return m_bSteerMin;}
  int getBGainMin() {return m_bGainMin;}
  int getBTxFreqMin() {return m_bTxFreqMin;}
  int getBFocusDepthMin() {return m_bFocusDepthMin;}
  int getBFocusCountMin() {return m_bFocusCountMin;}
  int getBFocusSpacingMin() {return m_bFocusSpacingMin;}
  int getBImageOptMin() {return m_bImageOptMin;}
  int getBMruMin() {return m_bMruMin;}
  int getDualActiveDisplayMin() {return m_dualActiveDisplayMin;}
  int getQuadActiveDisplayMin() {return m_quadActiveDisplayMin;}
  int getMModeMin() {return m_mModeMin;}
  int getMZoomMin() {return m_mZoomMin;}
  int getMPosMin() {return m_mPosMin;}
  int getMDepthMin() {return m_mDepthMin;}
  int getMSweepMin() {return m_mSweepMin;}
  int getMSteerMin() {return m_mSteerMin;}
  int getPwActiveDisplayMin() {return m_pwActiveDisplayMin;}
  int getPwGatePosMin() {return m_pwGatePosMin;}
  int getPwGateDepthMin() {return m_pwGateDepthMin;}
  int getPwGateSizeMin() {return m_pwGateSizeMin;}
  int getPwGainMin() {return m_pwGainMin;}
  int getPwInvertMin() {return m_pwInvertMin;}
  int getPWSteerMin() {return m_pWSteerMin;}
  int getPwSweepMin() {return m_pwSweepMin;}
  int getPwTxFreqMin() {return m_pwTxFreqMin;}
  int getPwBaselineMin() {return m_pwBaselineMin;}
  int getPwPrpMin() {return m_pwPrpMin;}
  int getColorBoxMin() {return m_colorBoxMin;}
  int getColorSteerMin() {return m_colorSteerMin;}
  int getColorPersistMin() {return m_colorPersistMin;}
  int getColorGainMin() {return m_colorGainMin;}
  int getColorInvertMin() {return m_colorInvertMin;}
  int getColorTxFreqMin() {return m_colorTxFreqMin;}
  int getColorEnsembleMin() {return m_colorEnsembleMin;}
  int getColorModeMin() {return m_colorModeMin;}
  int getColorPrpMin() {return m_colorPrpMin;}
  int getTriplexActiveDisplayMin() {return m_triplexActiveDisplayMin;}
  int getBSamplingFreqMin() {return m_bSamplingFreqMin;}
  int getBLineDensityMin() {return m_bLineDensityMin;}
  int getPowerPositiveMin() {return m_powerPositiveMin;}
  int getPowerNegativeMin() {return m_powerNegativeMin;}
  int getBSampleSizeMin() {return m_bSampleSizeMin;}
  int getRfModeMin() {return m_rfModeMin;}
  int getRfDecimMin() {return m_rfDecimMin;}
  int getMotorFramesMin() {return m_motorFramesMin;}
  int getMotorStepsMin() {return m_motorStepsMin;}
  int getMotorStatusMin() {return m_motorStatusMin;}

  //max values
  int getAcousticPowerMax() {return m_acousticPowerMax;}
  int getBImageDepthMax() {return m_bImageDepthMax;}
  int getBBImageDepthMax() {return m_bBImageDepthMax;}
  int getBImageDepthAMax() {return m_bImageDepthAMax;}
  int getBImageDepthBMax() {return m_bImageDepthBMax;}
  int getZoomMax() {return m_zoomMax;}
  int getTGCMax() {return m_tGCMax;}
  int getBrightnessMax() {return m_brightnessMax;}
  int getContrastMax() {return m_contrastMax;}
  int getGammaMax() {return m_gammaMax;}
  int getBMapMax() {return m_bMapMax;}
  int getBSectorMax() {return m_bSectorMax;}
  int getBBSectorMax() {return m_bBSectorMax;}
  int getBSectorAMax() {return m_bSectorAMax;}
  int getBSectorBMax() {return m_bSectorBMax;}
  int getBPersistMax() {return m_bPersistMax;}
  int getBDynRangeMax() {return m_bDynRangeMax;}
  int getBSteerMax() {return m_bSteerMax;}
  int getBGainMax() {return m_bGainMax;}
  int getBTxFreqMax() {return m_bTxFreqMax;}
  int getBFocusDepthMax() {return m_bFocusDepthMax;}
  int getBFocusCountMax() {return m_bFocusCountMax;}
  int getBFocusSpacingMax() {return m_bFocusSpacingMax;}
  int getBImageOptMax() {return m_bImageOptMax;}
  int getBMruMax() {return m_bMruMax;}
  int getDualActiveDisplayMax() {return m_dualActiveDisplayMax;}
  int getQuadActiveDisplayMax() {return m_quadActiveDisplayMax;}
  int getMModeMax() {return m_mModeMax;}
  int getMZoomMax() {return m_mZoomMax;}
  int getMPosMax() {return m_mPosMax;}
  int getMDepthMax() {return m_mDepthMax;}
  int getMSweepMax() {return m_mSweepMax;}
  int getMSteerMax() {return m_mSteerMax;}
  int getPwActiveDisplayMax() {return m_pwActiveDisplayMax;}
  int getPwGatePosMax() {return m_pwGatePosMax;}
  int getPwGateDepthMax() {return m_pwGateDepthMax;}
  int getPwGateSizeMax() {return m_pwGateSizeMax;}
  int getPwGainMax() {return m_pwGainMax;}
  int getPwInvertMax() {return m_pwInvertMax;}
  int getPWSteerMax() {return m_pWSteerMax;}
  int getPwSweepMax() {return m_pwSweepMax;}
  int getPwTxFreqMax() {return m_pwTxFreqMax;}
  int getPwBaselineMax() {return m_pwBaselineMax;}
  int getPwPrpMax() {return m_pwPrpMax;}
  int getColorBoxMax() {return m_colorBoxMax;}
  int getColorSteerMax() {return m_colorSteerMax;}
  int getColorPersistMax() {return m_colorPersistMax;}
  int getColorGainMax() {return m_colorGainMax;}
  int getColorInvertMax() {return m_colorInvertMax;}
  int getColorTxFreqMax() {return m_colorTxFreqMax;}
  int getColorEnsembleMax() {return m_colorEnsembleMax;}
  int getColorModeMax() {return m_colorModeMax;}
  int getColorPrpMax() {return m_colorPrpMax;}
  int getTriplexActiveDisplayMax() {return m_triplexActiveDisplayMax;}
  int getBSamplingFreqMax() {return m_bSamplingFreqMax;}
  int getBLineDensityMax() {return m_bLineDensityMax;}
  int getPowerPositiveMax() {return m_powerPositiveMax;}
  int getPowerNegativeMax() {return m_powerNegativeMax;}
  int getBSampleSizeMax() {return m_bSampleSizeMax;}
  int getRfModeMax() {return m_rfModeMax;}
  int getRfDecimMax() {return m_rfDecimMax;}
  int getMotorFramesMax() {return m_motorFramesMax;}
  int getMotorStepsMax() {return m_motorStepsMax;}
  int getMotorStatusMax() {return m_motorStatusMax;}

  // setters
  void setAcousticPower(int acousticPower);
  void setBImageDepth(int bImageDepth);
  void setBBImageDepth (int bBImageDepth);
  void setBImageDepthA (int bImageDepthA);
  void setBImageDepthB(int bImageDepthB);
  void setZoom(int zoom);
  void setTGC(int TGC);
  void setBrightness(int brightness);
  void setContrast(int contrast);
  void setGamma(int gamma);
  void setBMap(int bMap);
  void setBSector(int bSector);
  void setBBSector(int bBSector);
  void setBSectorA(int bSectorA);
  void setBSectorB(int bSectorB);
  void setBPersist(int bPersist);
  void setBDynRange(int bDynRange);
  void setBSteer(int bSteer);
  void setBGain(int bGain);
  void setBTxFreq(int bTxFreq);
  void setBFocusDepth(int bFocusDepth);
  void setBFocusCount(int bFocusCount);
  void setBFocusSpacing(int bFocusSpacing);
  void setBImageOpt(int bImageOpt);
  void setBMru(int bMru);
  void setDualActiveDisplay(int dualActiveDisplay);
  void setQuadActiveDisplay(int quadActiveDisplay);
  void setMMode(int mMode);
  void setMZoom(int mZoom);
  void setMPos(int mPos);
  void setMDepth(int mDepth);
  void setMSweep(int mSweep);
  void setMSteer(int mSteer);
  void setPwActiveDisplay(int pwActiveDisplay);
  void setPwGatePos(int pwGatePos);
  void setPwGateDepth(int pwGateDepth);
  void setPwGateSize(int pwGateSize);
  void setPwGain(int pwGain);
  void setPwInvert(int pwInvert);
  void setPWSteer(int pWSteer);
  void setPwSweep(int pwSweep);
  void setPwTxFreq(int pwTxFreq);
  void setPwBaseline(int pwBaseline);
  void setPwPrp(int pwPrp);
  void setColorBox(int colorBox);
  void setColorSteer(int colorSteer);
  void setColorPersist(int colorPersist);
  void setColorGain(int colorGain);
  void setColorInvert(int colorInvert);
  void setColorTxFreq(int colorTxFreq);
  void setColorEnsemble(int colorEnsemble);
  void setColorMode(int colorMode);
  void setColorPrp(int colorPrp);
  void setTriplexActiveDisplay(int triplexActiveDisplay);
  void setBSamplingFreq(int bSamplingFreq);
  void setBLineDensity(int bLineDensity);
  void setPowerPositive(int powerPositive);
  void setPowerNegative(int powerNegative);
  void setBSampleSize(int bSampleSize);
  void setRfMode(int rfMode);
  void setRfDecim(int rfDecim);
  void setMotorFrames(int motorFrames);
  void setMotorSteps(int motorSteps);
  void setMotorStatus(int motorStatus);

private :
  // values
  int m_acousticPower;
  int m_bImageDepth;
  int m_bBImageDepth ;
  int m_bImageDepthA ;
  int m_bImageDepthB;
  int m_zoom;
  int m_tGC;
  int m_brightness;
  int m_contrast;
  int m_gamma;
  int m_bMap;
  int m_bSector;
  int m_bBSector;
  int m_bSectorA;
  int m_bSectorB;
  int m_bPersist;
  int m_bDynRange;
  int m_bSteer;
  int m_bGain;
  int m_bTxFreq;
  int m_bFocusDepth;
  int m_bFocusCount;
  int m_bFocusSpacing;
  int m_bImageOpt;
  int m_bMru;
  int m_dualActiveDisplay;
  int m_quadActiveDisplay;
  int m_mMode;
  int m_mZoom;
  int m_mPos;
  int m_mDepth;
  int m_mSweep;
  int m_mSteer;
  int m_pwActiveDisplay;
  int m_pwGatePos;
  int m_pwGateDepth;
  int m_pwGateSize;
  int m_pwGain;
  int m_pwInvert;
  int m_pWSteer;
  int m_pwSweep;
  int m_pwTxFreq;
  int m_pwBaseline;
  int m_pwPrp;
  int m_colorBox;
  int m_colorSteer;
  int m_colorPersist;
  int m_colorGain;
  int m_colorInvert;
  int m_colorTxFreq;
  int m_colorEnsemble;
  int m_colorMode;
  int m_colorPrp;
  int m_triplexActiveDisplay;
  int m_bSamplingFreq;
  int m_bLineDensity;
  int m_powerPositive;
  int m_powerNegative;
  int m_bSampleSize;
  int m_rfMode;
  int m_rfDecim;
  int m_motorFrames;
  int m_motorSteps;
  int m_motorStatus;

  // min values
  int m_acousticPowerMin;
  int m_bImageDepthMin;
  int m_bBImageDepthMin;
  int m_bImageDepthAMin;
  int m_bImageDepthBMin;
  int m_zoomMin;
  int m_tGCMin;
  int m_brightnessMin;
  int m_contrastMin;
  int m_gammaMin;
  int m_bMapMin;
  int m_bSectorMin;
  int m_bBSectorMin;
  int m_bSectorAMin;
  int m_bSectorBMin;
  int m_bPersistMin;
  int m_bDynRangeMin;
  int m_bSteerMin;
  int m_bGainMin;
  int m_bTxFreqMin;
  int m_bFocusDepthMin;
  int m_bFocusCountMin;
  int m_bFocusSpacingMin;
  int m_bImageOptMin;
  int m_bMruMin;
  int m_dualActiveDisplayMin;
  int m_quadActiveDisplayMin;
  int m_mModeMin;
  int m_mZoomMin;
  int m_mPosMin;
  int m_mDepthMin;
  int m_mSweepMin;
  int m_mSteerMin;
  int m_pwActiveDisplayMin;
  int m_pwGatePosMin;
  int m_pwGateDepthMin;
  int m_pwGateSizeMin;
  int m_pwGainMin;
  int m_pwInvertMin;
  int m_pWSteerMin;
  int m_pwSweepMin;
  int m_pwTxFreqMin;
  int m_pwBaselineMin;
  int m_pwPrpMin;
  int m_colorBoxMin;
  int m_colorSteerMin;
  int m_colorPersistMin;
  int m_colorGainMin;
  int m_colorInvertMin;
  int m_colorTxFreqMin;
  int m_colorEnsembleMin;
  int m_colorModeMin;
  int m_colorPrpMin;
  int m_triplexActiveDisplayMin;
  int m_bSamplingFreqMin;
  int m_bLineDensityMin;
  int m_powerPositiveMin;
  int m_powerNegativeMin;
  int m_bSampleSizeMin;
  int m_rfModeMin;
  int m_rfDecimMin;
  int m_motorFramesMin;
  int m_motorStepsMin;
  int m_motorStatusMin;

  // max values
  int m_acousticPowerMax;
  int m_bImageDepthMax;
  int m_bBImageDepthMax;
  int m_bImageDepthAMax;
  int m_bImageDepthBMax;
  int m_zoomMax;
  int m_tGCMax;
  int m_brightnessMax;
  int m_contrastMax;
  int m_gammaMax;
  int m_bMapMax;
  int m_bSectorMax;
  int m_bBSectorMax;
  int m_bSectorAMax;
  int m_bSectorBMax;
  int m_bPersistMax;
  int m_bDynRangeMax;
  int m_bSteerMax;
  int m_bGainMax;
  int m_bTxFreqMax;
  int m_bFocusDepthMax;
  int m_bFocusCountMax;
  int m_bFocusSpacingMax;
  int m_bImageOptMax;
  int m_bMruMax;
  int m_dualActiveDisplayMax;
  int m_quadActiveDisplayMax;
  int m_mModeMax;
  int m_mZoomMax;
  int m_mPosMax;
  int m_mDepthMax;
  int m_mSweepMax;
  int m_mSteerMax;
  int m_pwActiveDisplayMax;
  int m_pwGatePosMax;
  int m_pwGateDepthMax;
  int m_pwGateSizeMax;
  int m_pwGainMax;
  int m_pwInvertMax;
  int m_pWSteerMax;
  int m_pwSweepMax;
  int m_pwTxFreqMax;
  int m_pwBaselineMax;
  int m_pwPrpMax;
  int m_colorBoxMax;
  int m_colorSteerMax;
  int m_colorPersistMax;
  int m_colorGainMax;
  int m_colorInvertMax;
  int m_colorTxFreqMax;
  int m_colorEnsembleMax;
  int m_colorModeMax;
  int m_colorPrpMax;
  int m_triplexActiveDisplayMax;
  int m_bSamplingFreqMax;
  int m_bLineDensityMax;
  int m_powerPositiveMax;
  int m_powerNegativeMax;
  int m_bSampleSizeMax;
  int m_rfModeMax;
  int m_rfDecimMax;
  int m_motorFramesMax;
  int m_motorStepsMax;
  int m_motorStatusMax;
};
#endif // __usAcquisitionParameters_h_
