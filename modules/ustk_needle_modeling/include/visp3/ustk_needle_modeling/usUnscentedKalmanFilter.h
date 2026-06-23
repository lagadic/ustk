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
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/


/**
 * @file usUnscentedKalmanFilter.h
 * @brief Generic abstract class for the implementation of an Unscented Kalman filter.
 */

#ifndef __usUnscentedKalmanFilter_h_
#define __usUnscentedKalmanFilter_h_

#include <visp3/core/vpConfig.h>

#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>


class VISP_EXPORT usUnscentedKalmanFilter
{
public:
  enum NoiseType : int { ADDITIVE_NOISE, GENERIC_NOISE };
  enum SigmaPointGenerationType : int { STANDARD_COVARIANCE, FIXED_SCALING_FACTOR, LIMITED_SPREAD };

protected:

  unsigned int m_stateDimension;
  unsigned int m_measureDimension;
  unsigned int m_processNoiseDimension;
  unsigned int m_measureNoiseDimension;

  VISP_NAMESPACE_ADDRESSING vpColVector m_state;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_stateCovarianceMatrix;

  NoiseType m_processNoiseType;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_processNoiseCovarianceMatrix;
  bool m_computeProcessNoiseCovarianceMatrixAutomatically;

  NoiseType m_measureNoiseType;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_measureNoiseCovarianceMatrix;
  bool m_computeMeasureNoiseCovarianceMatrixAutomatically;

  VISP_NAMESPACE_ADDRESSING vpColVector m_measure;

  SigmaPointGenerationType m_sigmaPointsGenerationType;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_sigmaPointsInit;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_sigmaPointsPropagated;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_sigmaPointsMeasure;
  VISP_NAMESPACE_ADDRESSING vpColVector m_sigmaPointsMeanWeights;
  VISP_NAMESPACE_ADDRESSING vpColVector m_sigmaPointsCovarianceWeights;
  double m_sigmaPointsScalingFactor;
  double m_sigmaPointsSpreadThreshold;

  VISP_NAMESPACE_ADDRESSING vpColVector m_stateSigmaMean;
  VISP_NAMESPACE_ADDRESSING vpColVector m_measureSigmaMean;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_stateSigmaCovarianceMatrix;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_stateMeasureSigmaCovarianceMatrix;
  VISP_NAMESPACE_ADDRESSING vpMatrix m_measureSigmaCovarianceMatrix;

public:
  usUnscentedKalmanFilter();
  virtual ~usUnscentedKalmanFilter();

  int getStateDimension() const;
  void setStateDimension(int dim);

  int getMeasureDimension() const;
  void setMeasureDimension(int dim);

  NoiseType getProcessNoiseType() const;
  void setProcessNoiseType(NoiseType type);
  int getProcessNoiseDimension() const;
  void setProcessNoiseDimension(int dim);

  NoiseType getMeasureNoiseType() const;
  void setMeasureNoiseType(NoiseType type);
  int getMeasureNoiseDimension() const;
  void setMeasureNoiseDimension(int dim);

  SigmaPointGenerationType getSigmaPointGenerationType() const;
  void setSigmaPointGenerationType(SigmaPointGenerationType type);
  double getSigmaPointScalingFactor() const;
  void setSigmaPointScalingFactor(double factor);
  double getSigmaPointSpreadThreshold() const;
  void setSigmaPointSpreadThreshold(double threshold);

  VISP_NAMESPACE_ADDRESSING vpColVector getState() const;
  void setState(const VISP_NAMESPACE_ADDRESSING vpColVector &state);

  VISP_NAMESPACE_ADDRESSING vpMatrix getStateCovarianceMatrix() const;
  void setStateCovarianceMatrix(const VISP_NAMESPACE_ADDRESSING vpMatrix &mat);

  VISP_NAMESPACE_ADDRESSING vpMatrix getProcessNoiseCovarianceMatrix() const;
  void setProcessNoiseCovarianceMatrix(const VISP_NAMESPACE_ADDRESSING vpMatrix &cov);
  bool computeProcessNoiseCovarianceMatrixAutomatically() const;
  void computeProcessNoiseCovarianceMatrixAutomatically(bool flag);

  VISP_NAMESPACE_ADDRESSING vpMatrix getMeasureNoiseCovarianceMatrix() const;
  void setMeasureNoiseCovarianceMatrix(const VISP_NAMESPACE_ADDRESSING vpMatrix &cov);
  bool computeMeasureNoiseCovarianceMatrixAutomatically() const;
  void computeMeasureNoiseCovarianceMatrixAutomatically(bool flag);

  virtual bool checkConsistency(const VISP_NAMESPACE_ADDRESSING vpColVector &measure);
  virtual void computeProcessNoiseCovarianceMatrix();
  virtual void computeMeasureNoiseCovarianceMatrix();
  bool generateSigmaPoints();
  bool computePropagatedSigmaPoints();
  bool computeSigmaMeasures();
  void computeMeansAndCovarianceMatricesFromSigmaPoints();
  bool updateState();

  bool filter(const VISP_NAMESPACE_ADDRESSING vpColVector &measure);

  virtual VISP_NAMESPACE_ADDRESSING vpColVector propagateSigmaPoint(const VISP_NAMESPACE_ADDRESSING vpColVector &sigmaPoint) = 0;
  virtual VISP_NAMESPACE_ADDRESSING vpColVector computeMeasureFromSigmaPoint(const VISP_NAMESPACE_ADDRESSING vpColVector &sigmaPoint) = 0;
  virtual double stateNorm(const VISP_NAMESPACE_ADDRESSING vpColVector &state) const;
  virtual VISP_NAMESPACE_ADDRESSING vpColVector measureLog(const VISP_NAMESPACE_ADDRESSING vpColVector &measure, const VISP_NAMESPACE_ADDRESSING vpColVector &measureCenter) const;
  virtual VISP_NAMESPACE_ADDRESSING vpColVector stateLog(const VISP_NAMESPACE_ADDRESSING vpColVector &state, const VISP_NAMESPACE_ADDRESSING vpColVector &stateCenter) const;
  virtual VISP_NAMESPACE_ADDRESSING vpColVector stateExp(const VISP_NAMESPACE_ADDRESSING vpColVector &state, const VISP_NAMESPACE_ADDRESSING vpColVector &stateCenter) const;
};

#endif // __usUnscentedKalmanFilter_h_
