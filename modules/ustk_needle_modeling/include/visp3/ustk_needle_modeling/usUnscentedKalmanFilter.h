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
    enum NoiseType : int {ADDITIVE_NOISE, GENERIC_NOISE};
    enum SigmaPointGenerationType : int {STANDARD_COVARIANCE, FIXED_SCALING_FACTOR, LIMITED_SPREAD};
    
protected:
    
    unsigned int m_stateDimension;
    unsigned int m_measureDimension;
    unsigned int m_processNoiseDimension;
    unsigned int m_measureNoiseDimension;

    vpColVector m_state;
    vpMatrix m_stateCovarianceMatrix;
    
    NoiseType m_processNoiseType;
    vpMatrix m_processNoiseCovarianceMatrix;
    bool m_computeProcessNoiseCovarianceMatrixAutomatically;
    
    NoiseType m_measureNoiseType;
    vpMatrix m_measureNoiseCovarianceMatrix;
    bool m_computeMeasureNoiseCovarianceMatrixAutomatically;
    
    vpColVector m_measure;
    
    SigmaPointGenerationType m_sigmaPointsGenerationType;
    vpMatrix m_sigmaPointsInit;
    vpMatrix m_sigmaPointsPropagated;
    vpMatrix m_sigmaPointsMeasure;
    vpColVector m_sigmaPointsMeanWeights;
    vpColVector m_sigmaPointsCovarianceWeights;
    double m_sigmaPointsScalingFactor;
    double m_sigmaPointsSpreadThreshold;
    
    vpColVector m_stateSigmaMean;
    vpColVector m_measureSigmaMean;
    vpMatrix m_stateSigmaCovarianceMatrix;
    vpMatrix m_stateMeasureSigmaCovarianceMatrix;
    vpMatrix m_measureSigmaCovarianceMatrix;

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
    
    vpColVector getState() const;
    void setState(const vpColVector &state);
    
    vpMatrix getStateCovarianceMatrix() const;
    void setStateCovarianceMatrix(const vpMatrix &mat);
    
    vpMatrix getProcessNoiseCovarianceMatrix() const;
    void setProcessNoiseCovarianceMatrix(const vpMatrix &cov);
    bool computeProcessNoiseCovarianceMatrixAutomatically() const;
    void computeProcessNoiseCovarianceMatrixAutomatically(bool flag);
    
    vpMatrix getMeasureNoiseCovarianceMatrix() const;
    void setMeasureNoiseCovarianceMatrix(const vpMatrix &cov);
    bool computeMeasureNoiseCovarianceMatrixAutomatically() const;
    void computeMeasureNoiseCovarianceMatrixAutomatically(bool flag);
    
    virtual bool checkConsistency(const vpColVector &measure);
    virtual void computeProcessNoiseCovarianceMatrix();
    virtual void computeMeasureNoiseCovarianceMatrix();
    bool generateSigmaPoints();
    bool computePropagatedSigmaPoints();
    bool computeSigmaMeasures();
    void computeMeansAndCovarianceMatricesFromSigmaPoints();
    bool updateState();
    
    bool filter(const vpColVector &measure);
    
    virtual vpColVector propagateSigmaPoint(const vpColVector &sigmaPoint)=0;
    virtual vpColVector computeMeasureFromSigmaPoint(const vpColVector &sigmaPoint)=0;
    virtual double stateNorm(const vpColVector &state) const;
    virtual vpColVector measureLog(const vpColVector &measure, const vpColVector &measureCenter) const;
    virtual vpColVector stateLog(const vpColVector &state, const vpColVector &stateCenter) const;
    virtual vpColVector stateExp(const vpColVector &state, const vpColVector &stateCenter) const;
};

#endif // __usUnscentedKalmanFilter_h_
