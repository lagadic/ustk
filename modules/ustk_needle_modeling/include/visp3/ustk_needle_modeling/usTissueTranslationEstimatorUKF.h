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
 * @file usTissueTranslationEstimatorUKF.h
 * @brief Unscented Kalman filter to estimate the position of the tissue representation of a usNeedleInsertionModelRayleighRitzSpline needle insertion model.
 */

#ifndef __usTissueTranslationEstimatorUKF_h_
#define __usTissueTranslationEstimatorUKF_h_

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelRayleighRitzSpline.h>
#include <visp3/ustk_needle_modeling/usUnscentedKalmanFilter.h>


class VISP_EXPORT usTissueTranslationEstimatorUKF : public usUnscentedKalmanFilter
{
public:
    
    enum StateDynamicsType : int {CONSTANT_POSITION, CONSTANT_VELOCITY};
    enum TissueTranslationType : int {LATERAL_TRANSLATIONS_ONLY, FULL_TRANSLATIONS};
    enum MeasureType : int {NEEDLE_BODY_POINTS, TIP_POSITION_AND_DIRECTION, BASE_FORCE_TORQUE};
    
    double m_var_measure_p;
    double m_var_measure_d;
    double m_var_measure_f;
    double m_var_measure_t;
    double m_var_process_p;
    double m_var_process_v;
    StateDynamicsType m_stateDynamicsType;
    TissueTranslationType m_tissueTranslationType;
    MeasureType m_measureType;
    
    double m_propagationTime;

    usNeedleInsertionModelRayleighRitzSpline m_needle;

public:
    usTissueTranslationEstimatorUKF();
    ~usTissueTranslationEstimatorUKF();
    
    double getPositionMeasureNoiseVariance() const;
    void setPositionMeasureNoiseVariance(double sigma);
    
    double getTipDirectionMeasureNoiseVariance() const;
    void setTipDirectionMeasureNoiseVariance(double sigma);
            
    double getForceMeasureNoiseVariance() const;
    void setForceMeasureNoiseVariance(double sigma);
    
    double getTorqueMeasureNoiseVariance() const;
    void setTorqueMeasureNoiseVariance(double sigma);
    
    double getTissuePositionProcessNoiseVariance() const;
    void setTissuePositionProcessNoiseVariance(double sigma);
    
    double getTissueVelocityProcessNoiseVariance() const;
    void setTissueVelocityProcessNoiseVariance(double sigma);
    
    StateDynamicsType getStateDynamicsType() const;
    void setStateDynamicsType(StateDynamicsType type);
    
    TissueTranslationType getTissueTranslationType() const;
    void setTissueTranslationType(TissueTranslationType type);
    
    MeasureType getMeasureType() const;
    void setMeasureType(MeasureType type);
    
    void setPropagationTime(double time);
    
    void setCurrentNeedle(const usNeedleInsertionModelRayleighRitzSpline& needle);
    void applyStateToNeedle(usNeedleInsertionModelRayleighRitzSpline& needle) const;
    
    bool checkConsistency(const vpColVector &measure);
    void computeProcessNoiseCovarianceMatrix();
    void computeMeasureNoiseCovarianceMatrix();
    vpColVector propagateSigmaPoint(const vpColVector &sigmaPoint);
    vpColVector computeMeasureFromSigmaPoint(const vpColVector &sigmaPoint);
    double stateNorm(const vpColVector& state) const;
    vpColVector measureLog(const vpColVector& measure, const vpColVector &measureCenter) const;
};

#endif // __usTissueTranslationEstimatorUKF_h_
