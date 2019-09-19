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

#include <visp3/ustk_needle_modeling/usUnscentedKalmanFilter.h>

#include <iostream>

#if defined(VISP_HAVE_EIGEN3)
#include <eigen3/Eigen/Cholesky>
#endif


vpMatrix root(const vpMatrix &M)
{
    if(M.getCols() != M.getRows()) throw vpException(vpException::dimensionError, "NeedleUpdate::root(const vpMatrix&): input matrix should be square");

    vpMatrix sqrtM(M.getRows(), M.getCols());
    
#if defined(VISP_HAVE_EIGEN3)
    Eigen::MatrixXd Meigen(M.getRows(),M.getCols());

    for(unsigned int i =0 ; i<M.getRows() ; i++)
    {
        for(unsigned int j=0 ; j<M.getCols() ; j++)
        {
            Meigen(i,j) = M[i][j];
        }
    }

    Eigen::MatrixXd sqrtMeigen(Meigen.llt().matrixL());

    for(unsigned int i =0 ; i<M.getRows() ; i++)
    {
        for(unsigned int j=0 ; j<M.getCols() ; j++)
        {
            sqrtM[i][j] = sqrtMeigen(i,j);
        }
    }
#else
      vpMatrix A(M);
      vpColVector w;
      vpMatrix V;
      try
      {
          A.svd(w,V);
      }
      catch(std::exception &e)
      {
        vpMatrix I;
        I.eye(M.getCols());
        A = M + std::numeric_limits<double>::epsilon() * I;
        A.svd(w,V);
      }
      for(unsigned int i=0 ; i<w.size() ; i++)
      {
          if(w[i] > 1e-8 * w[0]) sqrtM.insert(sqrt(w[i])*A.getCol(i), 0,i);
      }
#endif
    return sqrtM;
}

usUnscentedKalmanFilter::usUnscentedKalmanFilter():
    m_stateDimension(0),
    m_measureDimension(0),
    m_processNoiseDimension(0),
    m_measureNoiseDimension(0),

    m_state(),
    m_stateCovarianceMatrix(),
    
    m_processNoiseType(usUnscentedKalmanFilter::NoiseType::ADDITIVE_NOISE),
    m_processNoiseCovarianceMatrix(),
    m_computeProcessNoiseCovarianceMatrixAutomatically(false),
    
    m_measureNoiseType(usUnscentedKalmanFilter::NoiseType::ADDITIVE_NOISE),
    m_measureNoiseCovarianceMatrix(),
    m_computeMeasureNoiseCovarianceMatrixAutomatically(false),
    
    m_measure(),
    
    m_sigmaPointsGenerationType(usUnscentedKalmanFilter::STANDARD_COVARIANCE),
    m_sigmaPointsInit(),
    m_sigmaPointsPropagated(),
    m_sigmaPointsMeasure(),
    m_sigmaPointsMeanWeights(),
    m_sigmaPointsCovarianceWeights(),
    m_sigmaPointsScalingFactor(1),
    m_sigmaPointsSpreadThreshold(std::numeric_limits<double>::max()),
    
    m_stateSigmaMean(),
    m_measureSigmaMean(),
    m_stateSigmaCovarianceMatrix(),
    m_stateMeasureSigmaCovarianceMatrix(),
    m_measureSigmaCovarianceMatrix()
{
    
}

usUnscentedKalmanFilter::~usUnscentedKalmanFilter()
{
    
}

int usUnscentedKalmanFilter::getStateDimension() const
{
    return m_stateDimension;
}

void usUnscentedKalmanFilter::setStateDimension(int dim)
{
    if(dim <= 0) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::setStateDimension: state dimension should be positive");
    
    m_stateDimension = dim;
    m_state.resize(dim);
    m_stateCovarianceMatrix.resize(dim,dim);
    m_stateSigmaMean.resize(dim);
    m_stateSigmaCovarianceMatrix.resize(dim,dim);
    m_stateMeasureSigmaCovarianceMatrix.resize(m_stateDimension, m_measureDimension);
}

int usUnscentedKalmanFilter::getMeasureDimension() const
{
    return m_measureDimension;
}

void usUnscentedKalmanFilter::setMeasureDimension(int dim)
{
    if(dim <= 0) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::setMeasureDimension: measure dimension should be positive");
    
    m_measureDimension = dim;
    m_measureSigmaMean.resize(dim);
    m_measureSigmaCovarianceMatrix.resize(dim,dim);
    m_stateMeasureSigmaCovarianceMatrix.resize(m_stateDimension, m_measureDimension);
}

usUnscentedKalmanFilter::NoiseType usUnscentedKalmanFilter::getProcessNoiseType() const
{
    return m_processNoiseType;
}

void usUnscentedKalmanFilter::setProcessNoiseType(NoiseType type)
{
    m_processNoiseType = type;
}

int usUnscentedKalmanFilter::getProcessNoiseDimension() const
{
    return m_processNoiseDimension;
}

void usUnscentedKalmanFilter::setProcessNoiseDimension(int dim)
{
    if(dim <= 0) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::setProcessNoiseDimension: noise dimension should be positive");
    
    m_processNoiseDimension = dim;
    m_processNoiseCovarianceMatrix.resize(dim,dim);
}

usUnscentedKalmanFilter::NoiseType usUnscentedKalmanFilter::getMeasureNoiseType() const
{
    return m_measureNoiseType;
}

void usUnscentedKalmanFilter::setMeasureNoiseType(NoiseType type)
{
    m_measureNoiseType = type;
}

int usUnscentedKalmanFilter::getMeasureNoiseDimension() const
{
    return m_measureNoiseDimension;
}

void usUnscentedKalmanFilter::setMeasureNoiseDimension(int dim)
{
    if(dim <= 0) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::setMeasureNoiseDimension: noise dimension should be positive");
    
    m_measureNoiseDimension = dim;
    m_measureNoiseCovarianceMatrix.resize(dim,dim);
}

usUnscentedKalmanFilter::SigmaPointGenerationType usUnscentedKalmanFilter::getSigmaPointGenerationType() const
{
    return m_sigmaPointsGenerationType;
}

void usUnscentedKalmanFilter::setSigmaPointGenerationType(usUnscentedKalmanFilter::SigmaPointGenerationType type)
{
    m_sigmaPointsGenerationType = type;
}

double usUnscentedKalmanFilter::getSigmaPointScalingFactor() const
{
    return m_sigmaPointsScalingFactor;
}

void usUnscentedKalmanFilter::setSigmaPointScalingFactor(double factor)
{
    m_sigmaPointsScalingFactor = factor;
}

double usUnscentedKalmanFilter::getSigmaPointSpreadThreshold() const
{
    return m_sigmaPointsSpreadThreshold;
}

void usUnscentedKalmanFilter::setSigmaPointSpreadThreshold(double threshold)
{
    m_sigmaPointsSpreadThreshold = threshold;
}


vpColVector usUnscentedKalmanFilter::getState() const
{
    return m_state;
}

void usUnscentedKalmanFilter::setState(const vpColVector &state)
{
    if(state.getRows() != m_stateDimension) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::setState: invalid state dimension");
    
    m_state = state;
}

vpMatrix usUnscentedKalmanFilter::getStateCovarianceMatrix() const
{
    return m_stateCovarianceMatrix;
}

void usUnscentedKalmanFilter::setStateCovarianceMatrix(const vpMatrix &mat)
{
    if(mat.getRows() != m_stateDimension || mat.getCols() != m_stateDimension) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::setStateCovarianceMatrix: invalid matrix dimension");
    
    m_stateCovarianceMatrix = mat;
}

vpMatrix usUnscentedKalmanFilter::getProcessNoiseCovarianceMatrix() const
{
    return m_processNoiseCovarianceMatrix;
}

void usUnscentedKalmanFilter::setProcessNoiseCovarianceMatrix(const vpMatrix &cov)
{
    if(cov.getCols() != m_processNoiseDimension || cov.getRows() != m_processNoiseDimension) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::setProcessNoiseCovarianceMatrix: invalid matrix dimension");
    
    m_processNoiseCovarianceMatrix = cov;
}

bool usUnscentedKalmanFilter::computeProcessNoiseCovarianceMatrixAutomatically() const
{
    return m_computeProcessNoiseCovarianceMatrixAutomatically;
}

void usUnscentedKalmanFilter::computeProcessNoiseCovarianceMatrixAutomatically(bool flag)
{
    m_computeProcessNoiseCovarianceMatrixAutomatically = flag;
}

vpMatrix usUnscentedKalmanFilter::getMeasureNoiseCovarianceMatrix() const
{
    return m_measureNoiseCovarianceMatrix;
}

void usUnscentedKalmanFilter::setMeasureNoiseCovarianceMatrix(const vpMatrix &cov)
{
    if(cov.getCols() != m_measureNoiseDimension || cov.getRows() != m_measureNoiseDimension) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::setMeasureNoiseCovarianceMatrix: invalid matrix dimension");
    
    m_measureNoiseCovarianceMatrix = cov;
}

bool usUnscentedKalmanFilter::computeMeasureNoiseCovarianceMatrixAutomatically() const
{
    return m_computeMeasureNoiseCovarianceMatrixAutomatically;
}

void usUnscentedKalmanFilter::computeMeasureNoiseCovarianceMatrixAutomatically(bool flag)
{
    m_computeMeasureNoiseCovarianceMatrixAutomatically = flag;
}

bool usUnscentedKalmanFilter::checkConsistency(const vpColVector &measure)
{
    if(!m_computeProcessNoiseCovarianceMatrixAutomatically && (m_processNoiseType == usUnscentedKalmanFilter::ADDITIVE_NOISE) && (m_stateDimension != m_processNoiseDimension)) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::checkConsistency: additive process noise is set, but the noise dimension (%d) does not match the state dimension (%d)", m_processNoiseDimension, m_stateDimension);
    if(!m_computeMeasureNoiseCovarianceMatrixAutomatically && (m_measureNoiseType == usUnscentedKalmanFilter::ADDITIVE_NOISE) && (m_measureDimension != m_measureNoiseDimension)) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::checkConsistency: additive measure noise is set, but the noise dimension (%d) does not match the measure dimension (%d)", m_measureNoiseDimension, m_measureDimension);
    if(measure.size() != m_measureDimension) throw vpException(vpException::dimensionError, "usUnscentedKalmanFilter::checkConsistency: invamid input measure dimension (%d), should be %d", measure.size(), m_measureDimension);
    
    return true;
}

void usUnscentedKalmanFilter::computeProcessNoiseCovarianceMatrix()
{
    
}

void usUnscentedKalmanFilter::computeMeasureNoiseCovarianceMatrix()
{
    
}

bool usUnscentedKalmanFilter::generateSigmaPoints()
{
    int augmentedStateDimension = 0;
    vpMatrix augmentedStateCovarianceMatrix;
    
    switch(m_processNoiseType)
    {
        case NoiseType::ADDITIVE_NOISE:
        {
            switch(m_measureNoiseType)
            {
                case NoiseType::ADDITIVE_NOISE:
                {   
                    augmentedStateDimension = m_stateDimension;
                    augmentedStateCovarianceMatrix = m_stateCovarianceMatrix;
                    break;
                }
                case NoiseType::GENERIC_NOISE:
                {
                    augmentedStateDimension = m_stateDimension + m_measureNoiseDimension;
                    augmentedStateCovarianceMatrix.resize(augmentedStateDimension,augmentedStateDimension);
                    augmentedStateCovarianceMatrix.insert(m_stateCovarianceMatrix, 0,0);
                    augmentedStateCovarianceMatrix.insert(m_measureNoiseCovarianceMatrix, m_stateDimension,m_stateDimension);
                    break;
                }
            }
            break;
        }
        case NoiseType::GENERIC_NOISE:
        {
            switch(m_measureNoiseType)
            {
                case NoiseType::ADDITIVE_NOISE:
                {   
                    augmentedStateDimension = m_stateDimension + m_processNoiseDimension;
                    augmentedStateCovarianceMatrix.resize(augmentedStateDimension,augmentedStateDimension);
                    augmentedStateCovarianceMatrix.insert(m_stateCovarianceMatrix, 0,0);
                    augmentedStateCovarianceMatrix.insert(m_processNoiseCovarianceMatrix, m_stateDimension,m_stateDimension);
                    break;
                }
                case NoiseType::GENERIC_NOISE:
                {
                    augmentedStateDimension = m_stateDimension + m_processNoiseDimension + m_measureNoiseDimension;
                    augmentedStateCovarianceMatrix.resize(augmentedStateDimension,augmentedStateDimension);
                    augmentedStateCovarianceMatrix.insert(m_stateCovarianceMatrix, 0,0);
                    augmentedStateCovarianceMatrix.insert(m_processNoiseCovarianceMatrix, m_stateDimension,m_stateDimension);
                    augmentedStateCovarianceMatrix.insert(m_measureNoiseCovarianceMatrix, m_stateDimension + m_processNoiseDimension,m_stateDimension + m_processNoiseDimension);
                    break;
                }
            }
            break;
        }
    }
    
    vpMatrix rootP;
    
    try
    {
        rootP = root(augmentedStateCovarianceMatrix);
    }
    catch (std::exception &e)
    {
        std::cout << "usUnscentedKalmanFilter::generateSigmaPoints failed: " << e.what() << std::endl;
        return false;
    }
    
    double alpha = 0;
    const double beta = 2;
    const double k = 0; // 3 - augmentedStateDimension
    switch(m_sigmaPointsGenerationType)
    {
        case SigmaPointGenerationType::STANDARD_COVARIANCE:
        {
            alpha = 1;
            break;
        }
        case SigmaPointGenerationType::FIXED_SCALING_FACTOR:
        {
            alpha = m_sigmaPointsScalingFactor;
            break;
        }
        case SigmaPointGenerationType::LIMITED_SPREAD:
        {
            double max = 0;
            for(unsigned int i=0 ; i<m_stateDimension ; i++)
            {
                double val = stateNorm(rootP.getCol(i));
                if(val > max) max = val;
            }
            double thresholdForMax = m_sigmaPointsSpreadThreshold/(m_sigmaPointsScalingFactor*sqrt(augmentedStateDimension+k));
            alpha =  m_sigmaPointsScalingFactor * thresholdForMax / (max + thresholdForMax); 
            break;
        }
    }

    double lambda = alpha*alpha*(augmentedStateDimension+k)-augmentedStateDimension;

    vpMatrix sqrtP = sqrt(fabs(augmentedStateDimension+lambda))*rootP;

    m_sigmaPointsInit.resize(augmentedStateDimension, 1+2*augmentedStateDimension);

    vpColVector sigmaCenter(augmentedStateDimension, 0);
    sigmaCenter.insert(0, this->stateLog(m_state, m_state));
    
    for(int i=0 ; i<augmentedStateDimension ; i++)
    {
        m_sigmaPointsInit[i][0] = sigmaCenter[i];
        for(int j=0 ; j<augmentedStateDimension ; j++) m_sigmaPointsInit[i][j+1] = sigmaCenter[i] + sqrtP[i][j];
        for(int j=0 ; j<augmentedStateDimension ; j++) m_sigmaPointsInit[i][j+augmentedStateDimension+1] = sigmaCenter[i] - sqrtP[i][j];
    }

    m_sigmaPointsMeanWeights.resize(1+2*augmentedStateDimension);
    m_sigmaPointsCovarianceWeights.resize(1+2*augmentedStateDimension);

    m_sigmaPointsMeanWeights[0] = lambda / (augmentedStateDimension+lambda);
    m_sigmaPointsCovarianceWeights[0] = lambda / (augmentedStateDimension+lambda) + 1 - alpha*alpha + beta;
    m_sigmaPointsMeanWeights[1] = 1.0/(2*(augmentedStateDimension+lambda));
    m_sigmaPointsCovarianceWeights[1] = 1.0/(2*(augmentedStateDimension+lambda));

    for(unsigned int i=2 ; i<m_sigmaPointsMeanWeights.size() ; i++)
    {
        m_sigmaPointsMeanWeights[i] = m_sigmaPointsMeanWeights[i-1];
        m_sigmaPointsCovarianceWeights[i] = m_sigmaPointsCovarianceWeights[i-1];
    }
    
    return true;
}

bool usUnscentedKalmanFilter::computePropagatedSigmaPoints()
{
    m_sigmaPointsPropagated.resize(m_sigmaPointsInit.getRows(), m_sigmaPointsInit.getCols());
    for(unsigned int i=0 ; i<m_sigmaPointsInit.getCols() ; i++)
    {
        vpColVector propagatedSigmaPoint;
        try
        {
            propagatedSigmaPoint = this->propagateSigmaPoint(m_sigmaPointsInit.getCol(i));
        }
        catch(std::exception &e)
        {
            std::cout << "usUnscentedKalmanFilter::computePropagatedSigmaPoints failed: " << e.what() << std::endl;
            return false;
        }
        m_sigmaPointsPropagated.insert(propagatedSigmaPoint, 0,i);
    }
    return true;
}

bool usUnscentedKalmanFilter::computeSigmaMeasures()
{
    m_sigmaPointsMeasure.resize(m_measureDimension, m_sigmaPointsPropagated.getCols());
    vpColVector measureCenter;
    try
    {
        measureCenter = this->computeMeasureFromSigmaPoint(this->stateLog(m_state, m_state));
    }
    catch(std::exception &e)
    {
        std::cout << "usUnscentedKalmanFilter::computeSigmaMeasures failed: " << e.what() << std::endl;
        return false;
    }
    
    for(unsigned int i=0 ; i<m_sigmaPointsPropagated.getCols() ; i++)
    {
        vpColVector sigmaMeasure;
        try
        {
            sigmaMeasure = this->computeMeasureFromSigmaPoint(m_sigmaPointsPropagated.getCol(i));
        }
        catch(std::exception &e)
        {
            std::cout << "usUnscentedKalmanFilter::computeSigmaMeasures failed: " << e.what() << std::endl;
            return false;
        }
        m_sigmaPointsMeasure.insert(this->measureLog(sigmaMeasure, measureCenter), 0,i);
    }
    return true;
}

void usUnscentedKalmanFilter::computeMeansAndCovarianceMatricesFromSigmaPoints()
{
    int nbSigmaPoints = m_sigmaPointsPropagated.getCols();
    
    m_stateSigmaMean = 0;
    for(int i=0 ; i<nbSigmaPoints ; i++) m_stateSigmaMean += m_sigmaPointsMeanWeights[i] * m_sigmaPointsPropagated.getCol(i,0,m_stateDimension);

    m_stateSigmaCovarianceMatrix = 0;
    for(int i=0 ; i<nbSigmaPoints ; i++) m_stateSigmaCovarianceMatrix += m_sigmaPointsCovarianceWeights[i] * (m_sigmaPointsPropagated.getCol(i,0,m_stateDimension)-m_stateSigmaMean)*(m_sigmaPointsPropagated.getCol(i,0,m_stateDimension)-m_stateSigmaMean).t();
    
    switch(m_processNoiseType)
    {
        case NoiseType::ADDITIVE_NOISE:
        {
            m_stateSigmaCovarianceMatrix += m_processNoiseCovarianceMatrix;
            break;
        }
        case NoiseType::GENERIC_NOISE:
        {
            break;
        }
    }

    m_measureSigmaMean = 0;
    for(int i=0 ; i<nbSigmaPoints ; i++) m_measureSigmaMean += m_sigmaPointsMeanWeights[i] * m_sigmaPointsMeasure.getCol(i);

    m_stateMeasureSigmaCovarianceMatrix = 0;
    for(int i=0 ; i<nbSigmaPoints ; i++) m_stateMeasureSigmaCovarianceMatrix += m_sigmaPointsCovarianceWeights[i] * (m_sigmaPointsPropagated.getCol(i,0,m_stateDimension)-m_stateSigmaMean)*(m_sigmaPointsMeasure.getCol(i)-m_measureSigmaMean).t();
    
    m_measureSigmaCovarianceMatrix = 0;
    for(int i=0 ; i<nbSigmaPoints ; i++) m_measureSigmaCovarianceMatrix += m_sigmaPointsCovarianceWeights[i] * (m_sigmaPointsMeasure.getCol(i)-m_measureSigmaMean)*(m_sigmaPointsMeasure.getCol(i)-m_measureSigmaMean).t();
    
    switch(m_measureNoiseType)
    {
        case NoiseType::ADDITIVE_NOISE:
        {
            m_measureSigmaCovarianceMatrix += m_measureNoiseCovarianceMatrix;
            break;
        }
        case NoiseType::GENERIC_NOISE:
        {
            break;
        }
    }
}

bool usUnscentedKalmanFilter::updateState()
{
    try
    {
        vpMatrix kalmanGain = m_stateMeasureSigmaCovarianceMatrix * m_measureSigmaCovarianceMatrix.pseudoInverse();

        vpColVector Innov = this->measureLog(m_measure, this->computeMeasureFromSigmaPoint(this->stateLog(m_state, m_state))) - m_measureSigmaMean;

        m_state = this->stateExp(m_stateSigmaMean + kalmanGain * Innov, m_state);

        m_stateCovarianceMatrix = m_stateSigmaCovarianceMatrix - kalmanGain * m_measureSigmaCovarianceMatrix * kalmanGain.t();
    }
    catch (std::exception &e)
    {
        std::cout << "usUnscentedKalmanFilter::updateState failed: " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool usUnscentedKalmanFilter::filter(const vpColVector &measure)
{
    if(!this->checkConsistency(measure)) return false;

    m_measure = measure;
    
    if(m_computeProcessNoiseCovarianceMatrixAutomatically) this->computeProcessNoiseCovarianceMatrix();
    if(m_computeMeasureNoiseCovarianceMatrixAutomatically) this->computeMeasureNoiseCovarianceMatrix();
    if(!this->generateSigmaPoints()) return false;
    if(!this->computePropagatedSigmaPoints()) return false;
    if(!this->computeSigmaMeasures()) return false;
    this->computeMeansAndCovarianceMatricesFromSigmaPoints();
    
    if(!this->updateState()) return false;
    
    return true;
}

double usUnscentedKalmanFilter::stateNorm(const vpColVector &state) const
{
    return state.frobeniusNorm();
}

vpColVector usUnscentedKalmanFilter::measureLog(const vpColVector& measure, const vpColVector &measureCenter) const
{
    (void)measureCenter; // unused variable 
    return measure;
}

vpColVector usUnscentedKalmanFilter::stateLog(const vpColVector &state, const vpColVector &stateCenter) const
{
    (void)stateCenter; // unused variable 
    return state;
}

vpColVector usUnscentedKalmanFilter::stateExp(const vpColVector &state, const vpColVector &stateCenter) const
{
    (void)stateCenter; // unused variable 
    return state;
}

