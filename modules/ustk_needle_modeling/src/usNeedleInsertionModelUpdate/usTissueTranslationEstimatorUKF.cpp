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

#include <visp3/ustk_needle_modeling/usTissueTranslationEstimatorUKF.h>

#include <visp3/ustk_core/usGeometryTools.h>


usTissueTranslationEstimatorUKF::usTissueTranslationEstimatorUKF() :
  usUnscentedKalmanFilter(),
  m_var_measure_p(1e-3),
  m_var_measure_d(1e-3),
  m_var_measure_f(1e-3),
  m_var_measure_t(1e-6),
  m_var_process_p(1e-3),
  m_var_process_v(0),
  m_stateDynamicsType(usTissueTranslationEstimatorUKF::CONSTANT_POSITION),
  m_tissueTranslationType(usTissueTranslationEstimatorUKF::LATERAL_TRANSLATIONS_ONLY),
  m_measureType(usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS),
  m_propagationTime(0),
  m_needle()
{
  this->setStateDimension(3);
  m_computeProcessNoiseCovarianceMatrixAutomatically = true;
  m_computeMeasureNoiseCovarianceMatrixAutomatically = true;
}

usTissueTranslationEstimatorUKF::~usTissueTranslationEstimatorUKF()
{

}

double usTissueTranslationEstimatorUKF::getPositionMeasureNoiseVariance() const
{
  return m_var_measure_p;
}

void usTissueTranslationEstimatorUKF::setPositionMeasureNoiseVariance(double sigma)
{
  if (sigma >= 0) m_var_measure_p = sigma;
}

double usTissueTranslationEstimatorUKF::getTipDirectionMeasureNoiseVariance() const
{
  return m_var_measure_d;
}

void usTissueTranslationEstimatorUKF::setTipDirectionMeasureNoiseVariance(double sigma)
{
  if (sigma >= 0) m_var_measure_d = sigma;
}

double usTissueTranslationEstimatorUKF::getForceMeasureNoiseVariance() const
{
  return m_var_measure_f;
}

void usTissueTranslationEstimatorUKF::setForceMeasureNoiseVariance(double sigma)
{
  if (sigma >= 0) m_var_measure_f = sigma;
}

double usTissueTranslationEstimatorUKF::getTorqueMeasureNoiseVariance() const
{
  return m_var_measure_t;
}

void usTissueTranslationEstimatorUKF::setTorqueMeasureNoiseVariance(double sigma)
{
  if (sigma >= 0) m_var_measure_t = sigma;
}

double usTissueTranslationEstimatorUKF::getTissuePositionProcessNoiseVariance() const
{
  return m_var_process_p;
}

void usTissueTranslationEstimatorUKF::setTissuePositionProcessNoiseVariance(double sigma)
{
  if (sigma >= 0) m_var_process_p = sigma;
}

double usTissueTranslationEstimatorUKF::getTissueVelocityProcessNoiseVariance() const
{
  return m_var_process_v;
}

void usTissueTranslationEstimatorUKF::setTissueVelocityProcessNoiseVariance(double sigma)
{
  if (sigma >= 0) m_var_process_v = sigma;
}

usTissueTranslationEstimatorUKF::StateDynamicsType usTissueTranslationEstimatorUKF::getStateDynamicsType() const
{
  return m_stateDynamicsType;
}

void usTissueTranslationEstimatorUKF::setStateDynamicsType(usTissueTranslationEstimatorUKF::StateDynamicsType type)
{
  m_stateDynamicsType = type;

  switch (type) {
  case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
  {
    this->setStateDimension(3);
    break;
  }
  case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
  {
    this->setStateDimension(6);
    break;
  }
  }
}

usTissueTranslationEstimatorUKF::TissueTranslationType usTissueTranslationEstimatorUKF::getTissueTranslationType() const
{
  return m_tissueTranslationType;
}

void usTissueTranslationEstimatorUKF::setTissueTranslationType(usTissueTranslationEstimatorUKF::TissueTranslationType type)
{
  m_tissueTranslationType = type;
}

usTissueTranslationEstimatorUKF::MeasureType usTissueTranslationEstimatorUKF::getMeasureType() const
{
  return m_measureType;
}

void usTissueTranslationEstimatorUKF::setMeasureType(usTissueTranslationEstimatorUKF::MeasureType type)
{
  m_measureType = type;

  if (type == usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION || type == usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE) this->setMeasureDimension(6);
}

void usTissueTranslationEstimatorUKF::setPropagationTime(double time)
{
  m_propagationTime = time;
}

void usTissueTranslationEstimatorUKF::setCurrentNeedle(const usNeedleInsertionModelRayleighRitzSpline &needle)
{
  m_needle = needle;
  m_state.insert(0, vpColVector(vpColVector(m_needle.accessTissue().getPose()), 0, 3));
}

void usTissueTranslationEstimatorUKF::applyStateToNeedle(usNeedleInsertionModelRayleighRitzSpline &needle) const
{
  vpPoseVector p(needle.accessTissue().accessSurface().getPose());
  for (int i = 0; i < 3; i++) p[i] = m_state[i];
  needle.accessTissue().setPose(p);
}

bool usTissueTranslationEstimatorUKF::checkConsistency(const vpColVector &measure)
{
  if (!m_needle.IsNeedleInserted()) return false;
  if ((vpColVector(m_state, 0, 3) - vpColVector(vpColVector(m_needle.accessTissue().getPose()), 0, 3)).frobeniusNorm() > std::numeric_limits<double>::epsilon()) std::cout << "usTissueTranslationEstimatorUKF::checkConsistency: the state does not correspond to the needle model, make sure you used the method 'setCurrentNeedle' to initialize the state" << std::endl;

  if (m_measureType == usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS) {
    if (measure.size() % 3 != 0) throw vpException(vpException::notInitialized, "usTissueTranslationEstimatorUKF::checkConsistency: measure is set as needle body points, but the mesure dimension is not a multiple of 3");
    this->setMeasureDimension(measure.size());
    this->setMeasureNoiseDimension(measure.size());
  }

  return this->usUnscentedKalmanFilter::checkConsistency(measure);
}

void usTissueTranslationEstimatorUKF::computeProcessNoiseCovarianceMatrix()
{
  vpMatrix normalizedCovarianceMatrix;
  normalizedCovarianceMatrix.eye(3);

  if (m_tissueTranslationType == usTissueTranslationEstimatorUKF::LATERAL_TRANSLATIONS_ONLY) {
    vpColVector d_ins = m_needle.accessTissue().accessSurface().getDirection().normalize();
    normalizedCovarianceMatrix = normalizedCovarianceMatrix - d_ins * d_ins.t();
  }

  switch (m_stateDynamicsType) {
  case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
  {
    m_processNoiseCovarianceMatrix.resize(3, 3);
    m_processNoiseCovarianceMatrix = m_var_process_p * normalizedCovarianceMatrix;
    break;
  }
  case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
  {
    m_processNoiseCovarianceMatrix.resize(6, 6);
    m_processNoiseCovarianceMatrix.insert(m_var_process_p * normalizedCovarianceMatrix, 0, 0);
    m_processNoiseCovarianceMatrix.insert(m_var_process_v * normalizedCovarianceMatrix, 3, 3);
    break;
  }
  }
}

void usTissueTranslationEstimatorUKF::computeMeasureNoiseCovarianceMatrix()
{
  switch (m_measureType) {
  case usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS:
  {
    m_measureNoiseCovarianceMatrix.resize(m_measureDimension, m_measureDimension);
    m_measureNoiseCovarianceMatrix.diag(m_var_measure_p);
    break;
  }
  case usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION:
  {
    m_measureNoiseCovarianceMatrix.resize(6, 6);
    for (int i = 0; i < 3; i++) {
      m_measureNoiseCovarianceMatrix[i][i] = m_var_measure_p;
      m_measureNoiseCovarianceMatrix[i + 3][i + 3] = m_var_measure_d;
    }
    break;
  }
  case usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE:
  {
    m_measureNoiseCovarianceMatrix.resize(6, 6);
    for (int i = 0; i < 3; i++) {
      m_measureNoiseCovarianceMatrix[i][i] = m_var_measure_f;
      m_measureNoiseCovarianceMatrix[i + 3][i + 3] = m_var_measure_t;
    }
    break;
  }
  }
}

vpColVector usTissueTranslationEstimatorUKF::propagateSigmaPoint(const vpColVector &sigmaPoint)
{
  switch (m_stateDynamicsType) {
  case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
  {
    return sigmaPoint;
  }
  case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
  {
    vpColVector propagatedSigmaPoint(sigmaPoint);
    for (int i = 0; i < 3; i++) propagatedSigmaPoint[i] += m_propagationTime * propagatedSigmaPoint[i + 3];
    return propagatedSigmaPoint;
  }
  default:
  {
    return sigmaPoint;
  }
  }
}

vpColVector usTissueTranslationEstimatorUKF::computeMeasureFromSigmaPoint(const vpColVector &sigmaPoint)
{
  switch (m_measureType) {
  case usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS:
  {
    int nbPoints = m_measure.size() / 3;

    usNeedleInsertionModelRayleighRitzSpline testNeedle(m_needle);

    std::vector<int> index(nbPoints);
    std::vector<double> l(nbPoints);
    for (int i = 0; i < nbPoints; i++) {
      usGeometryTools::projectPointOnCurve(vpColVector(m_measure, 3 * i, 3), testNeedle.accessNeedle(), -1, &(index.at(i)), &(l.at(i)));
    }

    vpPoseVector p(testNeedle.accessTissue().getPose());
    testNeedle.accessTissue().setPose(vpPoseVector(sigmaPoint[0], sigmaPoint[1], sigmaPoint[2], p[3], p[4], p[5]));
    testNeedle.updateState();

    vpColVector measure(m_measure.size());
    for (int i = 0; i < nbPoints; i++) measure.insert(3 * i, testNeedle.accessNeedle().accessSegment(index.at(i)).getPoint(l.at(i)));

    return measure;
  }
  case usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION:
  {
    usNeedleInsertionModelRayleighRitzSpline testNeedle(m_needle);

    vpPoseVector p(testNeedle.accessTissue().getPose());
    testNeedle.accessTissue().setPose(vpPoseVector(sigmaPoint[0], sigmaPoint[1], sigmaPoint[2], p[3], p[4], p[5]));
    testNeedle.updateState();

    vpColVector measure(6);
    measure.insert(0, testNeedle.accessNeedle().getTipPosition());
    measure.insert(3, testNeedle.accessNeedle().getTipDirection());

    return measure;
  }
  case usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE:
  {
    usNeedleInsertionModelRayleighRitzSpline testNeedle(m_needle);

    vpPoseVector p(testNeedle.accessTissue().getPose());
    testNeedle.accessTissue().setPose(vpPoseVector(sigmaPoint[0], sigmaPoint[1], sigmaPoint[2], p[3], p[4], p[5]));
    testNeedle.updateState();

    vpColVector measure(testNeedle.accessNeedle().getBaseStaticTorsor());

    return measure;
  }
  default:
  {
    throw vpException(vpException::notImplementedError, "usTissueTranslationEstimatorUKF::computeMeasureFromSigmaPoint: measure type not implemented");
  }
  }
}

double usTissueTranslationEstimatorUKF::stateNorm(const vpColVector &state) const
{
  switch (m_stateDynamicsType) {
  case usTissueTranslationEstimatorUKF::CONSTANT_POSITION:
  {
    return vpColVector(state, 0, 3).frobeniusNorm();
  }
  case usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY:
  {
    return vpColVector(state, 0, 3).frobeniusNorm() + m_propagationTime * vpColVector(state, 3, 3).frobeniusNorm();
  }
  default:
  {
    return state.frobeniusNorm();
  }
  }
}

vpColVector usTissueTranslationEstimatorUKF::measureLog(const vpColVector &measure, const vpColVector &measureCenter) const
{
  switch (m_measureType) {
  case usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS:
  {
    return measure;
  }
  case usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION:
  {
    vpColVector measureL(6);
    for (int i = 0; i < 3; i++) measureL[i] = measure[i];

    vpColVector d_init(measureCenter, 3, 3);
    vpColVector d(measure, 3, 3);
    double angle = fabs(atan2(vpColVector::crossProd(d_init, d).frobeniusNorm(), vpColVector::dotProd(d_init, d)));
    measureL.insert(3, angle * vpColVector::crossProd(d_init, d).normalize());

    return measureL;
  }
  case usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE:
  {
    return measure;
  }
  default:
  {
    return measure;
  }
  }
}
