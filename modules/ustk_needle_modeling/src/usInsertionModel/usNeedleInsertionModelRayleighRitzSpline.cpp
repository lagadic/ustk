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

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelRayleighRitzSpline.h>

#include <visp3/ustk_core/usGeometryTools.h>

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_EIGEN3
#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/SparseLU>
#endif

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/gui/vpDisplayX.h>

#include <iomanip>

usNeedleInsertionModelRayleighRitzSpline::usNeedleInsertionModelRayleighRitzSpline()
  : m_needle(), m_needleTip(new usNeedleTipBeveled()), m_needleTipType(NeedleTipType::BeveledTip), m_tissue(),
    m_pathUpdateType(PathUpdateType::NoUpdate), m_pathUpdateLengthThreshold(0.001), m_pathUpdateMixCoefficient(0.5),
    m_solvingMethod(SolvingMethod::FixedBeamLength)
{
  m_stiffnessPerUnitLength.resize(1, 10000);
  m_layerLength.resize(1, 1);
  this->updateState();
}

usNeedleInsertionModelRayleighRitzSpline::usNeedleInsertionModelRayleighRitzSpline(
    const usNeedleInsertionModelRayleighRitzSpline &model)
  : m_needle(model.m_needle), m_needleTip(model.m_needleTip->clone()), m_needleTipType(model.m_needleTipType),
    m_tissue(model.m_tissue), m_stiffnessPerUnitLength(model.m_stiffnessPerUnitLength),
    m_layerLength(model.m_layerLength),

    m_pathUpdateType(model.m_pathUpdateType), m_pathUpdateLengthThreshold(model.m_pathUpdateLengthThreshold),
    m_pathUpdateMixCoefficient(model.m_pathUpdateMixCoefficient),

    m_solvingMethod(model.m_solvingMethod), m_restDilatationFactor(model.m_restDilatationFactor)
{
}

usNeedleInsertionModelRayleighRitzSpline::~usNeedleInsertionModelRayleighRitzSpline()
{
  if (m_needleTip != nullptr)
    delete m_needleTip;
}

const usNeedleInsertionModelRayleighRitzSpline &usNeedleInsertionModelRayleighRitzSpline::
operator=(const usNeedleInsertionModelRayleighRitzSpline &model)
{
  m_needle = model.m_needle;
  if (m_needleTip)
    delete m_needleTip;
  m_needleTip = model.m_needleTip->clone();
  m_needleTipType = model.m_needleTipType;
  m_tissue = model.m_tissue;
  m_stiffnessPerUnitLength = model.m_stiffnessPerUnitLength;
  m_layerLength = model.m_layerLength;

  m_pathUpdateType = model.m_pathUpdateType;
  m_pathUpdateLengthThreshold = model.m_pathUpdateLengthThreshold;
  m_pathUpdateMixCoefficient = model.m_pathUpdateMixCoefficient;

  m_solvingMethod = model.m_solvingMethod;
  m_restDilatationFactor = model.m_restDilatationFactor;

  return *this;
}

usNeedleInsertionModelRayleighRitzSpline *usNeedleInsertionModelRayleighRitzSpline::clone() const
{
  return new usNeedleInsertionModelRayleighRitzSpline(*this);
}

void usNeedleInsertionModelRayleighRitzSpline::loadPreset(const ModelPreset preset)
{
  switch (preset) {
  case ModelPreset::BiopsyNeedle: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::BiopsyNeedle);
    this->setNeedleTipType(NeedleTipType::BeveledTip);
    usNeedleTipBeveled *t = dynamic_cast<usNeedleTipBeveled *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d / tan(M_PI / 180 * 24));
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 20000);
    break;
  }
  case ModelPreset::BiopsyCannula: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::BiopsyCannula);
    this->setNeedleTipType(NeedleTipType::BeveledTip);
    usNeedleTipBeveled *t = dynamic_cast<usNeedleTipBeveled *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d / tan(M_PI / 180 * 24));
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 20000);
    break;
  }
  case ModelPreset::Symmetric: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::Symmetric);
    this->setNeedleTipType(NeedleTipType::SymmetricTip);
    usNeedleTipSymmetric *t = dynamic_cast<usNeedleTipSymmetric *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d / 2 / tan(M_PI / 180 * 30 / 2));
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 20000);
    break;
  }
  case ModelPreset::AbayazidRRM13: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::AbayazidRRM13);
    this->setNeedleTipType(NeedleTipType::BeveledTip);
    usNeedleTipBeveled *t = dynamic_cast<usNeedleTipBeveled *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d / tan(M_PI / 180 * 30));
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 35500 * 4 * 0.2 / 0.17);
    break;
  }
  case ModelPreset::MisraRSRO10_PlastisolA: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::MisraRSRO10_PlastisolA);
    this->setNeedleTipType(NeedleTipType::BeveledTip);
    usNeedleTipBeveled *t = dynamic_cast<usNeedleTipBeveled *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d / tan(M_PI / 180 * 32.09));
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 4830);
    break;
  }
  case ModelPreset::RoesthuisAM12: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::RoesthuisAM12);
    this->setNeedleTipType(NeedleTipType::BeveledTip);
    usNeedleTipBeveled *t = dynamic_cast<usNeedleTipBeveled *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d / tan(M_PI / 180 * 30));
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 150000);
    break;
  }
  case ModelPreset::SteelSoftTissue: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::SteelSoftTissue);
    this->setNeedleTipType(NeedleTipType::BeveledTip);
    usNeedleTipBeveled *t = dynamic_cast<usNeedleTipBeveled *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d);
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 500);
    break;
  }
  case ModelPreset::SRL_ActuatedFBG: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::SRL_ActuatedFBG);
    this->setNeedleTipType(NeedleTipType::ActuatedTip);
    usNeedleTipActuated *t = dynamic_cast<usNeedleTipActuated *>(m_needleTip);
    t->setDiameter(m_needle.getOuterDiameter());
    t->setLength(0.005);
    t->setTipAngleRad(0);
    t->setSteeringAngleRad(0);
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 35000);
    break;
  }
  case ModelPreset::SRL_BiopsySimple: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::SRL_BiopsySimple);
    this->setNeedleTipType(NeedleTipType::BeveledTip);
    usNeedleTipBeveled *t = dynamic_cast<usNeedleTipBeveled *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d / tan(M_PI / 180 * 45));
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 20000);
    break;
  }
  case ModelPreset::SRL_BiopsyNID: {
    m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::SRL_BiopsyNID);
    this->setNeedleTipType(NeedleTipType::BeveledTip);
    usNeedleTipBeveled *t = dynamic_cast<usNeedleTipBeveled *>(m_needleTip);
    double d = m_needle.getOuterDiameter();
    t->setDiameter(d);
    t->setLength(d / tan(M_PI / 180 * 30));
    for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
      this->setStiffnessPerUnitLength(i, 20000);
    break;
  }
  }
}

const usNeedleModelSpline &usNeedleInsertionModelRayleighRitzSpline::accessNeedle() const { return m_needle; }

usNeedleModelSpline &usNeedleInsertionModelRayleighRitzSpline::accessNeedle() { return m_needle; }

usNeedleTip const &usNeedleInsertionModelRayleighRitzSpline::accessNeedleTip() const { return *m_needleTip; }

usNeedleTip &usNeedleInsertionModelRayleighRitzSpline::accessNeedleTip() { return *m_needleTip; }

const usTissueModelSpline &usNeedleInsertionModelRayleighRitzSpline::accessTissue() const { return m_tissue; }

usTissueModelSpline &usNeedleInsertionModelRayleighRitzSpline::accessTissue() { return m_tissue; }

void usNeedleInsertionModelRayleighRitzSpline::setNeedleTipType(NeedleTipType type)
{
  if (type == m_needleTipType)
    return;

  vpPoseVector p(m_needleTip->getBasePose());
  delete m_needleTip;
  m_needleTip = nullptr;

  switch (type) {
  case NeedleTipType::ActuatedTip: {
    m_needleTip = new usNeedleTipActuated();
    m_needleTipType = NeedleTipType::ActuatedTip;
    break;
  }
  case NeedleTipType::BeveledTip: {
    m_needleTip = new usNeedleTipBeveled();
    m_needleTipType = NeedleTipType::BeveledTip;
    break;
  }
  case NeedleTipType::PrebentTip: {
    m_needleTip = new usNeedleTipPrebent();
    m_needleTipType = NeedleTipType::PrebentTip;
    break;
  }
  case NeedleTipType::SymmetricTip: {
    m_needleTip = new usNeedleTipSymmetric();
    m_needleTipType = NeedleTipType::SymmetricTip;
    break;
  }
  }

  m_needleTip->setBasePose(p);
}

usNeedleInsertionModelRayleighRitzSpline::NeedleTipType
usNeedleInsertionModelRayleighRitzSpline::getNeedleTipType() const
{
  return m_needleTipType;
}

bool usNeedleInsertionModelRayleighRitzSpline::addTissueLayer(double K, double l)
{
  if (K <= 0 || l <= 0)
    return false;

  m_stiffnessPerUnitLength.push_back(K);
  m_layerLength.push_back(l);
  return true;
}

bool usNeedleInsertionModelRayleighRitzSpline::setStiffnessPerUnitLength(int i, double K)
{
  if (K <= 0)
    return false;
  m_stiffnessPerUnitLength.at(i) = K;
  return true;
}

bool usNeedleInsertionModelRayleighRitzSpline::setStiffnessPerUnitLength(double K)
{
  if (K <= 0)
    return false;
  for (unsigned int i = 0; i < m_stiffnessPerUnitLength.size(); i++)
    m_stiffnessPerUnitLength.at(i) = K;
  return true;
}

double usNeedleInsertionModelRayleighRitzSpline::getStiffnessPerUnitLength(int i) const
{
  return m_stiffnessPerUnitLength.at(i);
}

bool usNeedleInsertionModelRayleighRitzSpline::setLayerLength(int i, double l)
{
  if (l <= 0)
    return false;
  m_layerLength.at(i) = l;
  return true;
}

double usNeedleInsertionModelRayleighRitzSpline::getLayerLength(int i) const { return m_layerLength.at(i); }

int usNeedleInsertionModelRayleighRitzSpline::getNbLayers() const { return m_layerLength.size(); }

int usNeedleInsertionModelRayleighRitzSpline::getNbCurrentLayers() const
{
  int n = 0;
  double l = 0;
  while (n + 1 < this->getNbLayers() && l < this->getInsertionDepth()) {
    l += m_layerLength.at(n);
    n++;
  }
  return n;
}

void usNeedleInsertionModelRayleighRitzSpline::setPathUpdateType(PathUpdateType type) { m_pathUpdateType = type; }

void usNeedleInsertionModelRayleighRitzSpline::setPathUpdateLengthThreshold(double length)
{
  m_pathUpdateLengthThreshold = length;
}

void usNeedleInsertionModelRayleighRitzSpline::setPathUpdateMixCoefficient(double coef)
{
  m_pathUpdateMixCoefficient = coef;
}

void usNeedleInsertionModelRayleighRitzSpline::setSolvingMethod(SolvingMethod method) { m_solvingMethod = method; }

bool usNeedleInsertionModelRayleighRitzSpline::IsNeedleInserted() const
{
  return usGeometryTools::IsPointInFrontOfPlane(m_needle.getTipPosition(), m_tissue.accessSurface());
}

double usNeedleInsertionModelRayleighRitzSpline::getNeedleFreeLength(int *seg, double *param) const
{
  if (this->IsNeedleInserted()) {
    if (usGeometryTools::IsPointInFrontOfPlane(m_needle.accessSegment(0).getStartPoint(), m_tissue.accessSurface())) {
      if (seg != nullptr)
        *seg = -1;
      if (param != nullptr)
        *param = -1;
      return 0;
    }

    int nbSeg = m_needle.getNbSegments();
    int i = 0;
    double l = 0;
    double ltot = 0;
    while (i < nbSeg) {
      if (!usGeometryTools::IsPointInFrontOfPlane(m_needle.accessSegment(i).getEndPoint(), m_tissue.accessSurface())) {
        ltot += m_needle.accessSegment(i).getParametricLength();
        i++;
      } else {
        if (usGeometryTools::DoesSegmentCrossPlane(m_needle.accessSegment(i), m_tissue.accessSurface())) {
          usGeometryTools::getPlaneCurveCrossingPoint(m_needle.accessSegment(i), m_tissue.accessSurface(), -1, &l);
          ltot += l;
        }
        break;
      }
    }

    if (i < nbSeg) {
      if (seg != nullptr)
        *seg = i;
      if (param != nullptr)
        *param = l;
    } else {
      if (seg != nullptr)
        *seg = -1;
      if (param != nullptr)
        *param = -1;
    }
    return ltot;
  } else {
    if (seg != nullptr)
      *seg = -1;
    if (param != nullptr)
      *param = -1;
    double L = 0;
    for (int i = 0; i < m_needle.getNbSegments(); i++)
      L += m_needle.accessSegment(i).getParametricLength();
    return L;
  }
}

double usNeedleInsertionModelRayleighRitzSpline::getInsertionDepth() const
{
  return m_needle.getFullLength() - this->getNeedleFreeLength();
}

vpColVector usNeedleInsertionModelRayleighRitzSpline::getNeedleInsertionPoint() const
{
  if (!this->IsNeedleInserted())
    throw vpException(vpException::fatalError,
                      "usNeedleInsertionModelRayleighRitzSpline::getNeedleInsertionPoint: needle is not inserted");

  return usGeometryTools::projectPointOnPlane(m_needle.getPoint(this->getNeedleFreeLength()), m_tissue.accessSurface());
}

vpColVector usNeedleInsertionModelRayleighRitzSpline::getTissueInsertionPoint() const
{
  if (m_tissue.accessPath().getNbSegments() < 1)
    throw vpException(vpException::fatalError,
                      "usNeedleInsertionModelRayleighRitzSpline::getTissueInsertionPoint: needle was not inserted");

  return m_tissue.accessPath().getPoint(0);
}

bool usNeedleInsertionModelRayleighRitzSpline::getCorrespondingPathPoint(double l, int &correspondingRestIndex,
                                                                         double &correspondingRestParam) const
{
  correspondingRestIndex = -1;
  correspondingRestParam = -1;
  if (m_tissue.accessPath().getNbSegments() < 1)
    return false;

  int needleIndex = -1;
  double needleParam = -1;
  double freeLength = this->getNeedleFreeLength(&needleIndex, &needleParam);
  if (l < freeLength || needleIndex < 0 || needleParam < 0)
    return false;

  double insertionLength = l - freeLength;
  double currentNeedleInsertedLength = 0;
  int restIndex = 0;
  double currentRestParam = 0;
  double currentRestLength = 0;

  while (currentNeedleInsertedLength < insertionLength) {
    // Go forward in needle

    if (needleIndex < m_needle.getNbSegments() - 1 &&
        currentNeedleInsertedLength + m_needle.accessSegment(needleIndex).getParametricLength() - needleParam <
            insertionLength) {
      currentNeedleInsertedLength += m_needle.accessSegment(needleIndex).getParametricLength() - needleParam;
      needleParam = 0;
    } else {
      currentNeedleInsertedLength = insertionLength;
    }

    while (restIndex < m_tissue.accessPath().getNbSegments() - 1 &&
           currentRestLength +
                   m_restDilatationFactor.at(needleIndex) *
                       (m_tissue.accessPath().accessSegment(restIndex).getParametricLength() - currentRestParam) <
               currentNeedleInsertedLength) {
      currentRestLength += m_restDilatationFactor.at(needleIndex) *
                           (m_tissue.accessPath().accessSegment(restIndex).getParametricLength() - currentRestParam);
      currentRestParam = 0;
      restIndex++;
    }

    currentRestParam += (currentNeedleInsertedLength - currentRestLength) / m_restDilatationFactor.at(needleIndex);
    currentRestLength = currentNeedleInsertedLength;
    needleIndex++;
  }

  correspondingRestIndex = restIndex;
  correspondingRestParam = currentRestParam;

  return true;
}

double usNeedleInsertionModelRayleighRitzSpline::getTissueDeformationEnergy() const
{
  double E = 0;

  int nbSeg = m_needle.getNbSegments();
  double totalLength = this->getNeedleFreeLength();

  double layerIndex = 0;
  double currentDepth = 0;
  double nextDepth = m_layerLength.front();
  double Kt = m_stiffnessPerUnitLength.front();

  for (int i = 1; i < nbSeg; i++) {
    const usPolynomialCurve3D &segment = m_needle.accessSegment(i);

    double dE = 0;
    double l = 0;
    double length = segment.getParametricLength();
    double dl = length / 50;

    int restIndex = -1;
    double restParameter = -1;
    vpColVector Pn = segment.getPoint(l);
    if (this->getCorrespondingPathPoint(totalLength + l, restIndex, restParameter)) {
      dE += Kt * (Pn - m_tissue.accessPath().accessSegment(restIndex).getPoint(restParameter)).sumSquare() / 2;
    }

    while (l < length) {
      Pn = segment.getPoint(l);

      if (this->getCorrespondingPathPoint(totalLength + l, restIndex, restParameter)) {
        dE += Kt * (Pn - m_tissue.accessPath().accessSegment(restIndex).getPoint(restParameter)).sumSquare();
      }

      l += dl;
      currentDepth += dl;

      while (layerIndex + 1 < m_layerLength.size() && currentDepth > nextDepth) {
        layerIndex++;
        nextDepth += m_layerLength.at(layerIndex);
        Kt = m_stiffnessPerUnitLength.at(layerIndex);
      }
    }

    l = length;
    Pn = segment.getPoint(l);

    if (this->getCorrespondingPathPoint(l, restIndex, restParameter)) {
      dE += Kt * (Pn - m_tissue.accessPath().accessSegment(restIndex).getPoint(restParameter)).sumSquare() / 2;
    }
    E += dl * dE;
  }

  return E;
}

double usNeedleInsertionModelRayleighRitzSpline::getSurfaceTissueStretch() const
{
  if (!this->IsNeedleInserted() || (m_tissue.accessPath().getNbSegments() < 1))
    return 0;

  return (this->getTissueInsertionPoint() - this->getNeedleInsertionPoint()).euclideanNorm();
}

double usNeedleInsertionModelRayleighRitzSpline::getMaxTissueStretch(double *lmax) const
{
  if (!this->IsNeedleInserted()) {
    if (lmax != nullptr)
      *lmax = 0;
    return 0;
  }

  double max = 0;
  double maxL = 0;

  int nbSeg = m_needle.getNbSegments();
  int i = 0;
  double l = 0;
  double freeLength = this->getNeedleFreeLength(&i, &l);
  if (i == -1) {
    if (lmax != nullptr)
      *lmax = 0;
    return 0;
  }
  double totalLength = freeLength - l;

  while (i < nbSeg) {
    const usPolynomialCurve3D &segment = m_needle.accessSegment(i);

    double length = segment.getParametricLength();
    double dl = (length - l) / 50;

    int restIndex = -1;
    double restParameter = -1;
    vpColVector Pn;

    while (l < length) {
      Pn = segment.getPoint(l);

      if (this->getCorrespondingPathPoint(totalLength + l, restIndex, restParameter)) {
        double s = (Pn - m_tissue.accessPath().accessSegment(restIndex).getPoint(restParameter)).euclideanNorm();

        if (s > max) {
          max = s;
          maxL = totalLength + l;
        }
      }

      l += dl;
    }

    l = length;
    Pn = segment.getPoint(l);

    if (this->getCorrespondingPathPoint(totalLength + l, restIndex, restParameter)) {
      double s = (Pn - m_tissue.accessPath().accessSegment(restIndex).getPoint(restParameter)).euclideanNorm();

      if (s > max) {
        max = s;
        maxL = totalLength + l;
      }
    }

    l = 0;
    totalLength += length;
    i++;
  }

  if (lmax != nullptr)
    *lmax = maxL;

  return max;
}

double usNeedleInsertionModelRayleighRitzSpline::getMeanTissueStretch() const
{
  if (!this->IsNeedleInserted())
    return 0;

  double mean = 0;

  int nbSeg = m_needle.getNbSegments();
  int i = 0;
  double l = 0;
  double freeLength = this->getNeedleFreeLength(&i, &l);
  if (i == -1)
    return 0;
  double totalLength = freeLength - l;

  while (i < nbSeg) {
    const usPolynomialCurve3D &segment = m_needle.accessSegment(i);

    double dmean = 0;

    double length = segment.getParametricLength();
    double dl = (length - l) / 50;

    int restIndex = -1;
    double restParameter = -1;
    vpColVector Pn = segment.getPoint(l);
    if (this->getCorrespondingPathPoint(totalLength + l, restIndex, restParameter)) {
      dmean += (Pn - m_tissue.accessPath().accessSegment(restIndex).getPoint(restParameter)).euclideanNorm() / 2;
    }

    while (l < length) {
      Pn = segment.getPoint(l);

      if (this->getCorrespondingPathPoint(totalLength + l, restIndex, restParameter)) {
        dmean += (Pn - m_tissue.accessPath().accessSegment(restIndex).getPoint(restParameter)).euclideanNorm();
      }

      l += dl;
    }

    l = length;
    Pn = segment.getPoint(l);

    if (this->getCorrespondingPathPoint(totalLength + l, restIndex, restParameter)) {
      dmean += (Pn - m_tissue.accessPath().accessSegment(restIndex).getPoint(restParameter)).euclideanNorm() / 2;
    }
    mean += dl * dmean;

    l = 0;
    totalLength += length;
    i++;
  }

  mean /= (totalLength - freeLength);

  return mean;
}

bool usNeedleInsertionModelRayleighRitzSpline::setBasePose(const vpPoseVector &pose)
{
  // Set base position in worldframe if the base doesn't go under the tissue

  if (m_tissue.accessSurface().getDirection().euclideanNorm() > 0) {
    vpColVector position(pose.getTranslationVector());
    if (usGeometryTools::IsPointInFrontOfPlane(position, m_tissue.accessSurface()))
      return false;
  }

  m_needle.setBasePose(pose);

  this->updateState();

  return true;
}

vpPoseVector usNeedleInsertionModelRayleighRitzSpline::getBasePose() const { return m_needle.getBasePose(); }

void usNeedleInsertionModelRayleighRitzSpline::setSurfaceAtTip()
{
  vpColVector t = m_needleTip->getTipPosition();
  vpPoseVector pose(m_needle.getTipPose());
  for (int i = 0; i < 3; i++)
    pose[i] = t[i];
  m_tissue.accessSurface().setPose(pose);
}

bool usNeedleInsertionModelRayleighRitzSpline::cutPathToPoint(const vpColVector &P)
{
  if (P.size() != 3)
    throw vpException(vpException::dimensionError,
                      "usNeedleInsertionModelRayleighRitzSpline::cutPathToPoint: invalid vector dimension");

  vpColVector lastPoint(3);
  vpColVector lastDirection(3);
  if (m_tissue.accessPath().getNbSegments() > 0) {
    lastPoint = m_tissue.accessPath().accessLastSegment().getEndPoint();
    lastDirection = m_needle.getTipDirection(); // m_tissue.accessPath().accessLastSegment().getEndTangent();
  } else if (usGeometryTools::IsPointInFrontOfPlane(P, m_tissue.accessSurface())) {
    lastDirection = m_needle.getTipDirection();
    lastPoint = usGeometryTools::projectPointOnPlane(P, m_tissue.accessSurface(), lastDirection);
  } else
    return false;

  if (vpColVector::dotProd(P - lastPoint, lastDirection) > m_pathUpdateLengthThreshold) {
    if ((P - m_needle.getBasePosition()).euclideanNorm() > m_needle.getFullLength() + 0.05) {
      std::cout << "Warning usNeedleInsertionModelRayleighRitzSpline::cutPathToPoint: cut point is inconsistent with "
                   "current needle state"
                << std::endl;
      return false;
    }

    usPolynomialCurve3D seg(1);
    vpMatrix M(3, 2);
    M.insert(lastPoint, 0, 0);
    M.insert((P - lastPoint).normalize(), 0, 1);
    seg.setPolynomialCoefficients(M);
    seg.setParametricLength((P - lastPoint).euclideanNorm());
    m_tissue.accessPath().addSegment(seg);
    return true;
  } else
    return false;
}

void usNeedleInsertionModelRayleighRitzSpline::solveSegmentsParametersSparseEigen()
{
#ifdef VISP_HAVE_EIGEN3

  double trueFreeLength = 0;

  if (usGeometryTools::DoesSegmentCrossPlane(m_needle, m_tissue.accessSurface())) {
    double freeLength = -1;
    usGeometryTools::getPlaneCurveCrossingPoint(m_needle, m_tissue.accessSurface(), -1, &freeLength);
    int seg = -1;
    double param = 0;
    m_needle.getParametersFromLength(freeLength, seg, param);

    for (int i = 0; i < seg; i++) {
      trueFreeLength += m_needle.accessSegment(i).getLength();
    }
    trueFreeLength += m_needle.accessSegment(seg).getSubPolynomialCurve(0, param).getLength();
  } else {
    vpColVector p(3);
    for (int i = 0; i < 3; i++)
      p[i] = m_needle.getBasePose()[i];
    vpColVector d(3);
    for (int i = 0; i < 3; i++)
      d[i] = m_needle.getWorldMbase()[i][2];
    double cosTheta = fabs(vpColVector::dotProd(m_tissue.accessSurface().getDirection(), d));
    if (cosTheta > std::numeric_limits<double>::epsilon())
      trueFreeLength = fabs(usGeometryTools::getPointPlaneDistance(p, m_tissue.accessSurface())) / cosTheta;
    else
      trueFreeLength = m_needle.getFullLength();
  }

  if (trueFreeLength >= m_needle.getFullLength() ||
      m_tissue.accessPath().getNbSegments() == 0) // needle is not inserted
  {
    usPolynomialCurve3D seg(m_needle.accessSegment(0));
    vpMatrix M(3, seg.getOrder() + 1, 0);
    for (int dim = 0; dim < 3; dim++) {
      M[dim][0] = m_needle.getBasePose()[dim];
      M[dim][1] = m_needle.getWorldMbase()[dim][2];
    }
    seg.setPolynomialCoefficients(M);
    seg.setParametricLength(m_needle.getFullLength());
    m_needle.init();
    m_needle.setSegment(0, seg);

    return;
  }

  m_needle.accessSegment(0).setParametricLength(trueFreeLength);

  double segDefaultLength = 0.01;
  int nbSegments = 1 + floor((m_needle.getFullLength() - trueFreeLength) / segDefaultLength) + 1;
  while (m_needle.getNbSegments() > nbSegments) {
    m_needle.removeLastSegment();
  }
  while (m_needle.getNbSegments() < nbSegments) {
    m_needle.addSegment(m_needle.accessLastSegment());
  }
  m_restDilatationFactor.resize(m_needle.getNbSegments() - 1, 1);
  for (int i = 1; i < nbSegments - 1; i++)
    m_needle.accessSegment(i).setParametricLength(segDefaultLength);
  m_needle.accessLastSegment().setParametricLength(
      m_needle.getFullLength() - m_needle.accessSegment(0).getParametricLength() - segDefaultLength * (nbSegments - 2));

  int nbcoef = 0;
  for (int i = 0; i < nbSegments; i++)
    nbcoef += m_needle.accessSegment(i).getOrder() + 1;

  typedef Eigen::Triplet<double> T;
  std::vector<T> L;

  double EI = m_needle.getEI();

  int nbConstraints = 2;
  for (int i = 0; i < nbSegments - 1; i++) {
    int order1 = m_needle.accessSegment(i).getOrder();
    int order2 = m_needle.accessSegment(i + 1).getOrder();
    int order = std::max(order1, order2);
    nbConstraints += std::min(3, order);
  }

  L.reserve((nbConstraints + nbcoef) * (nbConstraints + nbcoef) / 10);

  Eigen::MatrixX3d A = Eigen::MatrixX3d::Zero(nbConstraints + nbcoef, 3);
  Eigen::MatrixX3d B = Eigen::MatrixX3d::Zero(nbConstraints + nbcoef, 3);

  // Constraints matrix

  // Base conditions
  int line = nbcoef;
  for (int dim = 0; dim < 3; dim++)
    B(line, dim) = m_needle.getBasePose()[dim];
  L.push_back(T(line, 0, 1));
  L.push_back(T(0, line, -1));
  line++;
  for (int dim = 0; dim < 3; dim++)
    B(line, dim) = m_needle.getWorldMbase()[dim][2];
  L.push_back(T(line, 1, 1));
  L.push_back(T(1, line, -1));
  line++;

  int nbSegCoef = 0;
  int startIndex = 0;
  for (int i = 0; i < nbSegments - 1; i++) {
    nbSegCoef = m_needle.accessSegment(i).getOrder() + 1;
    int order1 = m_needle.accessSegment(i).getOrder();
    int order2 = m_needle.accessSegment(i + 1).getOrder();
    int order = std::max(order1, order2);
    double segLength = m_needle.accessSegment(i).getParametricLength();

    // Continuity

    double *tmp = new double[nbSegCoef];
    if (order > 0) // Order 0
    {
      tmp[0] = 1;
      for (int j = 1; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];

      for (int j = 0; j < nbSegCoef; j++) {
        L.push_back(T(line, startIndex + j, tmp[j]));
        L.push_back(T(startIndex + j, line, -tmp[j]));
      }
      L.push_back(T(line, startIndex + nbSegCoef, -1));
      L.push_back(T(startIndex + nbSegCoef, line, 1));
      line++;
    }
    if (order > 1) // Order 1
    {
      tmp[0] = 0;
      tmp[1] = 1;
      for (int j = 2; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];
      for (int j = 1; j < nbSegCoef; j++)
        tmp[j] *= j;

      for (int j = 1; j < nbSegCoef; j++) {
        L.push_back(T(line, startIndex + j, tmp[j]));
        L.push_back(T(startIndex + j, line, -tmp[j]));
      }
      L.push_back(T(line, startIndex + nbSegCoef + 1, -1));
      L.push_back(T(startIndex + nbSegCoef + 1, line, 1));
      line++;
    }
    if (order > 2) // Order 2
    {
      tmp[0] = 0;
      tmp[1] = 0;
      tmp[2] = 1;
      for (int j = 3; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];
      for (int j = 2; j < nbSegCoef; j++)
        tmp[j] *= j * (j - 1);

      for (int j = 2; j < nbSegCoef; j++) {
        L.push_back(T(line, startIndex + j, tmp[j]));
        L.push_back(T(startIndex + j, line, -tmp[j]));
      }
      L.push_back(T(line, startIndex + nbSegCoef + 2, -2));
      L.push_back(T(startIndex + nbSegCoef + 2, line, 2));
      line++;
    }
    delete[] tmp;
    startIndex += nbSegCoef;
  }

  // Optimization matrix

  line = 0;

  nbSegCoef = m_needle.accessSegment(0).getOrder() + 1;
  int nbSegEq = nbSegCoef;
  double segLength = m_needle.accessSegment(0).getParametricLength();
  for (int j = 0; j < nbSegEq; j++) {
    // Bending minimization
    for (int k = 2; k < nbSegCoef; k++) {
      if (j > 1)
        L.push_back(T(line, k, EI * pow(segLength, j + k - 3) * j * (j - 1) * k * (k - 1) / (j + k - 3)));
    }
    line++;
  }

  startIndex = nbSegCoef;
  int restIndex = 0;
  double currentRestParam = 0;
  int layerIndex = 0;
  double currentDepth = 0;
  double nextLayerDepth = m_layerLength.front();
  for (int i = 1; i < nbSegments; i++) {
    nbSegCoef = m_needle.accessSegment(i).getOrder() + 1;
    nbSegEq = nbSegCoef;
    segLength = m_needle.accessSegment(i).getParametricLength();

    double lseg = 0;
    double LsegMin = 0;
    double LsegMax = segLength;
    while (lseg < segLength) {
      double stiffnessPerUnitLength = m_stiffnessPerUnitLength.at(layerIndex);

      if (currentDepth - lseg + segLength > nextLayerDepth && layerIndex + 1 < (int)m_layerLength.size()) {
        LsegMax = nextLayerDepth - currentDepth;
      }

      for (int j = 0; j < nbSegEq; j++) {
        for (int k = 0; k < nbSegCoef; k++) {
          double c = 0;
          // Bending minimization
          if (j > 1 && k > 1)
            c += EI * (pow(LsegMax, j + k - 3) - pow(LsegMin, j + k - 3)) * j * (j - 1) * k * (k - 1) / (j + k - 3);
          // if(i==0) c*=100;
          // Tissue deformation minimization
          c += stiffnessPerUnitLength * (pow(LsegMax, j + k + 1) - pow(LsegMin, j + k + 1)) / (j + k + 1);

          L.push_back(T(line + j, startIndex + k, c));
        }
      }

      double l = LsegMin;
      while (l < LsegMax) {
        usPolynomialCurve3D p(m_tissue.accessPath().accessSegment(restIndex).getSubPolynomialCurve(
            currentRestParam,
            ((restIndex == m_tissue.accessPath().getNbSegments() - 1) ? currentRestParam : 0) +
                m_tissue.accessPath().accessSegment(restIndex).getParametricLength()));
        p.changeCoefficientsToFitBoundaries(l, l + m_restDilatationFactor.at(i - 1) * p.getParametricLength());
        int nbRestSegCoef = p.getOrder() + 1;

        double Lmin = l;
        double Lmax = 0;
        if ((LsegMax - l) < p.getParametricLength() || restIndex == m_tissue.accessPath().getNbSegments() - 1) {
          Lmax = LsegMax;
          currentRestParam += (LsegMax - l) / m_restDilatationFactor.at(i - 1);
          l = LsegMax;
        } else {
          Lmax = l + p.getParametricLength();
          l = Lmax;
          currentRestParam = 0;
          restIndex++;
        }

        vpMatrix coefP = p.getPolynomialCoefficients();
        for (int j = 0; j < nbSegEq; j++) {
          for (int k = 0; k < nbRestSegCoef; k++) {
            double c = stiffnessPerUnitLength * (pow(Lmax, j + k + 1) - pow(Lmin, j + k + 1)) / (j + k + 1);
            for (int dim = 0; dim < 3; dim++) {
              B(line + j, dim) += c * coefP[dim][k];
            }
          }
        }
      }

      if (LsegMax < segLength) {
        layerIndex++;
        nextLayerDepth += m_layerLength.at(layerIndex);
        lseg += LsegMax - LsegMin;
      } else
        lseg = segLength;

      currentDepth += LsegMax - LsegMin;
      LsegMin = LsegMax;
      LsegMax = segLength;
    }

    line += nbSegEq;
    startIndex += nbSegCoef;
  }

  Eigen::SparseMatrix<double> M(nbConstraints + nbcoef, nbConstraints + nbcoef);
  M.setFromTriplets(L.begin(), L.end());

  Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
  solver.compute(M);

  A = solver.solve(B);

  // Update coefficients
  startIndex = 0;
  for (int i = 0; i < nbSegments; i++) {
    int order = m_needle.accessSegment(i).getOrder();
    vpMatrix m(3, order + 1);
    for (int j = 0; j < order + 1; j++) {
      for (int dim = 0; dim < 3; dim++) {
        m[dim][j] = A(startIndex + j, dim);
      }
    }
    m_needle.accessSegment(i).setPolynomialCoefficients(m);
    startIndex += order + 1;
  }
#else
  throw vpException(
      vpException::functionNotImplementedError,
      "usNeedleInsertionModelRayleighRitzSpline::solveSegmentsParametersSparseEigen: not implemented without Eigen3");
#endif
}

void usNeedleInsertionModelRayleighRitzSpline::solveSegmentsParametersFullSparseEigen()
{
#ifdef VISP_HAVE_EIGEN3
  if (m_tissue.accessPath().getNbSegments() == 0) // no path present
  {
    usPolynomialCurve3D seg(m_needle.accessSegment(0));
    vpMatrix M(3, seg.getOrder() + 1, 0);
    for (int dim = 0; dim < 3; dim++) {
      M[dim][0] = m_needle.getBasePose()[dim];
      M[dim][1] = m_needle.getWorldMbase()[dim][2];
    }
    seg.setPolynomialCoefficients(M);
    seg.setParametricLength(m_needle.getFullLength());
    m_needle.init();
    m_needle.setSegment(0, seg);

    return;
  }

  double trueFreeLength = 0;

  if (usGeometryTools::DoesSegmentCrossPlane(m_needle, m_tissue.accessSurface())) {
    double freeLength = -1;
    usGeometryTools::getPlaneCurveCrossingPoint(m_needle, m_tissue.accessSurface(), -1, &freeLength);
    int seg = -1;
    double param = 0;
    m_needle.getParametersFromLength(freeLength, seg, param);

    for (int i = 0; i < seg; i++) {
      trueFreeLength += m_needle.accessSegment(i).getLength();
    }
    trueFreeLength += m_needle.accessSegment(seg).getSubPolynomialCurve(0, param).getLength();
  } else {
    vpColVector p(3);
    for (int i = 0; i < 3; i++)
      p[i] = m_needle.getBasePose()[i];
    vpColVector d(3);
    for (int i = 0; i < 3; i++)
      d[i] = m_needle.getWorldMbase()[i][2];
    double cosTheta = fabs(vpColVector::dotProd(m_tissue.accessSurface().getDirection(), d));
    if (cosTheta > std::numeric_limits<double>::epsilon())
      trueFreeLength = fabs(usGeometryTools::getPointPlaneDistance(p, m_tissue.accessSurface())) / cosTheta;
    else
      trueFreeLength = m_needle.getFullLength();
  }

  bool tipIn = usGeometryTools::IsPointInFrontOfPlane(m_needleTip->getTipPosition(), m_tissue.accessSurface());
  if (trueFreeLength >= m_needle.getFullLength() && !tipIn) // needle is not inserted
  {
    usPolynomialCurve3D seg(m_needle.accessSegment(0));
    vpMatrix M(3, seg.getOrder() + 1, 0);
    for (int dim = 0; dim < 3; dim++) {
      M[dim][0] = m_needle.getBasePose()[dim];
      M[dim][1] = m_needle.getWorldMbase()[dim][2];
    }
    seg.setPolynomialCoefficients(M);
    seg.setParametricLength(m_needle.getFullLength());
    m_needle.init();
    m_needle.setSegment(0, seg);

    return;
  }

  m_needle.accessSegment(0).setParametricLength(trueFreeLength);

  double segDefaultLength = 0.01;
  int nbSegments = 1 + floor((m_needle.getFullLength() - trueFreeLength) / segDefaultLength) + 1;

  if (trueFreeLength >= m_needle.getFullLength())
    nbSegments = 1;

  while (m_needle.getNbSegments() > nbSegments) {
    m_needle.removeLastSegment();
  }
  while (m_needle.getNbSegments() < nbSegments) {
    m_needle.addSegment(m_needle.accessLastSegment());
  }
  m_restDilatationFactor.resize(m_needle.getNbSegments(), 1);
  for (int i = 1; i < nbSegments - 1; i++)
    m_needle.accessSegment(i).setParametricLength(segDefaultLength);
  if (nbSegments > 1)
    m_needle.accessLastSegment().setParametricLength(m_needle.getFullLength() -
                                                     m_needle.accessSegment(0).getParametricLength() -
                                                     segDefaultLength * (nbSegments - 2));

  int nbcoef = 0;
  for (int i = 0; i < nbSegments; i++)
    nbcoef += m_needle.accessSegment(i).getOrder() + 1;

  typedef Eigen::Triplet<double> T;
  std::vector<T> L;

  double EI = m_needle.getEI();

  int nbConstraints = 2;
  for (int i = 0; i < nbSegments - 1; i++) {
    int order1 = m_needle.accessSegment(i).getOrder();
    int order2 = m_needle.accessSegment(i + 1).getOrder();
    int order = std::max(order1, order2);
    nbConstraints += std::min(3, order);
  }

  int nbMatrixLine = 3 * (nbConstraints + nbcoef);

  L.reserve(nbMatrixLine * nbMatrixLine / 10);

  Eigen::VectorXd A = Eigen::VectorXd::Zero(nbMatrixLine);
  Eigen::VectorXd B = Eigen::VectorXd::Zero(nbMatrixLine);

  // Constraints matrix

  // Base conditions
  int line = 3 * nbcoef;
  for (int dim = 0; dim < 3; dim++) {
    B(line) = m_needle.getBasePose()[dim];
    L.push_back(T(line, dim, 1));
    L.push_back(T(dim, line, -1));
    line++;
  }
  for (int dim = 0; dim < 3; dim++) {
    B(line) = m_needle.getWorldMbase()[dim][2];
    L.push_back(T(line, 3 + dim, 1));
    L.push_back(T(3 + dim, line, -1));
    line++;
  }

  int nbSegCoef = 0;
  int startIndex = 0;
  for (int i = 0; i < nbSegments - 1; i++) {
    nbSegCoef = m_needle.accessSegment(i).getOrder() + 1;
    int order1 = m_needle.accessSegment(i).getOrder();
    int order2 = m_needle.accessSegment(i + 1).getOrder();
    int order = std::max(order1, order2);
    double segLength = m_needle.accessSegment(i).getParametricLength();

    // Continuity

    double *tmp = new double[nbSegCoef];
    if (order > 0) // Order 0
    {
      tmp[0] = 1;
      for (int j = 1; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 0; j < nbSegCoef; j++) {
          L.push_back(T(line, startIndex + 3 * j + dim, tmp[j]));
          L.push_back(T(startIndex + 3 * j + dim, line, -tmp[j]));
        }
        L.push_back(T(line, startIndex + 3 * nbSegCoef + dim, -1));
        L.push_back(T(startIndex + 3 * nbSegCoef + dim, line, 1));
        line++;
      }
    }
    if (order > 1) // Order 1
    {
      tmp[0] = 0;
      tmp[1] = 1;
      for (int j = 2; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];
      for (int j = 1; j < nbSegCoef; j++)
        tmp[j] *= j;

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 1; j < nbSegCoef; j++) {
          L.push_back(T(line, startIndex + 3 * j + dim, tmp[j]));
          L.push_back(T(startIndex + 3 * j + dim, line, -tmp[j]));
        }
        L.push_back(T(line, startIndex + 3 * (nbSegCoef + 1) + dim, -1));
        L.push_back(T(startIndex + 3 * (nbSegCoef + 1) + dim, line, 1));
        line++;
      }
    }
    if (order > 2) // Order 2
    {
      tmp[0] = 0;
      tmp[1] = 0;
      tmp[2] = 1;
      for (int j = 3; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];
      for (int j = 2; j < nbSegCoef; j++)
        tmp[j] *= j * (j - 1);

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 2; j < nbSegCoef; j++) {
          L.push_back(T(line, startIndex + 3 * j + dim, tmp[j]));
          L.push_back(T(startIndex + 3 * j + dim, line, -tmp[j]));
        }
        L.push_back(T(line, startIndex + 3 * (nbSegCoef + 2) + dim, -2));
        L.push_back(T(startIndex + 3 * (nbSegCoef + 2) + dim, line, 2));
        line++;
      }
    }
    delete[] tmp;
    startIndex += 3 * nbSegCoef;
  }

  // Optimization matrix

  line = 0;

  nbSegCoef = m_needle.accessSegment(0).getOrder() + 1;
  int nbSegEq = nbSegCoef;
  double segLength = m_needle.accessSegment(0).getParametricLength();

  for (int j = 0; j < nbSegEq; j++) {
    // Bending minimization
    for (int k = 2; k < nbSegCoef; k++) {
      if (j > 1)
        for (int dim = 0; dim < 3; dim++)
          L.push_back(
              T(line + dim, 3 * k + dim, EI * pow(segLength, j + k - 3) * j * (j - 1) * k * (k - 1) / (j + k - 3)));
    }
    line += 3;
  }

  startIndex = 3 * nbSegCoef;
  int restIndex = 0;
  double currentRestParam = 0;
  int layerIndex = 0;
  double currentDepth = 0;
  double nextLayerDepth = m_layerLength.front();
  for (int i = 1; i < nbSegments; i++) {
    nbSegCoef = m_needle.accessSegment(i).getOrder() + 1;
    nbSegEq = nbSegCoef;
    segLength = m_needle.accessSegment(i).getParametricLength();

    double lseg = 0;
    double LsegMin = 0;
    double LsegMax = segLength;
    while (lseg < segLength) {
      double stiffnessPerUnitLength = m_stiffnessPerUnitLength.at(layerIndex);

      if (currentDepth - lseg + segLength > nextLayerDepth && layerIndex + 1 < (int)m_layerLength.size()) {
        LsegMax = nextLayerDepth - currentDepth;
      }

      for (int j = 0; j < nbSegEq; j++) {
        for (int k = 0; k < nbSegCoef; k++) {
          double c = 0;
          // Bending minimization
          if (j > 1 && k > 1)
            c += EI * (pow(LsegMax, j + k - 3) - pow(LsegMin, j + k - 3)) * j * (j - 1) * k * (k - 1) / (j + k - 3);
          // Tissue deformation minimization
          c += stiffnessPerUnitLength * (pow(LsegMax, j + k + 1) - pow(LsegMin, j + k + 1)) / (j + k + 1);

          for (int dim = 0; dim < 3; dim++)
            L.push_back(T(line + 3 * j + dim, startIndex + 3 * k + dim, c));
        }
      }

      double l = LsegMin;
      while (l < LsegMax) {
        usPolynomialCurve3D p(m_tissue.accessPath().accessSegment(restIndex).getSubPolynomialCurve(
            currentRestParam,
            ((restIndex == m_tissue.accessPath().getNbSegments() - 1) ? currentRestParam : 0) +
                m_tissue.accessPath().accessSegment(restIndex).getParametricLength()));
        p.changeCoefficientsToFitBoundaries(l, l + m_restDilatationFactor.at(i) * p.getParametricLength());
        int nbRestSegCoef = p.getOrder() + 1;

        double Lmin = l;
        double Lmax = 0;
        if ((LsegMax - l) < p.getParametricLength() || restIndex == m_tissue.accessPath().getNbSegments() - 1) {
          Lmax = LsegMax;
          currentRestParam += (LsegMax - l) / m_restDilatationFactor.at(i);
          l = LsegMax;
        } else {
          Lmax = l + p.getParametricLength();
          l = Lmax;
          currentRestParam = 0;
          restIndex++;
        }

        vpMatrix coefP = p.getPolynomialCoefficients();
        for (int j = 0; j < nbSegEq; j++) {
          for (int k = 0; k < nbRestSegCoef; k++) {
            double c = stiffnessPerUnitLength * (pow(Lmax, j + k + 1) - pow(Lmin, j + k + 1)) / (j + k + 1);
            for (int dim = 0; dim < 3; dim++) {
              B(line + 3 * j + dim) += c * coefP[dim][k];
            }
          }
        }
      }

      if (LsegMax < segLength) {
        layerIndex++;
        nextLayerDepth += m_layerLength.at(layerIndex);
        lseg += LsegMax - LsegMin;
      } else
        lseg = segLength;

      currentDepth += LsegMax - LsegMin;
      LsegMin = LsegMax;
      LsegMax = segLength;
    }

    line += 3 * nbSegEq;
    startIndex += 3 * nbSegCoef;
  }

  line -= 3 * (m_needle.accessLastSegment().getOrder() + 1);
  startIndex -= 3 * (m_needle.accessLastSegment().getOrder() + 1);

  // Bevel
  {
    vpMatrix I;
    I.eye(3);
    vpRotationMatrix R(0, 0, 0);

    vpColVector bevelSegment = m_needleTip->getTipPosition() - m_needleTip->getBasePosition();
    double bevLength = bevelSegment.euclideanNorm();
    bevelSegment.normalize();
    vpColVector p = vpColVector::crossProd(m_needleTip->getBaseAxisZ(), bevelSegment);
    R.buildFrom(p[0], p[1], p[2]);

    double segLength = m_needle.accessLastSegment().getParametricLength();
    double LbevMin = 0;
    if (currentDepth == 0) {
      double cosTheta = fabs(vpColVector::dotProd(m_tissue.accessSurface().getDirection(), bevelSegment));
      if (cosTheta > std::numeric_limits<double>::epsilon())
        LbevMin =
            fabs(usGeometryTools::getPointPlaneDistance(m_needleTip->getBasePosition(), m_tissue.accessSurface())) /
            cosTheta;
      else
        LbevMin = bevLength;
    }
    double lbev = LbevMin;
    double LbevMax = bevLength;
    while (lbev < bevLength) {
      double stiffnessPerUnitLength = m_stiffnessPerUnitLength.at(layerIndex);

      if (currentDepth - lbev + bevLength > nextLayerDepth && layerIndex + 1 < (int)m_layerLength.size()) {
        LbevMax = nextLayerDepth - currentDepth;
      }

      for (int j = 0; j < nbSegEq; j++) {
        for (int k = 0; k < nbSegCoef; k++) {
          double c0 = 0;
          double c1 = 0;
          double c2 = 0;
          double c3 = 0;
          // Tissue deformation minimization
          c0 = pow(segLength, j + k) * (LbevMax - LbevMin);
          if (j > 0)
            c1 = j * pow(segLength, j + k - 1) * (pow(LbevMax, 2) - pow(LbevMin, 2)) / 2;
          if (k > 0)
            c2 = k * pow(segLength, j + k - 1) * (pow(LbevMax, 2) - pow(LbevMin, 2)) / 2;
          if (j + k > 1)
            c3 = (j * k) * pow(segLength, j + k - 2) * (pow(LbevMax, 3) - pow(LbevMin, 3)) / 3;

          vpMatrix coefMat = stiffnessPerUnitLength * (c0 * I + c1 * R.t() + c2 * R + c3 * R.t() * R);

          for (int dimj = 0; dimj < 3; dimj++)
            for (int dimk = 0; dimk < 3; dimk++)
              L.push_back(T(line + 3 * j + dimj, startIndex + 3 * k + dimk, coefMat[dimj][dimk]));
        }
      }

      double l = LbevMin;
      while (l < LbevMax) {
        usPolynomialCurve3D p(m_tissue.accessPath().accessSegment(restIndex).getSubPolynomialCurve(
            currentRestParam,
            ((restIndex == m_tissue.accessPath().getNbSegments() - 1) ? currentRestParam : 0) +
                m_tissue.accessPath().accessSegment(restIndex).getParametricLength()));
        p.changeCoefficientsToFitBoundaries(l, l + p.getParametricLength());
        int nbRestSegCoef = p.getOrder() + 1;

        double Lmin = l;
        double Lmax = 0;
        if ((LbevMax - l) < p.getParametricLength() || restIndex == m_tissue.accessPath().getNbSegments() - 1) {
          Lmax = LbevMax;
          currentRestParam += LbevMax - l;
          l = LbevMax;
        } else {
          Lmax = l + p.getParametricLength();
          l = Lmax;
          currentRestParam = 0;
          restIndex++;
        }

        vpMatrix coefP = p.getPolynomialCoefficients();
        for (int j = 0; j < nbSegEq; j++) {
          vpColVector b0(3, 0);
          vpColVector b1(3, 0);
          for (int k = 0; k < nbRestSegCoef; k++) {
            b0 += pow(segLength, j) * (pow(Lmax, k + 1) - pow(Lmin, k + 1)) / (k + 1) * coefP.getCol(k);
          }
          if (j > 0)
            for (int k = 0; k < nbRestSegCoef; k++) {
              b1 += j * pow(segLength, j - 1) * (pow(Lmax, k + 2) - pow(Lmin, k + 2)) / (k + 2) * coefP.getCol(k);
            }
          vpColVector coef = stiffnessPerUnitLength * (b0 + R.t() * b1);

          for (int dim = 0; dim < 3; dim++)
            B(line + 3 * j + dim) += coef[dim];
        }
      }

      if (LbevMax < bevLength) {
        layerIndex++;
        nextLayerDepth += m_layerLength.at(layerIndex);
        lbev += LbevMax - LbevMin;
      } else
        lbev = bevLength;

      currentDepth += LbevMax - LbevMin;
      LbevMin = LbevMax;
      LbevMax = bevLength;
    }
  }

  Eigen::SparseMatrix<double> M(nbMatrixLine, nbMatrixLine);
  M.setFromTriplets(L.begin(), L.end());

  Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
  solver.compute(M);

  A = solver.solve(B);

  // Update coefficients
  startIndex = 0;
  for (int i = 0; i < nbSegments; i++) {
    int order = m_needle.accessSegment(i).getOrder();
    vpMatrix m(3, order + 1);
    for (int j = 0; j < order + 1; j++) {
      for (int dim = 0; dim < 3; dim++) {
        m[dim][j] = A(startIndex + 3 * j + dim);
      }
    }
    m_needle.accessSegment(i).setPolynomialCoefficients(m);
    startIndex += 3 * (order + 1);
  }
#else
  throw vpException(vpException::functionNotImplementedError, "usNeedleInsertionModelRayleighRitzSpline::"
                                                              "solveSegmentsParametersFullSparseEigen: not implemented "
                                                              "without Eigen3");
#endif
}

void usNeedleInsertionModelRayleighRitzSpline::solveSegmentsParametersFullSparseEigenFixedLength()
{
#ifdef VISP_HAVE_EIGEN3
  if (m_tissue.accessPath().getNbSegments() == 0 ||
      m_needle.getFullLength() <= std::numeric_limits<double>::epsilon()) // no path present
  {
    usPolynomialCurve3D seg(m_needle.accessSegment(0));
    vpMatrix M(3, seg.getOrder() + 1, 0);
    for (int dim = 0; dim < 3; dim++) {
      M[dim][0] = m_needle.getBasePose()[dim];
      M[dim][1] = m_needle.getWorldMbase()[dim][2];
    }
    seg.setPolynomialCoefficients(M);
    seg.setParametricLength(m_needle.getFullLength());
    m_needle.init();
    m_needle.setSegment(0, seg);

    return;
  }

  double trueFreeLength = 0;

  if (usGeometryTools::DoesSegmentCrossPlane(m_needle, m_tissue.accessSurface())) {
    double freeLength = -1;
    usGeometryTools::getPlaneCurveCrossingPoint(m_needle, m_tissue.accessSurface(), -1, &freeLength);
    int seg = -1;
    double param = 0;
    m_needle.getParametersFromLength(freeLength, seg, param);

    for (int i = 0; i < seg; i++) {
      trueFreeLength += m_needle.accessSegment(i).getLength();
    }
    trueFreeLength += m_needle.accessSegment(seg).getSubPolynomialCurve(0, param).getLength();
  } else {
    vpColVector p(3);
    for (int i = 0; i < 3; i++)
      p[i] = m_needle.getBasePose()[i];
    vpColVector d(3);
    for (int i = 0; i < 3; i++)
      d[i] = m_needle.getWorldMbase()[i][2];
    double cosTheta = fabs(vpColVector::dotProd(m_tissue.accessSurface().getDirection(), d));
    if (cosTheta > std::numeric_limits<double>::epsilon())
      trueFreeLength = fabs(usGeometryTools::getPointPlaneDistance(p, m_tissue.accessSurface())) / cosTheta;
    else
      trueFreeLength = m_needle.getFullLength();
  }

  bool tipIn = usGeometryTools::IsPointInFrontOfPlane(m_needleTip->getTipPosition(), m_tissue.accessSurface());
  if (trueFreeLength >= m_needle.getFullLength() && !tipIn) // needle is not inserted
  {
    usPolynomialCurve3D seg(m_needle.accessSegment(0));
    vpMatrix M(3, seg.getOrder() + 1, 0);
    for (int dim = 0; dim < 3; dim++) {
      M[dim][0] = m_needle.getBasePose()[dim];
      M[dim][1] = m_needle.getWorldMbase()[dim][2];
    }
    seg.setPolynomialCoefficients(M);
    seg.setParametricLength(m_needle.getFullLength());
    m_needle.init();
    m_needle.setSegment(0, seg);

    return;
  }

  double segDefaultLength = 0.01;
  int nbSegments = 1 + floor(m_needle.getFullLength() / segDefaultLength);

  while (m_needle.getNbSegments() > nbSegments) {
    m_needle.removeLastSegment();
  }
  while (m_needle.getNbSegments() < nbSegments) {
    m_needle.addSegment(m_needle.accessLastSegment());
  }
  m_restDilatationFactor.resize(m_needle.getNbSegments(), 1);
  for (int i = 0; i < nbSegments - 1; i++)
    m_needle.accessSegment(i).setParametricLength(segDefaultLength);
  if (nbSegments > 1) {
    double l = m_needle.getFullLength() - segDefaultLength * (nbSegments - 1);
    if (l > std::numeric_limits<double>::epsilon())
      m_needle.accessLastSegment().setParametricLength(l);
    else {
      m_needle.removeLastSegment();
      nbSegments--;
    }
  }
  int nbcoef = 0;
  for (int i = 0; i < nbSegments; i++)
    nbcoef += m_needle.accessSegment(i).getOrder() + 1;

  typedef Eigen::Triplet<double> T;
  std::vector<T> L;

  double EI = m_needle.getEI();

  int nbConstraints = 2;
  for (int i = 0; i < nbSegments - 1; i++) {
    int order1 = m_needle.accessSegment(i).getOrder();
    int order2 = m_needle.accessSegment(i + 1).getOrder();
    int order = std::max(order1, order2);
    nbConstraints += std::min(3, order);
  }

  int nbMatrixLine = 3 * (nbConstraints + nbcoef);

  L.reserve(nbMatrixLine * nbMatrixLine / 10);

  Eigen::VectorXd A = Eigen::VectorXd::Zero(nbMatrixLine);
  Eigen::VectorXd B = Eigen::VectorXd::Zero(nbMatrixLine);

  // Constraints matrix

  // Base conditions
  int line = 3 * nbcoef;
  for (int dim = 0; dim < 3; dim++) {
    B(line) = m_needle.getBasePose()[dim];
    L.push_back(T(line, dim, 1));
    L.push_back(T(dim, line, -1));
    line++;
  }
  for (int dim = 0; dim < 3; dim++) {
    B(line) = m_needle.getWorldMbase()[dim][2];
    L.push_back(T(line, 3 + dim, 1));
    L.push_back(T(3 + dim, line, -1));
    line++;
  }

  int nbSegCoef = 0;
  int startIndex = 0;
  for (int i = 0; i < nbSegments - 1; i++) {
    nbSegCoef = m_needle.accessSegment(i).getOrder() + 1;
    int order1 = m_needle.accessSegment(i).getOrder();
    int order2 = m_needle.accessSegment(i + 1).getOrder();
    int order = std::max(order1, order2);
    double segLength = m_needle.accessSegment(i).getParametricLength();

    // Continuity

    double *tmp = new double[nbSegCoef];
    if (order > 0) // Order 0
    {
      tmp[0] = 1;
      for (int j = 1; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 0; j < nbSegCoef; j++) {
          L.push_back(T(line, startIndex + 3 * j + dim, tmp[j]));
          L.push_back(T(startIndex + 3 * j + dim, line, -tmp[j]));
        }
        L.push_back(T(line, startIndex + 3 * nbSegCoef + dim, -1));
        L.push_back(T(startIndex + 3 * nbSegCoef + dim, line, 1));
        line++;
      }
    }
    if (order > 1) // Order 1
    {
      tmp[0] = 0;
      tmp[1] = 1;
      for (int j = 2; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];
      for (int j = 1; j < nbSegCoef; j++)
        tmp[j] *= j;

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 1; j < nbSegCoef; j++) {
          L.push_back(T(line, startIndex + 3 * j + dim, tmp[j]));
          L.push_back(T(startIndex + 3 * j + dim, line, -tmp[j]));
        }
        L.push_back(T(line, startIndex + 3 * (nbSegCoef + 1) + dim, -1));
        L.push_back(T(startIndex + 3 * (nbSegCoef + 1) + dim, line, 1));
        line++;
      }
    }
    if (order > 2) // Order 2
    {
      tmp[0] = 0;
      tmp[1] = 0;
      tmp[2] = 1;
      for (int j = 3; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];
      for (int j = 2; j < nbSegCoef; j++)
        tmp[j] *= j * (j - 1);

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 2; j < nbSegCoef; j++) {
          L.push_back(T(line, startIndex + 3 * j + dim, tmp[j]));
          L.push_back(T(startIndex + 3 * j + dim, line, -tmp[j]));
        }
        L.push_back(T(line, startIndex + 3 * (nbSegCoef + 2) + dim, -2));
        L.push_back(T(startIndex + 3 * (nbSegCoef + 2) + dim, line, 2));
        line++;
      }
    }
    delete[] tmp;
    startIndex += 3 * nbSegCoef;
  }

  // Optimization matrix

  line = 0;

  nbSegCoef = m_needle.accessSegment(0).getOrder() + 1;
  int nbSegEq = nbSegCoef;
  double segLength = m_needle.accessSegment(0).getParametricLength();

  startIndex = 0;
  int restIndex = 0;
  double currentRestParam = 0;
  int layerIndex = -1;
  double currentDepth = -trueFreeLength;
  double nextLayerDepth = 0;
  for (int i = 0; i < nbSegments; i++) {
    nbSegCoef = m_needle.accessSegment(i).getOrder() + 1;
    nbSegEq = nbSegCoef;
    segLength = m_needle.accessSegment(i).getParametricLength();

    double lseg = 0;
    double LsegMin = 0;
    double LsegMax = segLength;
    while (lseg < segLength) {
      double stiffnessPerUnitLength = 0;
      if (layerIndex >= 0)
        stiffnessPerUnitLength = m_stiffnessPerUnitLength.at(layerIndex);

      if (currentDepth - lseg + segLength > nextLayerDepth && layerIndex + 1 < (int)m_layerLength.size()) {
        LsegMax = nextLayerDepth - currentDepth;
      }

      for (int j = 0; j < nbSegEq; j++) {
        for (int k = 0; k < nbSegCoef; k++) {
          double c = 0;
          // Bending minimization
          if (j > 1 && k > 1)
            c += EI * (pow(LsegMax, j + k - 3) - pow(LsegMin, j + k - 3)) * j * (j - 1) * k * (k - 1) / (j + k - 3);
          // if(i==0) c*=100;
          // Tissue deformation minimization
          c += stiffnessPerUnitLength * (pow(LsegMax, j + k + 1) - pow(LsegMin, j + k + 1)) / (j + k + 1);

          for (int dim = 0; dim < 3; dim++)
            L.push_back(T(line + 3 * j + dim, startIndex + 3 * k + dim, c));
        }
      }

      if (layerIndex >= 0) {
        double l = LsegMin;
        double factor = m_restDilatationFactor.at(i);
        while (l < LsegMax) {
          usPolynomialCurve3D p(m_tissue.accessPath().accessSegment(restIndex).getSubPolynomialCurve(
              currentRestParam,
              ((restIndex == m_tissue.accessPath().getNbSegments() - 1) ? currentRestParam : 0) +
                  m_tissue.accessPath().accessSegment(restIndex).getParametricLength()));
          p.changeCoefficientsToFitBoundaries(l, l + factor * p.getParametricLength());
          int nbRestSegCoef = p.getOrder() + 1;

          double Lmin = l;
          double Lmax = 0;
          if ((LsegMax - l) < p.getParametricLength() || restIndex == m_tissue.accessPath().getNbSegments() - 1) {
            Lmax = LsegMax;
            currentRestParam += (LsegMax - l) / factor;
            l = LsegMax;
          } else {
            Lmax = l + p.getParametricLength();
            l = Lmax;
            currentRestParam = 0;
            restIndex++;
          }

          vpMatrix coefP = p.getPolynomialCoefficients();
          for (int j = 0; j < nbSegEq; j++) {
            for (int k = 0; k < nbRestSegCoef; k++) {
              double c = stiffnessPerUnitLength * (pow(Lmax, j + k + 1) - pow(Lmin, j + k + 1)) / (j + k + 1);
              for (int dim = 0; dim < 3; dim++) {
                B(line + 3 * j + dim) += c * coefP[dim][k];
              }
            }
          }
        }
      }

      if (LsegMax < segLength) {
        layerIndex++;
        nextLayerDepth += m_layerLength.at(layerIndex);
        lseg += LsegMax - LsegMin;
      } else
        lseg = segLength;

      currentDepth += LsegMax - LsegMin;
      LsegMin = LsegMax;
      LsegMax = segLength;
    }

    line += 3 * nbSegEq;
    startIndex += 3 * nbSegCoef;
  }

  line -= 3 * (m_needle.accessLastSegment().getOrder() + 1);
  startIndex -= 3 * (m_needle.accessLastSegment().getOrder() + 1);

  // Bevel
  {
    if (layerIndex < 0) {
      layerIndex = 0;
      currentDepth = 0;
    }
    vpMatrix I;
    I.eye(3);
    vpRotationMatrix R(0, 0, 0);

    vpColVector bevelSegment = m_needleTip->getTipPosition() - m_needleTip->getBasePosition();
    double bevLength = bevelSegment.euclideanNorm();
    bevelSegment.normalize();
    vpColVector p = vpColVector::crossProd(m_needleTip->getBaseAxisZ(), bevelSegment);
    R.buildFrom(p[0], p[1], p[2]);

    double segLength = m_needle.accessLastSegment().getParametricLength();
    double LbevMin = 0;
    if (currentDepth == 0) {
      double cosTheta = fabs(vpColVector::dotProd(m_tissue.accessSurface().getDirection(), bevelSegment));
      if (cosTheta > std::numeric_limits<double>::epsilon())
        LbevMin =
            fabs(usGeometryTools::getPointPlaneDistance(m_needleTip->getBasePosition(), m_tissue.accessSurface())) /
            cosTheta;
      else
        LbevMin = bevLength;
    }
    double lbev = LbevMin;
    double LbevMax = bevLength;
    while (lbev < bevLength) {
      double stiffnessPerUnitLength = m_stiffnessPerUnitLength.at(layerIndex);

      if (currentDepth - lbev + bevLength > nextLayerDepth && layerIndex + 1 < (int)m_layerLength.size()) {
        LbevMax = nextLayerDepth - currentDepth;
      }

      for (int j = 0; j < nbSegEq; j++) {
        for (int k = 0; k < nbSegCoef; k++) {
          double c0 = 0;
          double c1 = 0;
          double c2 = 0;
          double c3 = 0;
          // Tissue deformation minimization
          c0 = pow(segLength, j + k) * (LbevMax - LbevMin);
          if (j > 0)
            c1 = j * pow(segLength, j + k - 1) * (pow(LbevMax, 2) - pow(LbevMin, 2)) / 2;
          if (k > 0)
            c2 = k * pow(segLength, j + k - 1) * (pow(LbevMax, 2) - pow(LbevMin, 2)) / 2;
          if (j + k > 1)
            c3 = (j * k) * pow(segLength, j + k - 2) * (pow(LbevMax, 3) - pow(LbevMin, 3)) / 3;

          vpMatrix coefMat = stiffnessPerUnitLength * (c0 * I + c1 * R.t() + c2 * R + c3 * R.t() * R);

          for (int dimj = 0; dimj < 3; dimj++)
            for (int dimk = 0; dimk < 3; dimk++)
              L.push_back(T(line + 3 * j + dimj, startIndex + 3 * k + dimk, coefMat[dimj][dimk]));
        }
      }

      double l = LbevMin;
      while (l < LbevMax) {
        usPolynomialCurve3D p(m_tissue.accessPath().accessSegment(restIndex).getSubPolynomialCurve(
            currentRestParam,
            ((restIndex == m_tissue.accessPath().getNbSegments() - 1) ? currentRestParam : 0) +
                m_tissue.accessPath().accessSegment(restIndex).getParametricLength()));
        p.changeCoefficientsToFitBoundaries(l, l + p.getParametricLength());
        int nbRestSegCoef = p.getOrder() + 1;

        double Lmin = l;
        double Lmax = 0;
        if ((LbevMax - l) < p.getParametricLength() || restIndex == m_tissue.accessPath().getNbSegments() - 1) {
          Lmax = LbevMax;
          currentRestParam += LbevMax - l;
          l = LbevMax;
        } else {
          Lmax = l + p.getParametricLength();
          l = Lmax;
          currentRestParam = 0;
          restIndex++;
        }

        vpMatrix coefP = p.getPolynomialCoefficients();
        for (int j = 0; j < nbSegEq; j++) {
          vpColVector b0(3, 0);
          vpColVector b1(3, 0);
          for (int k = 0; k < nbRestSegCoef; k++) {
            b0 += pow(segLength, j) * (pow(Lmax, k + 1) - pow(Lmin, k + 1)) / (k + 1) * coefP.getCol(k);
          }
          if (j > 0)
            for (int k = 0; k < nbRestSegCoef; k++) {
              b1 += j * pow(segLength, j - 1) * (pow(Lmax, k + 2) - pow(Lmin, k + 2)) / (k + 2) * coefP.getCol(k);
            }
          vpColVector coef = stiffnessPerUnitLength * (b0 + R.t() * b1);

          for (int dim = 0; dim < 3; dim++)
            B(line + 3 * j + dim) += coef[dim];
        }
      }

      if (LbevMax < bevLength) {
        layerIndex++;
        nextLayerDepth += m_layerLength.at(layerIndex);
        lbev += LbevMax - LbevMin;
      } else
        lbev = bevLength;

      currentDepth += LbevMax - LbevMin;
      LbevMin = LbevMax;
      LbevMax = bevLength;
    }
  }

  Eigen::SparseMatrix<double> M(nbMatrixLine, nbMatrixLine);
  M.setFromTriplets(L.begin(), L.end());

  Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
  solver.compute(M);

  A = solver.solve(B);

  // Update coefficients
  startIndex = 0;
  for (int i = 0; i < nbSegments; i++) {
    int order = m_needle.accessSegment(i).getOrder();
    vpMatrix m(3, order + 1);
    for (int j = 0; j < order + 1; j++) {
      for (int dim = 0; dim < 3; dim++) {
        m[dim][j] = A(startIndex + 3 * j + dim);
      }
    }
    m_needle.accessSegment(i).setPolynomialCoefficients(m);
    startIndex += 3 * (order + 1);
  }
#else
  throw vpException(vpException::functionNotImplementedError, "usNeedleInsertionModelRayleighRitzSpline::"
                                                              "solveSegmentsParametersFullSparseEigenFixedLength: not "
                                                              "implemented without Eigen3");
#endif
}

void usNeedleInsertionModelRayleighRitzSpline::solveSegmentsParametersDense()
{
  if (m_tissue.accessPath().getNbSegments() == 0 ||
      m_needle.getFullLength() <= std::numeric_limits<double>::epsilon()) // no path present
  {
    usPolynomialCurve3D seg(m_needle.accessSegment(0));
    vpMatrix M(3, seg.getOrder() + 1, 0);
    for (int dim = 0; dim < 3; dim++) {
      M[dim][0] = m_needle.getBasePose()[dim];
      M[dim][1] = m_needle.getWorldMbase()[dim][2];
    }
    seg.setPolynomialCoefficients(M);
    seg.setParametricLength(m_needle.getFullLength());
    m_needle.init();
    m_needle.setSegment(0, seg);

    return;
  }

  double trueFreeLength = 0;

  if (usGeometryTools::DoesSegmentCrossPlane(m_needle, m_tissue.accessSurface())) {
    double freeLength = -1;
    usGeometryTools::getPlaneCurveCrossingPoint(m_needle, m_tissue.accessSurface(), -1, &freeLength);
    int seg = -1;
    double param = 0;
    m_needle.getParametersFromLength(freeLength, seg, param);

    for (int i = 0; i < seg; i++) {
      trueFreeLength += m_needle.accessSegment(i).getLength();
    }
    trueFreeLength += m_needle.accessSegment(seg).getSubPolynomialCurve(0, param).getLength();
  } else {
    vpColVector p(3);
    for (int i = 0; i < 3; i++)
      p[i] = m_needle.getBasePose()[i];
    vpColVector d(3);
    for (int i = 0; i < 3; i++)
      d[i] = m_needle.getWorldMbase()[i][2];
    double cosTheta = fabs(vpColVector::dotProd(m_tissue.accessSurface().getDirection(), d));
    if (cosTheta > std::numeric_limits<double>::epsilon())
      trueFreeLength = fabs(usGeometryTools::getPointPlaneDistance(p, m_tissue.accessSurface())) / cosTheta;
    else
      trueFreeLength = m_needle.getFullLength();
  }

  bool tipIn = usGeometryTools::IsPointInFrontOfPlane(m_needleTip->getTipPosition(), m_tissue.accessSurface());
  if (trueFreeLength >= m_needle.getFullLength() && !tipIn) // needle is not inserted
  {
    usPolynomialCurve3D seg(m_needle.accessSegment(0));
    vpMatrix M(3, seg.getOrder() + 1, 0);
    for (int dim = 0; dim < 3; dim++) {
      M[dim][0] = m_needle.getBasePose()[dim];
      M[dim][1] = m_needle.getWorldMbase()[dim][2];
    }
    seg.setPolynomialCoefficients(M);
    seg.setParametricLength(m_needle.getFullLength());
    m_needle.init();
    m_needle.setSegment(0, seg);

    return;
  }

  double segDefaultLength = 0.01;
  int nbSegments = 1 + floor(m_needle.getFullLength() / segDefaultLength);

  while (m_needle.getNbSegments() > nbSegments) {
    m_needle.removeLastSegment();
  }
  while (m_needle.getNbSegments() < nbSegments) {
    m_needle.addSegment(m_needle.accessLastSegment());
  }
  m_restDilatationFactor.resize(m_needle.getNbSegments(), 1);
  for (int i = 0; i < nbSegments - 1; i++)
    m_needle.accessSegment(i).setParametricLength(segDefaultLength);
  if (nbSegments > 1) {
    double l = m_needle.getFullLength() - segDefaultLength * (nbSegments - 1);
    if (l > std::numeric_limits<double>::epsilon())
      m_needle.accessLastSegment().setParametricLength(l);
    else {
      m_needle.removeLastSegment();
      nbSegments--;
    }
  }
  int nbcoef = 0;
  for (int i = 0; i < nbSegments; i++)
    nbcoef += m_needle.accessSegment(i).getOrder() + 1;

  double EI = m_needle.getEI();

  int nbConstraints = 2;
  for (int i = 0; i < nbSegments - 1; i++) {
    int order1 = m_needle.accessSegment(i).getOrder();
    int order2 = m_needle.accessSegment(i + 1).getOrder();
    int order = std::max(order1, order2);
    nbConstraints += std::min(3, order);
  }

  int nbMatrixLine = 3 * (nbConstraints + nbcoef);

  vpMatrix M(nbMatrixLine, nbMatrixLine, 0);

  vpColVector A(nbMatrixLine, 0);
  vpColVector B(nbMatrixLine, 0);

  // Constraints matrix

  // Base conditions
  int line = 3 * nbcoef;
  for (int dim = 0; dim < 3; dim++) {
    B[line] = m_needle.getBasePose()[dim];
    M[line][dim] = 1;
    M[dim][line] = -1;
    line++;
  }
  for (int dim = 0; dim < 3; dim++) {
    B[line] = m_needle.getWorldMbase()[dim][2];
    M[line][3 + dim] = 1;
    M[3 + dim][line] = -1;
    line++;
  }

  int nbSegCoef = 0;
  int startIndex = 0;
  for (int i = 0; i < nbSegments - 1; i++) {
    nbSegCoef = m_needle.accessSegment(i).getOrder() + 1;
    int order1 = m_needle.accessSegment(i).getOrder();
    int order2 = m_needle.accessSegment(i + 1).getOrder();
    int order = std::max(order1, order2);
    double segLength = m_needle.accessSegment(i).getParametricLength();

    // Continuity

    double *tmp = new double[nbSegCoef];
    if (order > 0) // Order 0
    {
      tmp[0] = 1;
      for (int j = 1; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 0; j < nbSegCoef; j++) {
          M[line][startIndex + 3 * j + dim] = tmp[j];
          M[startIndex + 3 * j + dim][line] = -tmp[j];
        }
        M[line][startIndex + 3 * nbSegCoef + dim] = -1;
        M[startIndex + 3 * nbSegCoef + dim][line] = 1;
        line++;
      }
    }
    if (order > 1) // Order 1
    {
      tmp[0] = 0;
      tmp[1] = 1;
      for (int j = 2; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];
      for (int j = 1; j < nbSegCoef; j++)
        tmp[j] *= j;

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 1; j < nbSegCoef; j++) {
          M[line][startIndex + 3 * j + dim] = tmp[j];
          M[startIndex + 3 * j + dim][line] = -tmp[j];
        }
        M[line][startIndex + 3 * (nbSegCoef + 1) + dim] = -1;
        M[startIndex + 3 * (nbSegCoef + 1) + dim][line] = 1;
        line++;
      }
    }
    if (order > 2) // Order 2
    {
      tmp[0] = 0;
      tmp[1] = 0;
      tmp[2] = 1;
      for (int j = 3; j < nbSegCoef; j++)
        tmp[j] = segLength * tmp[j - 1];
      for (int j = 2; j < nbSegCoef; j++)
        tmp[j] *= j * (j - 1);

      for (int dim = 0; dim < 3; dim++) {
        for (int j = 2; j < nbSegCoef; j++) {
          M[line][startIndex + 3 * j + dim] = tmp[j];
          M[startIndex + 3 * j + dim][line] = -tmp[j];
        }
        M[line][startIndex + 3 * (nbSegCoef + 2) + dim] = -2;
        M[startIndex + 3 * (nbSegCoef + 2) + dim][line] = 2;
        line++;
      }
    }
    delete[] tmp;
    startIndex += 3 * nbSegCoef;
  }

  // Optimization matrix

  line = 0;

  nbSegCoef = m_needle.accessSegment(0).getOrder() + 1;
  int nbSegEq = nbSegCoef;
  double segLength = m_needle.accessSegment(0).getParametricLength();

  startIndex = 0;
  int restIndex = 0;
  double currentRestParam = 0;
  int layerIndex = -1;
  double currentDepth = -trueFreeLength;
  double nextLayerDepth = 0;
  for (int i = 0; i < nbSegments; i++) {
    nbSegCoef = m_needle.accessSegment(i).getOrder() + 1;
    nbSegEq = nbSegCoef;
    segLength = m_needle.accessSegment(i).getParametricLength();

    double lseg = 0;
    double LsegMin = 0;
    double LsegMax = segLength;
    while (lseg < segLength) {
      double stiffnessPerUnitLength = 0;
      if (layerIndex >= 0)
        stiffnessPerUnitLength = m_stiffnessPerUnitLength.at(layerIndex);

      if (currentDepth - lseg + segLength > nextLayerDepth && layerIndex + 1 < (int)m_layerLength.size()) {
        LsegMax = nextLayerDepth - currentDepth;
      }

      for (int j = 0; j < nbSegEq; j++) {
        for (int k = 0; k < nbSegCoef; k++) {
          double c = 0;
          // Bending minimization
          if (j > 1 && k > 1)
            c += EI * (pow(LsegMax, j + k - 3) - pow(LsegMin, j + k - 3)) * j * (j - 1) * k * (k - 1) / (j + k - 3);
          // if(i==0) c*=100;
          // Tissue deformation minimization
          c += stiffnessPerUnitLength * (pow(LsegMax, j + k + 1) - pow(LsegMin, j + k + 1)) / (j + k + 1);

          for (int dim = 0; dim < 3; dim++)
            M[line + 3 * j + dim][startIndex + 3 * k + dim] = c;
        }
      }

      if (layerIndex >= 0) {
        double l = LsegMin;
        double factor = m_restDilatationFactor.at(i);
        while (l < LsegMax) {
          usPolynomialCurve3D p(m_tissue.accessPath().accessSegment(restIndex).getSubPolynomialCurve(
              currentRestParam,
              ((restIndex == m_tissue.accessPath().getNbSegments() - 1) ? currentRestParam : 0) +
                  m_tissue.accessPath().accessSegment(restIndex).getParametricLength()));
          p.changeCoefficientsToFitBoundaries(l, l + factor * p.getParametricLength());
          int nbRestSegCoef = p.getOrder() + 1;

          double Lmin = l;
          double Lmax = 0;
          if ((LsegMax - l) < p.getParametricLength() || restIndex == m_tissue.accessPath().getNbSegments() - 1) {
            Lmax = LsegMax;
            currentRestParam += (LsegMax - l) / factor;
            l = LsegMax;
          } else {
            Lmax = l + p.getParametricLength();
            l = Lmax;
            currentRestParam = 0;
            restIndex++;
          }

          vpMatrix coefP = p.getPolynomialCoefficients();
          for (int j = 0; j < nbSegEq; j++) {
            for (int k = 0; k < nbRestSegCoef; k++) {
              double c = stiffnessPerUnitLength * (pow(Lmax, j + k + 1) - pow(Lmin, j + k + 1)) / (j + k + 1);
              for (int dim = 0; dim < 3; dim++) {
                B[line + 3 * j + dim] += c * coefP[dim][k];
              }
            }
          }
        }
      }

      if (LsegMax < segLength) {
        layerIndex++;
        nextLayerDepth += m_layerLength.at(layerIndex);
        lseg += LsegMax - LsegMin;
      } else
        lseg = segLength;

      currentDepth += LsegMax - LsegMin;
      LsegMin = LsegMax;
      LsegMax = segLength;
    }

    line += 3 * nbSegEq;
    startIndex += 3 * nbSegCoef;
  }

  line -= 3 * (m_needle.accessLastSegment().getOrder() + 1);
  startIndex -= 3 * (m_needle.accessLastSegment().getOrder() + 1);

  // Bevel
  {
    if (layerIndex < 0) {
      layerIndex = 0;
      currentDepth = 0;
    }
    vpMatrix I;
    I.eye(3);
    vpRotationMatrix R(0, 0, 0);

    vpColVector bevelSegment = m_needleTip->getTipPosition() - m_needleTip->getBasePosition();
    double bevLength = bevelSegment.euclideanNorm();
    bevelSegment.normalize();
    vpColVector p = vpColVector::crossProd(m_needleTip->getBaseAxisZ(), bevelSegment);
    R.buildFrom(p[0], p[1], p[2]);

    double segLength = m_needle.accessLastSegment().getParametricLength();
    double LbevMin = 0;
    if (currentDepth == 0) {
      double cosTheta = fabs(vpColVector::dotProd(m_tissue.accessSurface().getDirection(), bevelSegment));
      if (cosTheta > std::numeric_limits<double>::epsilon())
        LbevMin =
            fabs(usGeometryTools::getPointPlaneDistance(m_needleTip->getBasePosition(), m_tissue.accessSurface())) /
            cosTheta;
      else
        LbevMin = bevLength;
    }
    double lbev = LbevMin;
    double LbevMax = bevLength;
    while (lbev < bevLength) {
      double stiffnessPerUnitLength = m_stiffnessPerUnitLength.at(layerIndex);

      if (currentDepth - lbev + bevLength > nextLayerDepth && layerIndex + 1 < (int)m_layerLength.size()) {
        LbevMax = nextLayerDepth - currentDepth;
      }

      for (int j = 0; j < nbSegEq; j++) {
        for (int k = 0; k < nbSegCoef; k++) {
          double c0 = 0;
          double c1 = 0;
          double c2 = 0;
          double c3 = 0;
          // Tissue deformation minimization
          c0 = pow(segLength, j + k) * (LbevMax - LbevMin);
          if (j > 0)
            c1 = j * pow(segLength, j + k - 1) * (pow(LbevMax, 2) - pow(LbevMin, 2)) / 2;
          if (k > 0)
            c2 = k * pow(segLength, j + k - 1) * (pow(LbevMax, 2) - pow(LbevMin, 2)) / 2;
          if (j + k > 1)
            c3 = (j * k) * pow(segLength, j + k - 2) * (pow(LbevMax, 3) - pow(LbevMin, 3)) / 3;

          vpMatrix coefMat = stiffnessPerUnitLength * (c0 * I + c1 * R.t() + c2 * R + c3 * R.t() * R);

          for (int dimj = 0; dimj < 3; dimj++)
            for (int dimk = 0; dimk < 3; dimk++)
              M[line + 3 * j + dimj][startIndex + 3 * k + dimk] += coefMat[dimj][dimk];
        }
      }

      double l = LbevMin;
      while (l < LbevMax) {
        usPolynomialCurve3D p(m_tissue.accessPath().accessSegment(restIndex).getSubPolynomialCurve(
            currentRestParam,
            ((restIndex == m_tissue.accessPath().getNbSegments() - 1) ? currentRestParam : 0) +
                m_tissue.accessPath().accessSegment(restIndex).getParametricLength()));
        p.changeCoefficientsToFitBoundaries(l, l + p.getParametricLength());
        int nbRestSegCoef = p.getOrder() + 1;

        double Lmin = l;
        double Lmax = 0;
        if ((LbevMax - l) < p.getParametricLength() || restIndex == m_tissue.accessPath().getNbSegments() - 1) {
          Lmax = LbevMax;
          currentRestParam += LbevMax - l;
          l = LbevMax;
        } else {
          Lmax = l + p.getParametricLength();
          l = Lmax;
          currentRestParam = 0;
          restIndex++;
        }

        vpMatrix coefP = p.getPolynomialCoefficients();
        for (int j = 0; j < nbSegEq; j++) {
          vpColVector b0(3, 0);
          vpColVector b1(3, 0);
          for (int k = 0; k < nbRestSegCoef; k++) {
            b0 += pow(segLength, j) * (pow(Lmax, k + 1) - pow(Lmin, k + 1)) / (k + 1) * coefP.getCol(k);
          }
          if (j > 0)
            for (int k = 0; k < nbRestSegCoef; k++) {
              b1 += j * pow(segLength, j - 1) * (pow(Lmax, k + 2) - pow(Lmin, k + 2)) / (k + 2) * coefP.getCol(k);
            }
          vpColVector coef = stiffnessPerUnitLength * (b0 + R.t() * b1);

          for (int dim = 0; dim < 3; dim++)
            B[line + 3 * j + dim] += coef[dim];
        }
      }

      if (LbevMax < bevLength) {
        layerIndex++;
        nextLayerDepth += m_layerLength.at(layerIndex);
        lbev += LbevMax - LbevMin;
      } else
        lbev = bevLength;

      currentDepth += LbevMax - LbevMin;
      LbevMin = LbevMax;
      LbevMax = bevLength;
    }
  }

  A = M.inverseByLU() * B;

  // Update coefficients
  startIndex = 0;
  for (int i = 0; i < nbSegments; i++) {
    int order = m_needle.accessSegment(i).getOrder();
    vpMatrix m(3, order + 1);
    for (int j = 0; j < order + 1; j++) {
      for (int dim = 0; dim < 3; dim++) {
        m[dim][j] = A[startIndex + 3 * j + dim];
      }
    }
    m_needle.accessSegment(i).setPolynomialCoefficients(m);
    startIndex += 3 * (order + 1);
  }
}

void usNeedleInsertionModelRayleighRitzSpline::fitLength()
{
  double l = 0;
  double length = m_needle.getFullLength();
  int i = 0;
  double trueLength = 0;

  while (i < m_needle.getNbSegments() && l < length) {
    trueLength = m_needle.accessSegment(i).getLength();

    l += trueLength;
    m_needle.accessSegment(i).changeCoefficientsToFitBoundaries(0, trueLength);

    i++;
  }
  if (i < m_needle.getNbSegments())
    m_needle.removeSegments(i, m_needle.getNbSegments() - 1);

  double lastSegLength = m_needle.getFullLength() - (l - trueLength);
  m_needle.accessLastSegment().setParametricLength(lastSegLength);
  m_needle.accessLastSegment().changeCoefficientsToFitBoundaries(0, m_needle.accessLastSegment().getLength());
  m_needle.accessLastSegment().setParametricLength(lastSegLength);
}

void usNeedleInsertionModelRayleighRitzSpline::updateTipPose()
{
  vpColVector pt(m_needle.accessLastSegment().getEndPoint());
  vpTranslationVector t(pt[0], pt[1], pt[2]);

  vpColVector vect =
      vpColVector::crossProd(m_needle.accessSegment(0).getStartTangent(), m_needle.accessLastSegment().getEndTangent());
  double dot =
      vpColVector::dotProd(m_needle.accessSegment(0).getStartTangent(), m_needle.accessLastSegment().getEndTangent());
  double theta = atan2(vect.euclideanNorm(), dot);
  vect.normalize();
  vect = theta * vect;
  vpThetaUVector thetaU(vect[0], vect[1], vect[2]);
  vpRotationMatrix R(thetaU);
  vpRotationMatrix worldRtip(m_needle.getWorldMbase());
  worldRtip = R * worldRtip;

  vpPoseVector tipPose(t, worldRtip);
  m_needle.setTipPose(tipPose);
  m_needleTip->setBasePose(tipPose);
}

void usNeedleInsertionModelRayleighRitzSpline::solveSegmentsParameters()
{
#ifdef VISP_HAVE_EIGEN3
  switch (m_solvingMethod) {
  case SolvingMethod::Classic:
    this->solveSegmentsParametersFullSparseEigen();
    break;
  case SolvingMethod::FixedBeamLength:
    this->solveSegmentsParametersFullSparseEigenFixedLength();
    break;
  case SolvingMethod::NoBevel:
    this->solveSegmentsParametersSparseEigen();
    break;
  }
#else
  this->solveSegmentsParametersDense();
#endif
}

bool usNeedleInsertionModelRayleighRitzSpline::updateState()
{
  int loopIndex = 0;
  do {
    this->solveSegmentsParameters();

    this->fitLength();

    this->updateTipPose();

    if (this->IsNeedleInserted() && m_tissue.accessPath().getNbSegments() > 0) {
      usOrientedPlane3D pl(m_needle.accessLastSegment().getEndPoint(), m_needle.accessLastSegment().getEndTangent());
      if (usGeometryTools::DoesSegmentCrossPlaneDirect(m_tissue.accessPath(), pl)) {
        double l = this->getInsertionDepth();
        double restLength = -1;
        usGeometryTools::getPlaneCurveCrossingPoint(m_tissue.accessPath(), pl, 0.1 * m_needle.getOuterDiameter(),
                                                    &restLength);

        for (unsigned int i = 0; i < m_restDilatationFactor.size(); i++)
          m_restDilatationFactor.at(i) = l / restLength;
      } else {
        pl.setPosition(m_tissue.accessPath().accessLastSegment().getEndPoint());
        if (usGeometryTools::DoesSegmentCrossPlaneDirect(m_needle, pl)) {
          double l = -1;
          double restLength = m_tissue.accessPath().getParametricLength();
          usGeometryTools::getPlaneCurveCrossingPoint(m_needle, pl, 0.1 * m_needle.getOuterDiameter(), &l);
          l -= this->getNeedleFreeLength();

          for (unsigned int i = 0; i < m_restDilatationFactor.size(); i++)
            m_restDilatationFactor.at(i) = l / restLength;
        } else
          for (unsigned int i = 0; i < m_restDilatationFactor.size(); i++)
            m_restDilatationFactor.at(i) = 1;
      }

      this->solveSegmentsParameters();

      this->fitLength();

      this->updateTipPose();
    }
    loopIndex++;
  } while (this->updatePath() && (loopIndex < 2));

  return true;
}

bool usNeedleInsertionModelRayleighRitzSpline::updatePath()
{
  switch (m_pathUpdateType) {
  case PathUpdateType::NoUpdate: {
    return false;
  }
  case PathUpdateType::WithTipPosition: {
    return this->cutPathToPoint(m_needleTip->getTipPosition());
  }
  case PathUpdateType::WithTipDirection: {
    vpColVector lastPoint(3);
    vpColVector lastDirection(3);
    int restIndex = -1;
    double restParam = -1;
    if (m_tissue.accessPath().getNbSegments() > 0) {
      this->getCorrespondingPathPoint(
          m_needle.getFullLength() + (m_needleTip->getTipPosition() - m_needleTip->getBasePosition()).euclideanNorm(),
          restIndex, restParam);
      lastPoint = m_tissue.accessPath().accessLastSegment().getEndPoint();
      lastDirection = m_tissue.accessPath().accessLastSegment().getEndTangent();
    } else if (usGeometryTools::IsPointInFrontOfPlane(m_needleTip->getTipPosition(), m_tissue.accessSurface())) {
      lastDirection = m_needleTip->getTipDirection();
      lastPoint =
          usGeometryTools::projectPointOnPlane(m_needleTip->getTipPosition(), m_tissue.accessSurface(), lastDirection);
    } else
      return false;

    double insertionStep = 0;

    if (m_tissue.accessPath().getNbSegments() > 0 && restIndex == m_tissue.accessPath().getNbSegments() - 1) {
      insertionStep = restParam - m_tissue.accessPath().accessSegment(restIndex).getParametricLength();
    } else if (m_tissue.accessPath().getNbSegments() == 0) {
      insertionStep = vpColVector::dotProd(m_needleTip->getTipPosition() - lastPoint, lastDirection);
    } else
      return false;

    if (insertionStep > m_pathUpdateLengthThreshold) {
      usPolynomialCurve3D seg(1);
      vpMatrix M(3, 2);
      M.insert(lastPoint, 0, 0);

      vpColVector u = insertionStep * m_needleTip->getTipDirection();

      M.insert((1 * u).normalize(), 0, 1);
      seg.setPolynomialCoefficients(M);
      seg.setParametricLength(u.euclideanNorm());
      m_tissue.accessPath().addSegment(seg);
      return true;
    } else
      return false;
  }
  case PathUpdateType::WithTipMix: {
    vpColVector lastPoint(3);
    vpColVector lastDirection(3);
    int restIndex = -1;
    double restParam = -1;
    if (m_tissue.accessPath().getNbSegments() > 0) {
      this->getCorrespondingPathPoint(
          m_needle.getFullLength() + (m_needleTip->getTipPosition() - m_needleTip->getBasePosition()).euclideanNorm(),
          restIndex, restParam);
      lastPoint = m_tissue.accessPath().accessLastSegment().getEndPoint();
      lastDirection = m_tissue.accessPath().accessLastSegment().getEndTangent();
    } else if (usGeometryTools::IsPointInFrontOfPlane(m_needleTip->getTipPosition(), m_tissue.accessSurface())) {
      lastDirection = m_needleTip->getTipDirection();
      lastPoint =
          usGeometryTools::projectPointOnPlane(m_needleTip->getTipPosition(), m_tissue.accessSurface(), lastDirection);
    } else
      return false;

    double insertionStep = 0;

    if (m_tissue.accessPath().getNbSegments() > 0 && restIndex == m_tissue.accessPath().getNbSegments() - 1) {
      insertionStep = restParam - m_tissue.accessPath().accessSegment(restIndex).getParametricLength();
    } else if (m_tissue.accessPath().getNbSegments() == 0) {
      insertionStep = vpColVector::dotProd(m_needleTip->getTipPosition() - lastPoint, lastDirection);
    } else
      return false;

    if (insertionStep > m_pathUpdateLengthThreshold) {
      usPolynomialCurve3D seg(1);
      vpMatrix M(3, 2);
      M.insert(lastPoint, 0, 0);

      vpColVector newPoint = m_pathUpdateMixCoefficient * (lastPoint + insertionStep * m_needleTip->getTipDirection()) +
                             (1 - m_pathUpdateMixCoefficient) * m_needleTip->getTipPosition();

      M.insert((newPoint - lastPoint).normalize(), 0, 1);
      seg.setPolynomialCoefficients(M);
      seg.setParametricLength((newPoint - lastPoint).euclideanNorm());
      m_tissue.accessPath().addSegment(seg);
      return true;
    } else
      return false;
  }
  default:
    return false;
  }
}

std::ostream &operator<<(std::ostream &s, const usNeedleInsertionModelRayleighRitzSpline &model)
{
  s << "usNeedleInsertionModelRayleighRitzSpline\n";

  s << model.m_needle;

  switch (model.m_needleTipType) {
  case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::ActuatedTip: {
    s << 0 << '\n';
    s << *dynamic_cast<usNeedleTipActuated *>(model.m_needleTip);
    break;
  }
  case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::BeveledTip: {
    s << 1 << '\n';
    ;
    s << *dynamic_cast<usNeedleTipBeveled *>(model.m_needleTip);
    break;
  }
  case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::PrebentTip: {
    s << 2 << '\n';
    s << *dynamic_cast<usNeedleTipPrebent *>(model.m_needleTip);
    break;
  }
  case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::SymmetricTip: {
    s << 3 << '\n';
    s << *dynamic_cast<usNeedleTipSymmetric *>(model.m_needleTip);
    break;
  }
  }

  s << model.m_tissue;

  int n = model.m_stiffnessPerUnitLength.size();
  s << n << '\n';
  for (int i = 0; i < n; i++)
    s << model.m_stiffnessPerUnitLength.at(i) << '\n';
  for (int i = 0; i < n; i++)
    s << model.m_layerLength.at(i) << '\n';

  n = model.m_restDilatationFactor.size();
  s << n << '\n';
  for (int i = 0; i < n; i++)
    s << model.m_restDilatationFactor.at(i) << '\n';

  s.flush();
  return s;
}

std::istream &operator>>(std::istream &s, usNeedleInsertionModelRayleighRitzSpline &model)
{
  std::string c;
  s >> c;
  if (c != "usNeedleInsertionModelRayleighRitzSpline") {
    vpException e(vpException::ioError, "Stream does not contain usNeedleInsertionModelRayleighRitzSpline data");
    throw e;
  }

  s >> model.m_needle;

  int t;
  s >> t;
  switch (t) {
  case 0: {
    model.setNeedleTipType(usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::ActuatedTip);
    s >> *dynamic_cast<usNeedleTipActuated *>(model.m_needleTip);
    break;
  }
  case 1: {
    model.setNeedleTipType(usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::BeveledTip);
    s >> *dynamic_cast<usNeedleTipBeveled *>(model.m_needleTip);
    break;
  }
  case 2: {
    model.setNeedleTipType(usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::PrebentTip);
    s >> *dynamic_cast<usNeedleTipPrebent *>(model.m_needleTip);
    break;
  }
  case 3: {
    model.setNeedleTipType(usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::SymmetricTip);
    s >> *dynamic_cast<usNeedleTipSymmetric *>(model.m_needleTip);
    break;
  }
  }

  s >> model.m_tissue;
  int n;
  s >> n;
  model.m_stiffnessPerUnitLength.resize(n);
  model.m_layerLength.resize(n);
  for (int i = 0; i < n; i++)
    s >> model.m_stiffnessPerUnitLength.at(i);
  for (int i = 0; i < n; i++)
    s >> model.m_layerLength.at(i);

  s >> n;
  model.m_restDilatationFactor.resize(n);
  for (int i = 0; i < n; i++)
    s >> model.m_restDilatationFactor.at(i);

  return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleInsertionModelRayleighRitzSpline &model)
{
  s.write("usNeedleInsertionModelRayleighRitzSpline", 41);

  s <<= model.m_needle;

  int t;
  switch (model.m_needleTipType) {
  case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::ActuatedTip: {
    t = 0;
    s.write((char *)&(t), sizeof(int));
    s <<= *dynamic_cast<usNeedleTipActuated *>(model.m_needleTip);
    break;
  }
  case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::BeveledTip: {
    t = 1;
    s.write((char *)&(t), sizeof(int));
    s <<= *dynamic_cast<usNeedleTipBeveled *>(model.m_needleTip);
    break;
  }
  case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::PrebentTip: {
    t = 2;
    s.write((char *)&(t), sizeof(int));
    s <<= *dynamic_cast<usNeedleTipPrebent *>(model.m_needleTip);
    break;
  }
  case usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::SymmetricTip: {
    t = 3;
    s.write((char *)&(t), sizeof(int));
    s <<= *dynamic_cast<usNeedleTipSymmetric *>(model.m_needleTip);
    break;
  }
  }

  s <<= model.m_tissue;

  int n = model.m_stiffnessPerUnitLength.size();
  s.write((char *)&(n), sizeof(int));

  for (int i = 0; i < n; i++)
    s.write((char *)&(model.m_stiffnessPerUnitLength.at(i)), sizeof(double));
  for (int i = 0; i < n; i++)
    s.write((char *)&(model.m_layerLength.at(i)), sizeof(double));

  n = model.m_restDilatationFactor.size();
  s.write((char *)&(n), sizeof(int));
  for (int i = 0; i < n; i++)
    s.write((char *)&(model.m_restDilatationFactor.at(i)), sizeof(double));

  s.flush();
  return s;
}

std::istream &operator>>=(std::istream &s, usNeedleInsertionModelRayleighRitzSpline &model)
{
  char c[41];
  s.read(c, 41);
  if (strcmp(c, "usNeedleInsertionModelRayleighRitzSpline")) {
    vpException e(vpException::ioError, "Stream does not contain usNeedleInsertionModelRayleighRitzSpline data");
    throw e;
  }

  s >>= model.m_needle;

  int t;
  s.read((char *)&(t), sizeof(int));
  switch (t) {
  case 0: {
    s >>= *dynamic_cast<usNeedleTipActuated *>(model.m_needleTip);
    break;
  }
  case 1: {
    s >>= *dynamic_cast<usNeedleTipBeveled *>(model.m_needleTip);
    break;
  }
  case 2: {
    s >>= *dynamic_cast<usNeedleTipPrebent *>(model.m_needleTip);
    break;
  }
  case 3: {
    s >>= *dynamic_cast<usNeedleTipSymmetric *>(model.m_needleTip);
    break;
  }
  }

  s >>= model.m_tissue;

  int n;
  s.read((char *)&(n), sizeof(int));

  model.m_stiffnessPerUnitLength.resize(n);
  model.m_layerLength.resize(n);
  for (int i = 0; i < n; i++)
    s.read((char *)&(model.m_stiffnessPerUnitLength.at(i)), sizeof(double));
  for (int i = 0; i < n; i++)
    s.read((char *)&(model.m_layerLength.at(i)), sizeof(double));

  s.read((char *)&(n), sizeof(int));
  model.m_restDilatationFactor.resize(n);
  for (int i = 0; i < n; i++)
    s.read((char *)&(model.m_restDilatationFactor.at(i)), sizeof(double));
  return s;
}
