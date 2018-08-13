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

#ifndef __usNeedleInsertionModelRayleighRitzSpline_h
#define __usNeedleInsertionModelRayleighRitzSpline_h

#include <iostream>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>

#include <visp3/ustk_needle_modeling/usNeedleTip.h>
#include <visp3/ustk_needle_modeling/usNeedleTipActuated.h>
#include <visp3/ustk_needle_modeling/usNeedleTipBeveled.h>
#include <visp3/ustk_needle_modeling/usNeedleTipPrebent.h>
#include <visp3/ustk_needle_modeling/usNeedleTipSymmetric.h>

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelInterface.h>
#include <visp3/ustk_needle_modeling/usNeedleModelSpline.h>
#include <visp3/ustk_needle_modeling/usTissueModelSpline.h>

class VISP_EXPORT usNeedleInsertionModelRayleighRitzSpline : public usNeedleInsertionModelInterface
{
public:
  enum class ModelPreset : int {
    BiopsyNeedle,
    BiopsyCannula,
    Symmetric,
    AbayazidRRM13,
    MisraRSRO10_PlastisolA,
    RoesthuisAM12,
    SteelSoftTissue,
    SRL_ActuatedFBG,
    SRL_BiopsySimple,
    SRL_BiopsyNID,
    NDI_Pink_Stylet
  };
  enum class PathUpdateType : int { NoUpdate, WithTipPosition, WithTipDirection, WithTipMix };
  enum class NeedleTipType : int { SymmetricTip, BeveledTip, PrebentTip, ActuatedTip };
  enum class SolvingMethod : int { Classic, FixedBeamLength, NoBevel };

public: // protected:
  //! Model Parameters

  //! Needle

  usNeedleModelSpline m_needle;

  usNeedleTip *m_needleTip;
  NeedleTipType m_needleTipType;

  //! Tissue

  usTissueModelSpline m_tissue;
  std::vector<double> m_stiffnessPerUnitLength;
  std::vector<double> m_layerLength;

  //! Path update

  PathUpdateType m_pathUpdateType;
  double m_pathUpdateLengthThreshold; // insertion step before adding a new segment to the rest path (min length of the
                                      // rest path segment)
  double m_pathUpdateMixCoefficient;  // coefficient to weight the effect of tip position update(1) and tip direction
                                      // update(0) for Mix PathUpdateType

  //! Internal

  //! Linear system solving

  SolvingMethod m_solvingMethod;
  std::vector<double> m_restDilatationFactor;

public:
  //! Constructors, destructor

  usNeedleInsertionModelRayleighRitzSpline();
  usNeedleInsertionModelRayleighRitzSpline(const usNeedleInsertionModelRayleighRitzSpline &model);
  virtual ~usNeedleInsertionModelRayleighRitzSpline();
  const usNeedleInsertionModelRayleighRitzSpline &operator=(const usNeedleInsertionModelRayleighRitzSpline &model);
  virtual usNeedleInsertionModelRayleighRitzSpline *clone() const; // Polymorph copy method

  //! Parameters saving and loading

  void loadPreset(const ModelPreset preset);

  //! Parameters setters and getters

  //! Needle

  const usNeedleModelSpline &accessNeedle() const;
  usNeedleModelSpline &accessNeedle();

  usNeedleTip const &accessNeedleTip() const;
  usNeedleTip &accessNeedleTip();

  void setNeedleTipType(NeedleTipType type);
  NeedleTipType getNeedleTipType() const;

  //! Tissue parameters

  const usTissueModelSpline &accessTissue() const;
  usTissueModelSpline &accessTissue();

  bool addTissueLayer(double K, double l);

  bool setStiffnessPerUnitLength(int i, double K);
  bool setStiffnessPerUnitLength(double K);
  double getStiffnessPerUnitLength(int i = 0) const;

  bool setLayerLength(int i, double l);
  double getLayerLength(int i) const;

  int getNbLayers() const;
  int getNbCurrentLayers() const;

  //! Model behavior

  void setPathUpdateType(PathUpdateType type);
  void setPathUpdateLengthThreshold(double length);
  void setPathUpdateMixCoefficient(double coef);
  void setSolvingMethod(SolvingMethod method);

  //! Model parameters

  //! Needle

  bool IsNeedleInserted() const;
  double getNeedleFreeLength(int *seg = nullptr, double *param = nullptr) const;
  double getInsertionDepth() const;
  vpColVector getNeedleInsertionPoint() const;
  vpColVector getTissueInsertionPoint() const;

  bool getCorrespondingPathPoint(double l, int &correspondingRestIndex, double &correspondingRestParam) const;

  //! Measure model information

  //! Tissue deformation energy

  double getTissueDeformationEnergy() const;
  double getSurfaceTissueStretch() const;
  double getMaxTissueStretch(double *lmax = nullptr) const;
  double getMeanTissueStretch() const;

  //! Curvature

  double getCurvatureFromKinematics() const;

  //! Control of the needle

  using usNeedleInsertionModelInterface::setBasePose;
  bool setBasePose(const vpPoseVector &p);
  vpPoseVector getBasePose() const;

  //! Control of the tissue

  void setSurfaceAtTip();
  bool cutPathToPoint(const vpColVector &P);

  //! Internal model command

  void solveSegmentsParametersSparseEigen();     // solve without bevel (less accurate at tip, but less expensive since
                                                 // equation independant on dimension)
  void solveSegmentsParametersFullSparseEigen(); // solve with bevel (more accurate modeling of bevel effect, but
                                                 // slighly more expensive)
  void solveSegmentsParametersFullSparseEigenFixedLength(); // solve with bevel + fixed size for sub splines
  void solveSegmentsParametersDense(); // solve with dense method with bevel + fixed size for sub splines (very slow,
                                       // only here for compatibility but should be avoided)
  void fitLength();
  void updateTipPose();

  void solveSegmentsParameters();
  virtual bool updateState();

  bool updatePath();

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleInsertionModelRayleighRitzSpline &model);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleInsertionModelRayleighRitzSpline &model);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleInsertionModelRayleighRitzSpline &model);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleInsertionModelRayleighRitzSpline &model);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleInsertionModelRayleighRitzSpline &model);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleInsertionModelRayleighRitzSpline &model);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleInsertionModelRayleighRitzSpline &model);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleInsertionModelRayleighRitzSpline &model);

#endif // __usNeedleInsertionModelRayleighRitzSpline_h
