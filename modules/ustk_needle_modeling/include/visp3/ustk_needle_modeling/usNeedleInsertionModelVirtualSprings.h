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

#ifndef __usNeedleInsertionModelVirtualSprings_h
#define __usNeedleInsertionModelVirtualSprings_h

#include <iostream>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/ustk_core/usOrientedPlane3D.h>
#include <visp3/ustk_needle_modeling/usNeedleInsertionModelInterface.h>
#include <visp3/ustk_needle_modeling/usNeedleModelSpline.h>
#include <visp3/ustk_needle_modeling/usVirtualSpring.h>

class VISP_EXPORT usNeedleInsertionModelVirtualSprings : public usNeedleInsertionModelInterface
{
public:
  enum class InsertionType : int { ForceInsert, ForceRemove, NaturalBehavior };
  enum class ModelPreset : int {
    BiopsyNeedle,
    BiopsyCannula,
    AbayazidRRM13,
    MisraRSRO10_PlastisolA,
    RoesthuisAM12,
    SteelSoftTissue,
    SRL_BiopsyNID
  };

protected:
  //! Needle parameters

  usNeedleModelSpline m_needle;

  double m_tipForce;
  double m_tipMoment;
  double m_cutAngle; // degree
  double m_bevelLength;

  //! Tissue parameters

  double m_defaultSpringStiffness;
  double m_stiffnessPerUnitLength;

  //! Model Parameters

  //! Tissue

  std::vector<usVirtualSpring> m_springs;                  // active springs used for calculation
  std::vector<usVirtualSpring> m_inactiveAutoAddedSprings; // springs that have been automatically added by the model
                                                           // and that are currently disactivated
  std::vector<usVirtualSpring> m_inactiveMeasureSprings;   // springs that have been added by the user and that are
                                                           // currently disactivated (not reached)
  usOrientedPlane3D m_tissueSurface;
  double m_interSpringDistance;    // distance between successive springs
  double m_interTipSpringDistance; // distance between successive springs at the tip

  //! Internal

  //! Linear system solving

  bool m_IsStateConsistent;

  //! Segments lengths measurement

  bool m_LastSegmentLengthComputed;

  //! Insertion type

  InsertionType m_insertionBehavior;
  bool m_IsInserting;
  bool m_AllowSpringAddition;
  bool m_AllowSpringRemoval;
  bool m_AutomaticSpringAddition;

  //! Tip springs

  int m_tipSpringsIndex;
  int m_nbMinTipSprings;
  int m_nbMaxTipSprings;

public:
  //! Constructors, destructor

  usNeedleInsertionModelVirtualSprings();
  usNeedleInsertionModelVirtualSprings(const usNeedleInsertionModelVirtualSprings &needle);
  virtual ~usNeedleInsertionModelVirtualSprings();
  const usNeedleInsertionModelVirtualSprings &operator=(const usNeedleInsertionModelVirtualSprings &needle);
  virtual usNeedleInsertionModelVirtualSprings *clone() const; // Polymorph copy method

  //! Parameters saving and loading

  void loadPreset(const ModelPreset preset);

  //! Parameters setters and getters

  //! Needle parameters

  void setTipForce(double tipForce);
  double getTipForce();

  void setBevelAngle(double angle);
  double getBevelAngle() const;

  //! Tissue parameters

  void setDefaultSpringStiffness(double K);
  double getDefaultSpringStiffness() const;

  void setStiffnessPerUnitLength(double K);
  double getStiffnessPerUnitLength() const;

  int getNbSprings() const;
  int getNbMeasureSprings() const;

  //! Model parameters

  //! Needle

  const usNeedleModelSpline &accessNeedle() const;
  usNeedleModelSpline &accessNeedle();
  bool IsNeedleInserted() const;
  vpColVector getNeedleInsertionPoint() const;
  vpColVector getTissueInsertionPoint() const;
  double getNeedleFreeLength() const;
  double getInsertionDepth() const;

  //! Tissue

  const usOrientedPlane3D &accessSurface() const;
  usOrientedPlane3D &accessSurface();
  const usVirtualSpring &accessSpring(int i) const;
  void setInterSpringDistance(double interSpringDistance);
  double getInterSpringDistance() const;
  void setInterTipSpringDistance(double interTipSpringDistance);
  double getInterTipSpringDistance() const;

  void setNbMinTipSprings(int nb);
  int getNbMinTipSprings() const;

  void setNbMaxTipSprings(int nb);
  int getNbMaxTipSprings() const;

  //! Internal

  void AllowSpringAddition(bool flag);
  void AllowSpringRemoval(bool flag);

  void setInsertionBehavior(InsertionType type);
  InsertionType getInsertionBehavior() const;

  void setAutomaticSpringAddition(bool flag);
  bool getAutomaticSpringAddition() const;

  //! Measure model information

  //! Needle position

  double getPathDistanceFromPoint(const vpColVector &P) const;

  //! Tissue deformation energy

  double getTissueDeformationEnergy() const;
  double getSurfaceTissueStretch() const;
  double getMaxTissueStretch(double *lmax = nullptr) const;
  double getMeanTissueStretch() const;

  //! Control of the needle

  //! Control of the needle

  bool setBasePose(const vpPoseVector &p);
  vpPoseVector getBasePose() const;

  //! Control of the tissue

  void addMeasureSpring(
      const vpColVector &p,
      const vpColVector &d); // Add a spring in space and recompute stiffnesses in vicinity to keep the tissue property

  bool setSpringPosition(int index, const vpColVector &P, bool update = false);
  bool setSpringDirection(int index, const vpColVector &D, bool update = false);
  void setSpringStiffness(int index, double K, bool update = false);
  bool moveSpringPosition(int index, const vpColVector &dP, bool update = false);
  bool moveSpringDirection(int index, const vpThetaUVector &thetaU, bool update = false);
  void addSpringStiffness(int index, double dK, bool update = false);

  void setSurfaceAtTip();

  //! Internal model command

  int addInsertionPoint(usVirtualSpring spg);
  int addInsertionPoint(
      const vpColVector &p,
      const vpColVector &d); // Add a spring in space and recompute stiffnesses in vicinity to keep the tissue property

  void addInsertionPointOnSegmentHard(int segment,
                                      double s); // Add a spring on the needle with default spring stiffness
  void addInsertionPointAtTipHard();
  void addInsertionPointOnSegment(
      int segment,
      double s); // Add a spring on the needle and recompute stiffnesses in vicinity to keep the tissue property
  void addInsertionPointAtTip();

  void removeInsertionPointsHard(int first, int last = -1);
  void removeLastInsertionPointHard();
  void removeInsertionPoints(int first, int last = -1);
  void removeLastInsertionPoint();
  void removeAutoAddedSprings();

  void fusionSprings(int firstSpring, int lastSpring);

  void updateSpringsStiffness();
  void updateCutAngle();
  void updateTipForce();
  void updateInsertionDirections();

  bool checkInactiveMeasureSprings();

  void solveSegmentsParametersSparseEigen();
  void solveSegmentsParametersOpenCV();
  void solveSegmentsParametersViSP();
  void solveSegmentsParameters();
  void computeSegmentsLengths();
  bool addRemoveSprings();
  bool updateState();

  //! Display

  void showInsertionPoints() const;
  void showInsertionDirections() const;
  void showStiffnesses() const;

  //! Data saving

  //! Text
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleInsertionModelVirtualSprings &needle);
  friend VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleInsertionModelVirtualSprings &needle);
  //! Binary
  friend VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleInsertionModelVirtualSprings &needle);
  friend VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleInsertionModelVirtualSprings &needle);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usNeedleInsertionModelVirtualSprings &needle);
VISP_EXPORT std::istream &operator>>(std::istream &s, usNeedleInsertionModelVirtualSprings &needle);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usNeedleInsertionModelVirtualSprings &needle);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usNeedleInsertionModelVirtualSprings &needle);

#endif // __usNeedleInsertionModelVirtualSprings_h
