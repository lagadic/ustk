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

#ifndef __usNeedleJacobians_h_
#define __usNeedleJacobians_h_

#include <functional>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRotationMatrix.h>


namespace usNeedleJacobians
{
  //! Generic Jacobians
  
  template<class NeedleInsertionModel>
  bool getJacobian(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod, std::function<vpColVector(const NeedleInsertionModel&)> OutputVectorMethod, std::function<vpColVector(const vpColVector&, const vpColVector&, const double step)> DifferenceMethod, vpMatrix &J, const vpColVector &discretizationStep, bool centeredDifference=true);
  
  template<class NeedleInsertionModel>
  bool getJacobianScalarToScalar(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, double)> InputScalarMethod, std::function<double(const NeedleInsertionModel&)> OutputScalarMethod, vpMatrix &J, double discretizationStep = 1e-5, bool centeredDifference=true);
  
  template<class NeedleInsertionModel>
  bool getJacobianScalarToVector(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, double)> InputScalarMethod, std::function<vpColVector(const NeedleInsertionModel&)> OutputVectorMethod, vpMatrix &J, double discretizationStep = 1e-5, bool centeredDifference=true);
  
  template<class NeedleInsertionModel>
  bool getJacobianVectorToScalar(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod, std::function<double(const NeedleInsertionModel&)> OutputScalarMethod, vpMatrix &J, const vpColVector &discretizationStep, bool centeredDifference=true);
  
  template<class NeedleInsertionModel>
  bool getJacobianVectorToVector(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod, std::function<vpColVector(const NeedleInsertionModel&)> OutputVectorMethod, vpMatrix &J, const vpColVector &discretizationStep, bool centeredDifference=true);
  
  template<class NeedleInsertionModel>
  bool getJacobianVectorToPose(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod, std::function<vpHomogeneousMatrix(const NeedleInsertionModel&)> OutputVectorMethod, vpMatrix &J, const vpColVector &discretizationStep, bool centeredDifference=true);
  
  //! Base to tip (manipulation Jacobian) (6x6)
  
  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToWorldTipVelocity(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToWorldTipVelocity(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToTipVelocity(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToTipVelocity(const NeedleInsertionModel &model, vpMatrix &J);
  
  //! Base to bending energy (1x6)
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToBendingEnergy(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToBendingEnergy(const NeedleInsertionModel &model, vpMatrix &J);
  
  //! Base to tissue deformation energy (1x6)
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToTissueEnergy(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToTissueEnergy(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToNeedleInsertionPointVelocity(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToSurfaceTissueStretch(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToMaxTissueStretch(const NeedleInsertionModel &model, vpMatrix &J);
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToMeanTissueStretch(const NeedleInsertionModel &model, vpMatrix &J);
  
  //! Base to needle point (3x6)
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToPointVelocity(const NeedleInsertionModel &model, vpMatrix &J, double l);
  
  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToPointVelocity(const NeedleInsertionModel &model, vpMatrix &J, double l);
  
  //! Base to features (1x6)
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToTransversalTargetDistance(const NeedleInsertionModel &model, vpMatrix &J, const vpColVector &target);
  
  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToTargetAngle(const NeedleInsertionModel &model, vpMatrix &J, const vpColVector &target);
  
  //! Springs to needle point (3x6)
  
  template<class NeedleInsertionModel>
  bool getJacobianSpringVelocityToPointVelocity(const NeedleInsertionModel &model, vpMatrix &J, int spring, double l);
  
  //! Springs to distance from points (nbPointsx6)
  
  template<class NeedleInsertionModel>
  bool getJacobianSpringVelocityToPointsDistance(const NeedleInsertionModel &model, vpMatrix &J, int spring, const std::vector<vpColVector> &points);
  
  //! Rest path points to model point (3x3)
  
  template<class NeedleInsertionModel>
  bool getJacobianRestPathPointVelocityToPointsVelocity(const NeedleInsertionModel &model, vpMatrix &J, int restPoint, const std::vector<double> &l);
  
  //! Rest path points distance from points (nbPointsx3)
  
  template<class NeedleInsertionModel>
  bool getJacobianRestPathPointVelocityToPointsDistance(const NeedleInsertionModel &model, vpMatrix &J, int restPoint, const std::vector<vpColVector> &points);
  
  //! Stiffness per unit length to distance from points (nbPointsx1)
  
  template<class NeedleInsertionModel>
  bool getJacobianStiffnessPerUnitLengthToPointsDistance(const NeedleInsertionModel &model, vpMatrix &J, const std::vector<vpColVector> &points);
  
  //! Stiffness per unit length to needle point ( (3*nbPoints)x1 matrix)
  
  template<class NeedleInsertionModel>
  bool getJacobianStiffnessPerUnitLengthToPointsVelocity(const NeedleInsertionModel &model, vpMatrix &J, const std::vector<double> &l);

  /*!
   *  Implementation
  !*/
  
  //! Generic Jacobian computation

  template<class NeedleInsertionModel>
  bool getJacobian(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod, std::function<vpColVector(const NeedleInsertionModel&)> OutputVectorMethod, std::function<vpColVector( const vpColVector&, const vpColVector&, const double step)> DifferenceMethod, vpMatrix &J, const vpColVector &discretizationStep, bool centeredDifference)
  {
      // dVectorOut = J * dVectorIn
  
      double stepFactor = (centeredDifference?2:1);
      NeedleInsertionModel SimulationNeedle(model);
  
      int dim = discretizationStep.size();
      vpColVector test(OutputVectorMethod(SimulationNeedle));
      vpMatrix M(test.size(), dim);
  
      vpColVector v2;
      if(!centeredDifference) v2 = OutputVectorMethod(model);
      for(int i=0 ; i<dim ; i++)
      {
          vpColVector input(dim,0);
  
          input[i] = discretizationStep[i];
  
          SimulationNeedle = model;
          if(!InputVectorMethod(SimulationNeedle, input)) return false;
  
          vpColVector v1 = OutputVectorMethod(SimulationNeedle);
  
          if(centeredDifference)
          {
              input[i] = -discretizationStep[i];
  
              SimulationNeedle = model;
              if(!InputVectorMethod(SimulationNeedle, input)) return false;
  
              v2 = OutputVectorMethod(SimulationNeedle);
          }
  
          M.insert(DifferenceMethod(v1,v2,stepFactor*discretizationStep[i]), 0,i);
      }
  
      J = M;
    
      return true;
  }

  template<class NeedleInsertionModel>
  bool getJacobianScalarToScalar(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, double)> InputScalarMethod, std::function<double(const NeedleInsertionModel&)> OutputScalarMethod, vpMatrix &J, double discretizationStep, bool centeredDifference)
  {
      // dScalarOut = J * dScalarIn

      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod = [&InputScalarMethod](NeedleInsertionModel& m, const vpColVector& v)
      {
          return InputScalarMethod(m, v[0]);
      };

      std::function<vpColVector(const NeedleInsertionModel&)> OutputVectorMethod = [&OutputScalarMethod](const NeedleInsertionModel& m)
      {
          return vpColVector(1,OutputScalarMethod(m));
      };

      std::function<vpColVector(const vpColVector&, const vpColVector&, const double step)> DifferenceMethod = [](const vpColVector& v1, const vpColVector& v2, const double step)
      {
          return 1./step*(v1-v2);
      };
      vpColVector vectorDiscretizationStep(1,discretizationStep);
  
      return getJacobian(model, InputVectorMethod, OutputVectorMethod, DifferenceMethod, J,  vectorDiscretizationStep, centeredDifference);
  }

  template<class NeedleInsertionModel>
  bool getJacobianScalarToVector(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, double)> InputScalarMethod, std::function<vpColVector(const NeedleInsertionModel&)> OutputVectorMethod, vpMatrix &J, double discretizationStep, bool centeredDifference)
  {
      // dVectorOut = J * dScalarIn

      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod = [&InputScalarMethod](NeedleInsertionModel& m, const vpColVector& v)
      {
          return InputScalarMethod(m, v[0]);
      };
      
      std::function<vpColVector(const vpColVector&, const vpColVector&, const double step)> DifferenceMethod = [](const vpColVector& v1, const vpColVector& v2, const double step)
      {
          return 1./step*(v1-v2);
      };
      vpColVector vectorDiscretizationStep(1,discretizationStep);
      
      return getJacobian(model, InputVectorMethod, OutputVectorMethod, DifferenceMethod, J,  vectorDiscretizationStep, centeredDifference);
  }

  template<class NeedleInsertionModel>
  bool getJacobianVectorToScalar(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod, std::function<double(const NeedleInsertionModel&)> OutputScalarMethod, vpMatrix &J, const vpColVector &discretizationStep, bool centeredDifference)
  {
      // dScalarOut = J * dVectorIn
      
      std::function<vpColVector(const NeedleInsertionModel&)> OutputVectorMethod = [&OutputScalarMethod](const NeedleInsertionModel& m)
      {
          return vpColVector(1,OutputScalarMethod(m));
      };
      
      std::function<vpColVector(const vpColVector&, const vpColVector&, const double step)> DifferenceMethod = [](const vpColVector& v1, const vpColVector& v2, const double step)
      {
          return 1./step*(v1-v2);
      };
      
      return getJacobian(model, InputVectorMethod, OutputVectorMethod, DifferenceMethod, J, discretizationStep, centeredDifference);
  }

  template<class NeedleInsertionModel>
  bool getJacobianVectorToVector(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod, std::function<vpColVector(const NeedleInsertionModel&)> OutputVectorMethod, vpMatrix &J, const vpColVector &discretizationStep, bool centeredDifference)
  {
      // dVectorOut = J * dVectorIn

      std::function<vpColVector(const vpColVector&, const vpColVector&, const double step)> DifferenceMethod = [](const vpColVector& v1, const vpColVector& v2, const double step)
      {
          return 1./step*(v1-v2);
      };

      return getJacobian(model, InputVectorMethod, OutputVectorMethod, DifferenceMethod, J,  discretizationStep, centeredDifference);
  }

  template<class NeedleInsertionModel>
  bool getJacobianVectorToPose(NeedleInsertionModel model, std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputVectorMethod, std::function<vpHomogeneousMatrix(const NeedleInsertionModel&)> OutputVectorMethod, vpMatrix &J, const vpColVector &discretizationStep, bool centeredDifference)
  {
      // dPose/dt= J * dVectorIn/dt (use exponential map instead of difference)
      
      std::function<vpColVector(const NeedleInsertionModel&)> trueOutputVectorMethod = [&OutputVectorMethod](const NeedleInsertionModel& m)
      {
          return vpColVector(vpPoseVector(OutputVectorMethod(m)));
      };
      
      std::function<vpColVector(const vpColVector&, const vpColVector&, const double step)> DifferenceMethod = [](const vpColVector& v1, const vpColVector& v2, const double step)
      {
          vpHomogeneousMatrix H1(v1[0], v1[1], v1[2], v1[3], v1[4], v1[5]);
          vpHomogeneousMatrix H2(v2[0], v2[1], v2[2], v2[3], v2[4], v2[5]);
          return vpExponentialMap::inverse(H2.inverse()*H1, step);
      };
      
      return getJacobian(model, InputVectorMethod, trueOutputVectorMethod, DifferenceMethod, J,  discretizationStep, centeredDifference);
  }

  //! Base to tip

  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToTipVelocity(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBaseWorldFrame(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<vpHomogeneousMatrix(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n)
                                                                          {
                                                                              return n.accessNeedle().getWorldMtip();
                                                                          };

      bool success = getJacobianVectorToPose(model, InputFunction, OutputFunction, J, discretStep);

      return success;
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToTipVelocity(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<vpHomogeneousMatrix(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n)
                                                                          {
                                                                              return n.accessNeedle().getWorldMtip();
                                                                          };

      bool success = getJacobianVectorToPose(model, InputFunction, OutputFunction, J, discretStep);

      return success;
  }

  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToWorldTipVelocity(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpMatrix M;
      bool success = getJacobianWorldBaseVelocityToTipVelocity(model,M);

      if(success)
      {
          // Vtip_tip = J * Vbase_world
          //                   (worldRtip     0    )
          // => Vtip_world =   (    0     worldRtip) * J  * Vbase_world

          vpRotationMatrix worldRtip(model.accessNeedle().getWorldMtip());

          vpMatrix Jtranslation(M, 0,0, 3,6);
          vpMatrix Jrotation(M, 3,0, 3,6);

          Jtranslation = static_cast<vpMatrix>(worldRtip) * Jtranslation;
          Jrotation = static_cast<vpMatrix>(worldRtip) * Jrotation;

          J.resize(6,6);
          J.insert(Jtranslation, 0,0);
          J.insert(Jrotation, 3,0);
      }

      return success;
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToWorldTipVelocity(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpMatrix M;
      bool success = getJacobianBaseVelocityToTipVelocity(model,M);

      if(success)
      {
          // Vtip_tip = J * Vbase_base
          //                   (worldRtip     0    )
          // => Vtip_world =   (    0     worldRtip) * J  * Vbase_base

          vpRotationMatrix worldRtip(model.accessNeedle().getWorldMtip());

          vpMatrix Jtranslation(M, 0,0, 3,6);
          vpMatrix Jrotation(M, 3,0, 3,6);

          Jtranslation = static_cast<vpMatrix>(worldRtip) * Jtranslation;
          Jrotation = static_cast<vpMatrix>(worldRtip) * Jrotation;

          J.resize(6,6);
          J.insert(Jtranslation, 0,0);
          J.insert(Jrotation, 3,0);
      }

      return success;
  }

  //! Base to bending energy (1x6)

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToBendingEnergy(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 1e-5*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n){return n.accessNeedle().getBendingEnergy();};
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToBendingEnergy(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 1e-5*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBaseWorldFrame(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n){return n.accessNeedle().getBendingEnergy();};
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Base to tissue deformation energy (1x6)

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToTissueEnergy(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n){return n.getTissueDeformationEnergy();};
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToTissueEnergy(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n){return n.getTissueDeformationEnergy();};
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToNeedleInsertionPointVelocity(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n){return n.getNeedleInsertionPoint();};
      return getJacobianVectorToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToBaseForceTorque(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n)
                                                                          {
                                                                              vpColVector baseForceTorque = n.accessNeedle().getBaseStaticTorsor();
                                                                              vpRotationMatrix bRw = n.accessNeedle().getWorldMbase().getRotationMatrix().inverse();
                                                                              baseForceTorque.insert(0, bRw * vpColVector(baseForceTorque, 0,3));
                                                                              baseForceTorque.insert(3, bRw * vpColVector(baseForceTorque, 3,3));
                                                                              return baseForceTorque;
                                                                          };
      return getJacobianVectorToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToInsertionLength(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n){return n.getInsertionDepth();};
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToSurfaceTissueStretch(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector e = model.getNeedleInsertionPoint() - model.getTissueInsertionPoint();
      double norm = e.frobeniusNorm();
      if(norm <= std::numeric_limits<double>::epsilon()) return false;
      else e /= norm;

      vpMatrix M(6,3,0);
      if(getJacobianBaseVelocityToNeedleInsertionPointVelocity(model,M))
      {
          J = e.t() * M;
          return true;
      }
      else return false;
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToMaxTissueStretch(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n){return n.getMaxTissueStretch();};
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToMeanTissueStretch(const NeedleInsertionModel &model, vpMatrix &J)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [](const NeedleInsertionModel& n){return n.getMeanTissueStretch();};
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Base to model point (3x6)

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToPointVelocity(const NeedleInsertionModel &model, vpMatrix &J, double l)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [l](const NeedleInsertionModel& n){return n.accessNeedle().getPoint(l);};
      return getJacobianVectorToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianWorldBaseVelocityToPointVelocity(const NeedleInsertionModel &model, vpMatrix &J, double l)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBaseWorldFrame(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [l](const NeedleInsertionModel& n){return n.accessNeedle().getPoint(l);};
      return getJacobianVectorToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Base to features (1x6)

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToTransversalTargetDistance(const NeedleInsertionModel &model, vpMatrix &J, const vpColVector &target)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [target](const NeedleInsertionModel& n)
                                                                  {
                                                                      vpColVector v(4,1);
                                                                      v.insert(0,target);
                                                                      v = n.accessNeedle().getWorldMtip().inverse()*v;
                                                                      return sqrt(v[0]*v[0]+v[1]*v[1]);
                                                                  };
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  template<class NeedleInsertionModel>
  bool getJacobianBaseVelocityToTargetAngle(const NeedleInsertionModel &model, vpMatrix &J, const vpColVector &target)
  {
      vpColVector discretStep(6, 0.01*model.accessNeedle().getParametricLength());
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  n.accessNeedle().moveBase(v);
                                                                                  return n.updateState();
                                                                              };
      std::function<double(const NeedleInsertionModel&)> OutputFunction = [target](const NeedleInsertionModel& n)
                                                                  {
                                                                      vpColVector v(4,1);
                                                                      v.insert(0,target);
                                                                      v = n.accessNeedle().getWorldMtip().inverse()*v;
                                                                      return atan2(sqrt(v[0]*v[0]+v[1]*v[1]),v[2]);
                                                                  };
      return getJacobianVectorToScalar(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Springs to model point (3x3)

  template<class NeedleInsertionModel>
  bool getJacobianSpringVelocityToPointVelocity(const NeedleInsertionModel &model, vpMatrix &J, int spring, double l)
  {
      vpColVector discretStep(3, 1e-6);
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [spring](NeedleInsertionModel& n, const vpColVector& v){return n.moveSpringPosition(spring, v, true);};
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [l](const NeedleInsertionModel& n){return n.accessNeedle().getPoint(l);};
      return getJacobianVectorToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Springs to distance from points (nbPointsx3)

  template<class NeedleInsertionModel>
  bool getJacobianSpringVelocityToPointsDistance(const NeedleInsertionModel &model, vpMatrix &J, int spring, const std::vector<vpColVector> &points)
  {
      vpColVector discretStep(3, 1e-6);
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [spring](NeedleInsertionModel& n, const vpColVector& v){return n.moveSpringPosition(spring, v, true);};
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [points](const NeedleInsertionModel& n)
                                                                      {
                                                                          int m = points.size();
                                                                          vpColVector P(m,1);
                                                                          for(int i=0 ; i<m ; i++)
                                                                          {
                                                                              P[i] = n.accessNeedle().getDistanceFromPoint(points.at(i));
                                                                          }
                                                                          return P;
                                                                      };
      return getJacobianVectorToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Rest path points to model point (3x3)

  template<class NeedleInsertionModel>
  bool getJacobianRestPathPointVelocityToPointsVelocity(const NeedleInsertionModel &model, vpMatrix &J, int restPoint, const std::vector<double> &l)
  {
      vpColVector discretStep(3, 1e-6);
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [restPoint](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  int m = n.accessTissue().accessPath().getNbSegments()+1;
                                                                                  if(restPoint<0 || restPoint>=m || v.size()!=3) return false;
                                                                                  std::vector<vpColVector> P(m);
                                                                                  std::vector<double> lengths(m-1);
                                                                                  for(int i=0 ; i<m-1 ; i++)
                                                                                  {
                                                                                      P.at(i) = n.accessTissue().accessPath().accessSegment(i).getStartPoint();
                                                                                      lengths.at(i) = n.accessTissue().accessPath().accessSegment(i).getParametricLength();
                                                                                  }
                                                                                  P.back() = n.accessTissue().accessPath().accessLastSegment().getEndPoint();

                                                                                  P.at(restPoint) += v;
                                                                                  n.accessTissue().accessPath().defineFromPoints(P,lengths,1);
                                                                                  n.updateState();
                                                                                  return true;
                                                                              };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [l](const NeedleInsertionModel& n)
                                                                      {
                                                                          int m = l.size();
                                                                          vpColVector P(3*m,1);
                                                                          for(int i=0 ; i<m ; i++)
                                                                          {
                                                                              P.insert(3*i, n.accessNeedle().getPoint(l.at(i)));
                                                                          }
                                                                          return P;
                                                                      };
      return getJacobianVectorToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Rest path points distance from points (nbPointsx3)

  template<class NeedleInsertionModel>
  bool getJacobianRestPathPointVelocityToPointsDistance(const NeedleInsertionModel &model, vpMatrix &J, int restPoint, const std::vector<vpColVector> &points)
  {
      vpColVector discretStep(3, 1e-6);
      std::function<bool(NeedleInsertionModel&, const vpColVector&)> InputFunction = [restPoint](NeedleInsertionModel& n, const vpColVector& v)
                                                                              {
                                                                                  int m = n.accessTissue().accessPath().getNbSegments()+1;
                                                                                  if(restPoint<0 || restPoint>=m || v.size()!=3) return false;
                                                                                  std::vector<vpColVector> P(m);
                                                                                  std::vector<double> lengths(m-1);
                                                                                  for(int i=0 ; i<m-1 ; i++)
                                                                                  {
                                                                                      P.at(i) = n.accessTissue().accessPath().accessSegment(i).getStartPoint();
                                                                                      lengths.at(i) = n.accessTissue().accessPath().accessSegment(i).getParametricLength();
                                                                                  }
                                                                                  P.back() = n.accessTissue().accessPath().accessLastSegment().getEndPoint();

                                                                                  P.at(restPoint) += v;
                                                                                  n.updateState();
                                                                                  return true;
                                                                              };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [points](const NeedleInsertionModel& n)
                                                                      {
                                                                          int m = points.size();
                                                                          vpColVector P(m,1);
                                                                          for(int i=0 ; i<m ; i++)
                                                                          {
                                                                              P[i] = n.accessNeedle().getDistanceFromPoint(points.at(i));
                                                                          }
                                                                          return P;
                                                                      };
      return getJacobianVectorToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Stiffness per unit length to distance from points (nbPointsx1)

  template<class NeedleInsertionModel>
  bool getJacobianStiffnessPerUnitLengthToPointsDistance(const NeedleInsertionModel &model, vpMatrix &J, const std::vector<vpColVector> &points)
  {
      double discretStep = 0.01*model.getStiffnessPerUnitLength();
      std::function<bool(NeedleInsertionModel&, double)> InputFunction = [](NeedleInsertionModel& n, double dK)
                                                                  {
                                                                      n.setStiffnessPerUnitLength(n.getStiffnessPerUnitLength()+dK);
                                                                      return n.updateState();
                                                                  };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [points](const NeedleInsertionModel& n)
                                                                  {
                                                                      vpColVector d(points.size());
                                                                      for(unsigned int i=0 ; i<points.size() ; i++) d[i] = n.accessNeedle().getDistanceFromPoint(points.at(i));
                                                                      return d;
                                                                  };
      return getJacobianScalarToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Stiffness per unit length to model point ( (3*nbPoints)x1 matrix)

  template<class NeedleInsertionModel>
  bool getJacobianStiffnessPerUnitLengthToPointsVelocity(const NeedleInsertionModel &model, vpMatrix &J, const std::vector<double> &l)
  {
      double discretStep = 0.01*model.getStiffnessPerUnitLength();
      std::function<bool(NeedleInsertionModel&, double)> InputFunction = [](NeedleInsertionModel& n, double dK)
                                                                  {
                                                                      n.setStiffnessPerUnitLength(n.getStiffnessPerUnitLength()+dK);
                                                                      return n.updateState();
                                                                  };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [l](const NeedleInsertionModel& n)
                                                                      {
                                                                          int m = l.size();
                                                                          vpColVector P(3*m,1);
                                                                          for(int i=0 ; i<m ; i++)
                                                                          {
                                                                              P.insert(3*i, n.accessNeedle().getPoint(l.at(i)));
                                                                          }
                                                                          return P;
                                                                      };
      return getJacobianScalarToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

  //! Stiffness per unit length to model point (multilayer) ( (3*nbPoints) x 1 matrix)

  template<class NeedleInsertionModel>
  bool getJacobianStiffnessPerUnitLengthToPointsVelocity(const NeedleInsertionModel &model, vpMatrix &J, int i, const std::vector<double> &l)
  {
      double discretStep = 0.01*model.getStiffnessPerUnitLength(i);
      std::function<bool(NeedleInsertionModel&, double)> InputFunction = [i](NeedleInsertionModel& n, double dK)
                                                                  {
                                                                      n.setStiffnessPerUnitLength(i, n.getStiffnessPerUnitLength(i)+dK);
                                                                      return n.updateState();
                                                                  };
      std::function<vpColVector(const NeedleInsertionModel&)> OutputFunction = [l](const NeedleInsertionModel& n)
                                                                      {
                                                                          int m = l.size();
                                                                          vpColVector P(3*m,1);
                                                                          for(int i=0 ; i<m ; i++)
                                                                          {
                                                                              P.insert(3*i, n.accessNeedle().getPoint(l.at(i)));
                                                                          }
                                                                          return P;
                                                                      };
      return getJacobianScalarToVector(model, InputFunction, OutputFunction, J, discretStep);
  }

} //namespace usNeedleJacobians

#endif // __usNeedleJacobians_h_
