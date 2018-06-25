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

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelVirtualSprings.h>

#include<iomanip>

#include <visp3/ustk_needle_detection/usGeometryTools.h>

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_EIGEN3
#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/SparseLU>
#elif VISP_HAVE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpTranslationVector.h>


usNeedleInsertionModelVirtualSprings::usNeedleInsertionModelVirtualSprings():
    m_needle(),

    m_tipForce(0),
    m_tipMoment(0),
    m_cutAngle(0),
    m_bevelLength(0.001),

    m_defaultSpringStiffness(100),
    m_stiffnessPerUnitLength(25000),
    m_tissueSurface(vpColVector(3,0), vpColVector(3,0)),

    m_interSpringDistance(0.005),
    m_interTipSpringDistance(0.0005),

    m_IsStateConsistent(false),

    m_LastSegmentLengthComputed(true),

    m_insertionBehavior(InsertionType::NaturalBehavior),
    m_IsInserting(false),
    m_AllowSpringAddition(true),
    m_AllowSpringRemoval(true),
    m_AutomaticSpringAddition(true),

    m_tipSpringsIndex(0),
    m_nbMinTipSprings(1),
    m_nbMaxTipSprings(10)

{
    // Computation of tip position
    this->solveSegmentsParameters();
}

usNeedleInsertionModelVirtualSprings::usNeedleInsertionModelVirtualSprings(const usNeedleInsertionModelVirtualSprings &needle):
    m_needle(needle.m_needle),

    m_tipForce(needle.m_tipForce),
    m_tipMoment(needle.m_tipMoment),
    m_cutAngle(needle.m_cutAngle),
    m_bevelLength(needle.m_bevelLength),

    m_defaultSpringStiffness(needle.m_defaultSpringStiffness),
    m_stiffnessPerUnitLength(needle.m_stiffnessPerUnitLength),
            
    m_springs(needle.m_springs),
    m_inactiveAutoAddedSprings(needle.m_inactiveAutoAddedSprings),
    m_inactiveMeasureSprings(needle.m_inactiveMeasureSprings),
    m_tissueSurface(needle.m_tissueSurface),
    m_interSpringDistance(needle.m_interSpringDistance),
    m_interTipSpringDistance(needle.m_interTipSpringDistance),

    m_IsStateConsistent(needle.m_IsStateConsistent),

    m_LastSegmentLengthComputed(needle.m_LastSegmentLengthComputed),

    m_insertionBehavior(needle.m_insertionBehavior),
    m_IsInserting(needle.m_IsInserting),
    m_AllowSpringAddition(needle.m_AllowSpringAddition),
    m_AllowSpringRemoval(needle.m_AllowSpringRemoval),
    m_AutomaticSpringAddition(needle. m_AutomaticSpringAddition),

    m_tipSpringsIndex(needle.m_tipSpringsIndex),
    m_nbMinTipSprings(needle.m_nbMinTipSprings),
    m_nbMaxTipSprings(needle.m_nbMaxTipSprings)

{
}

usNeedleInsertionModelVirtualSprings::~usNeedleInsertionModelVirtualSprings()
{

}

usNeedleInsertionModelVirtualSprings* usNeedleInsertionModelVirtualSprings::clone() const
{
    return new usNeedleInsertionModelVirtualSprings(*this);
}


const usNeedleInsertionModelVirtualSprings& usNeedleInsertionModelVirtualSprings::operator=(const usNeedleInsertionModelVirtualSprings &needle)
{
    m_needle = needle.m_needle;
    m_tipForce = needle.m_tipForce;
    m_tipMoment = needle.m_tipMoment;
    m_cutAngle = needle.m_cutAngle;
    m_bevelLength = needle.m_bevelLength;

    m_defaultSpringStiffness = needle.m_defaultSpringStiffness;
    m_stiffnessPerUnitLength = needle.m_stiffnessPerUnitLength;

    m_springs = needle.m_springs;
    m_inactiveAutoAddedSprings = needle.m_inactiveAutoAddedSprings;
    m_inactiveMeasureSprings = needle.m_inactiveMeasureSprings;
    m_tissueSurface = needle.m_tissueSurface;
    m_interSpringDistance = needle.m_interSpringDistance;
    m_interTipSpringDistance = needle.m_interTipSpringDistance;

    m_IsStateConsistent = needle.m_IsStateConsistent;

    m_LastSegmentLengthComputed = needle.m_LastSegmentLengthComputed;

    m_insertionBehavior = needle.m_insertionBehavior;
    m_IsInserting = needle.m_IsInserting;
    m_AllowSpringAddition = needle.m_AllowSpringAddition;
    m_AllowSpringRemoval = needle.m_AllowSpringRemoval;
    m_AutomaticSpringAddition = needle.m_AutomaticSpringAddition;

    m_tipSpringsIndex = needle.m_tipSpringsIndex;
    m_nbMinTipSprings = needle.m_nbMinTipSprings;
    m_nbMaxTipSprings = needle.m_nbMaxTipSprings;

    return *this;
}

void usNeedleInsertionModelVirtualSprings::loadPreset(const ModelPreset preset)
{
    switch(preset)
    {
        case ModelPreset::BiopsyNeedle :
        {
            m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::BiopsyNeedle);
            this->setInterSpringDistance(0.005);
            this->setInterTipSpringDistance(0.0005);
            this->setNbMinTipSprings(10);
            this->setNbMaxTipSprings(12);
            this->setStiffnessPerUnitLength(20000);
            this->setBevelAngle(M_PI/180*24);
            break;
        }
        case ModelPreset::BiopsyCannula :
        {
            m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::BiopsyCannula);
            this->setInterSpringDistance(0.005);
            this->setInterTipSpringDistance(0.0005);
            this->setNbMinTipSprings(10);
            this->setNbMaxTipSprings(12);
            this->setStiffnessPerUnitLength(20000);
            this->setBevelAngle(M_PI/180*24);
            break;
        }
        case ModelPreset::AbayazidRRM13 :
        {
            m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::AbayazidRRM13);
            this->setInterSpringDistance(0.005);
            this->setInterTipSpringDistance(0.0005);
            this->setNbMinTipSprings(10);
            this->setNbMaxTipSprings(12);
            this->setStiffnessPerUnitLength(35500*4*0.2/0.17);
            break;
        }
        case ModelPreset::MisraRSRO10_PlastisolA :
        {
            m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::MisraRSRO10_PlastisolA);
            this->setInterSpringDistance(0.005);
            this->setInterTipSpringDistance(0.0005);
            this->setNbMinTipSprings(10);
            this->setNbMaxTipSprings(12);
            this->setStiffnessPerUnitLength(4830);
            break;
        }
        case ModelPreset::RoesthuisAM12 :
        {
            m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::RoesthuisAM12);
            this->setInterSpringDistance(0.005);
            this->setInterTipSpringDistance(0.0005);
            this->setNbMinTipSprings(10);
            this->setNbMaxTipSprings(12);
            this->setStiffnessPerUnitLength(150000);
            break;
        }
        case ModelPreset::SteelSoftTissue :
        {
            m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::SteelSoftTissue);
            this->setInterSpringDistance(0.005);
            this->setInterTipSpringDistance(0.0005);
            this->setNbMinTipSprings(10);
            this->setNbMaxTipSprings(10);
            this->setStiffnessPerUnitLength(500);
            break;
        }
        case ModelPreset::SRL_BiopsyNID :
        {
            m_needle.loadPreset(usNeedleModelSpline::NeedlePreset::SRL_BiopsyNID);
            this->setInterSpringDistance(0.005);
            this->setInterTipSpringDistance(0.0005);
            this->setNbMinTipSprings(10);
            this->setNbMaxTipSprings(12);
            this->setStiffnessPerUnitLength(20000);
            this->setBevelAngle(M_PI/180*30);
            break;
        }
    }
}

void usNeedleInsertionModelVirtualSprings::setTipForce(double tipForce)
{
    m_tipForce = tipForce;
}

double usNeedleInsertionModelVirtualSprings::getTipForce()
{
    return m_tipForce;
}

void usNeedleInsertionModelVirtualSprings::setBevelAngle(double angle)
{
    m_bevelLength = m_needle.getOuterDiameter() / tan(angle);
}

double usNeedleInsertionModelVirtualSprings::getBevelAngle() const
{
    return atan2(m_needle.getOuterDiameter(),m_bevelLength);
}

void usNeedleInsertionModelVirtualSprings::setDefaultSpringStiffness(double K)
{
    if(K > 0) m_defaultSpringStiffness = K;
}

double usNeedleInsertionModelVirtualSprings::getDefaultSpringStiffness() const
{
    return m_defaultSpringStiffness;
}

void usNeedleInsertionModelVirtualSprings::setStiffnessPerUnitLength(double K)
{
    if(K > 0)
    {
        m_stiffnessPerUnitLength = K;
        updateTipForce();
    }
    else m_stiffnessPerUnitLength = 0;
}

double usNeedleInsertionModelVirtualSprings::getStiffnessPerUnitLength() const
{
    return m_stiffnessPerUnitLength;
}

int usNeedleInsertionModelVirtualSprings::getNbSprings() const
{
    return m_springs.size();
}

int usNeedleInsertionModelVirtualSprings::getNbMeasureSprings() const
{
    int n=0;
    for(unsigned int i=0 ; i<m_springs.size() ; i++) if(!m_springs.at(i).IsPositionUpdateAllowed()) n++;
    return n;
}
const usNeedleModelSpline &usNeedleInsertionModelVirtualSprings::accessNeedle() const
{
    return m_needle;
}

usNeedleModelSpline &usNeedleInsertionModelVirtualSprings::accessNeedle()
{
    return m_needle;
}

vpColVector usNeedleInsertionModelVirtualSprings::getInsertionPoint() const
{
    return m_springs.front().getPosition();
}

double usNeedleInsertionModelVirtualSprings::getNeedleFreeLength() const
{
    return m_needle.accessSegment(0).getParametricLength();
}

double usNeedleInsertionModelVirtualSprings::getInsertionDepth() const
{
    if(m_needle.getNbSegments()>1) return m_needle.getFullLength()-m_needle.accessSegment(0).getParametricLength();
    else return 0;
}

const usOrientedPlane3D &usNeedleInsertionModelVirtualSprings::accessSurface() const
{
    return m_tissueSurface;
}

usOrientedPlane3D &usNeedleInsertionModelVirtualSprings::accessSurface()
{
    return m_tissueSurface;
}

const usVirtualSpring &usNeedleInsertionModelVirtualSprings::accessSpring(int i) const
{
    return m_springs.at(i);
}

void usNeedleInsertionModelVirtualSprings::setInterSpringDistance(double interSpringDistance)
{
    m_interSpringDistance = interSpringDistance;
}

double usNeedleInsertionModelVirtualSprings::getInterSpringDistance() const
{
    return m_interSpringDistance;
}

void usNeedleInsertionModelVirtualSprings::setInterTipSpringDistance(double interTipSpringDistance)
{
    if(interTipSpringDistance <= m_interSpringDistance) m_interTipSpringDistance = interTipSpringDistance;
    else m_interTipSpringDistance = m_interSpringDistance;
}

double usNeedleInsertionModelVirtualSprings::getInterTipSpringDistance() const
{
    return m_interTipSpringDistance;
}

void usNeedleInsertionModelVirtualSprings::setNbMinTipSprings(int nb)
{
    if(nb>0)
    {
        if(nb<=m_nbMaxTipSprings) m_nbMinTipSprings = nb;
        else m_nbMinTipSprings = m_nbMaxTipSprings;
    }
    else m_nbMinTipSprings = 1;
}

int usNeedleInsertionModelVirtualSprings::getNbMinTipSprings() const
{
    return m_nbMinTipSprings;
}

void usNeedleInsertionModelVirtualSprings::setNbMaxTipSprings(int nb)
{
    if(nb>0)
    {
        if(nb>=m_nbMinTipSprings) m_nbMaxTipSprings = nb;
        else m_nbMaxTipSprings = m_nbMaxTipSprings;
    }
    else m_nbMaxTipSprings = 1;
}

int usNeedleInsertionModelVirtualSprings::getNbMaxTipSprings() const
{
    return m_nbMaxTipSprings;
}

void usNeedleInsertionModelVirtualSprings::AllowSpringAddition(bool flag)
{
    m_AllowSpringAddition = flag;
}

void usNeedleInsertionModelVirtualSprings::AllowSpringRemoval(bool flag)
{
    m_AllowSpringRemoval = flag;
}

void usNeedleInsertionModelVirtualSprings::setInsertionBehavior(InsertionType type)
{
    m_insertionBehavior = type;
}

usNeedleInsertionModelVirtualSprings::InsertionType usNeedleInsertionModelVirtualSprings::getInsertionBehavior() const
{
    return m_insertionBehavior;
}

void usNeedleInsertionModelVirtualSprings::setAutomaticSpringAddition(bool flag)
{
    m_AutomaticSpringAddition = flag;
}

bool usNeedleInsertionModelVirtualSprings::getAutomaticSpringAddition() const
{
    return m_AutomaticSpringAddition;
}

double usNeedleInsertionModelVirtualSprings::getPathDistanceFromPoint(const vpColVector &P) const
{
    if(P.size() != 3) throw vpException(vpException::dimensionError, "usNeedleInsertionModelVirtualSprings::getPathDistanceFromPoint: invalid point dimension");

    double min = std::numeric_limits<double>::infinity();

    for(unsigned int i=0 ; i<m_springs.size() ; i++)
    {
        vpColVector p = usGeometryTools::projectPointOnPlane(P, m_springs.at(i));
        double d = (m_springs.at(i).getPosition()-p).euclideanNorm();
        if(d<min) min = d;
    }

    return min;
}

double usNeedleInsertionModelVirtualSprings::getTissueDeformationEnergy() const
{
    double E = 0;

    for(unsigned int n=0 ; n<m_springs.size() ; n++)
    {
        double K = m_springs.at(n).getStiffness();
        double l = (m_needle.accessSegment(n).getEndPoint() - m_springs.at(n).getPosition()).euclideanNorm();

        E += 0.5 * K * l*l;
    }

    return E;
}


double usNeedleInsertionModelVirtualSprings::getSurfaceTissueStretch() const
{
    double s = 0;
    double l = this->getNeedleFreeLength();
    if(l<this->accessNeedle().getParametricLength()) s = (this->accessNeedle().getPoint(l) - this->getInsertionPoint()).euclideanNorm();

    return s;
}

double usNeedleInsertionModelVirtualSprings::getMaxTissueStretch() const
{
    double max = 0;

    for(unsigned int i=0 ; i<m_springs.size() ; i++)
    {
        double d = (m_springs.at(i).getPosition()-m_needle.accessSegment(i).getEndPoint()).euclideanNorm();
        if(d>max) max = d;
    }

    return max;
}

double usNeedleInsertionModelVirtualSprings::getMeanTissueStretch() const
{
    double mean = 0;
    double L = this->getInsertionDepth();

    for(unsigned int i=0 ; i<m_springs.size() ; i++)
    {
        double d = (m_springs.at(i).getPosition()-m_needle.accessSegment(i).getEndPoint()).euclideanNorm();
        double l1 = m_needle.accessSegment(i).getParametricLength();
        if(i==0) l1=0;
        double l2 = m_needle.accessSegment(i+1).getParametricLength();
        mean += d * (l1+l2)/2;
    }
    if(L>std::numeric_limits<double>::epsilon()) mean /= L;

    return mean;
}

bool usNeedleInsertionModelVirtualSprings::setBasePose(const vpPoseVector &pose)
{
    // Set base position in worldframe if the base doesn't go under the tissue

    if(m_springs.size() >0)
    {
        vpColVector position(pose.getTranslationVector());
        if( usGeometryTools::IsPointInFrontOfPlane(position, m_tissueSurface) ) return false;
    }

    m_needle.setBasePose(pose);

    this->updateState();

    return true;
}

vpPoseVector usNeedleInsertionModelVirtualSprings::getBasePose() const
{
    return m_needle.getBasePose();
}

bool usNeedleInsertionModelVirtualSprings::setSpringPosition(int index, const vpColVector &P, bool update)
{
    if(index<0 || index>(int)m_springs.size()-1) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::setSpringPosition: invlaid spring index");

    if(P.size() != 3) throw vpException(vpException::dimensionError, "usNeedleInsertionModelVirtualSprings::setSpringPosition: bad vector dimension");

    m_springs.at(index).setPosition(P);
    if(update) this->updateState();

    return true;
}

bool usNeedleInsertionModelVirtualSprings::setSpringDirection(int index, const vpColVector &D, bool update)
{
    if(index<0 || index>(int)m_springs.size()-1) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::setSpringDirection: invalid spring index");
    if(D.size() != 3) throw vpException(vpException::dimensionError, "usNeedleInsertionModelVirtualSprings::setSpringDirection: invalid vector dimension");

    m_springs.at(index).setDirection(D);
    if(update) this->updateState();

    return true;
}

void usNeedleInsertionModelVirtualSprings::setSpringStiffness(int index, double K, bool update)
{
    if(index<0 || index>(int)m_springs.size()-1) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::setSpringStiffness: bad spring index");
    if(K <= 0) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::setSpringStiffness: negative stiffness");

    m_springs.at(index).setStiffness(K);
    if(update) this->updateState();
}

bool usNeedleInsertionModelVirtualSprings::moveSpringPosition(int index, const vpColVector &dP, bool update)
{
    return this->setSpringPosition(index, m_springs.at(index).getPosition()+dP, update);
}

bool usNeedleInsertionModelVirtualSprings::moveSpringDirection(int index, const vpThetaUVector &thetaU, bool update)
{
    vpRotationMatrix R(thetaU);
    return this->setSpringDirection(index, R*m_springs.at(index).getDirection(), update);
}

void usNeedleInsertionModelVirtualSprings::addSpringStiffness(int index, double dK, bool update)
{
    return this->setSpringStiffness(index, m_springs.at(index).getStiffness()+dK, update);
}

void usNeedleInsertionModelVirtualSprings::setSurfaceAtTip()
{
    vpColVector p = m_needle.accessLastSegment().getEndPoint();
    vpColVector d = m_needle.accessLastSegment().getEndTangent();
    p += m_bevelLength*d + m_needle.getOuterDiameter()/2 * vpColVector(vpHomogeneousMatrix(m_needle.getTipPose()).getCol(1), 0,3);
    m_tissueSurface.setPosition(p);
    m_tissueSurface.setDirection(d);
}

void usNeedleInsertionModelVirtualSprings::addInsertionPointOnSegmentHard(int segment, double s)
{
    if(segment<0 || segment>=m_needle.getNbSegments()) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::addInsertionPointOnSegment: invalid segment index");
    if( (s<=0) || ( (segment!=m_needle.getNbSegments()-1) && (s>m_needle.accessSegment(segment).getParametricLength()) ) ) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::addInsertionPointOnSegment: invalid segment length parameter");

    //if(segment == m_needle.getNbSegments()-1) && !m_LastSegmentLengthComputed) return;

    // Keep segment coef for the first subsegment and switch origine for the second

    usPolynomialCurve3D seg(m_needle.accessSegment(segment).getSubPolynomialCurve(s,m_needle.accessSegment(segment).getParametricLength()));

    seg.changeCoefficientsToFitBoundaries(0, seg.getParametricLength());

    m_needle.insertSegment(segment, seg);

    m_needle.accessSegment(segment).setParametricLength(s);

    
    if(segment == m_needle.getNbSegments()-2)
    {
        vpColVector bevelDirection(vpHomogeneousMatrix(m_needle.getTipPose()).getCol(1), 0,3);
        double l = (m_needle.getOuterDiameter()/2-(m_bevelLength+m_needle.accessLastSegment().getParametricLength())*tan(M_PI/180*m_cutAngle));
        if(l < 0) l = 0;
        usVirtualSpring spg(m_needle.accessLastSegment().getStartPoint()+l* bevelDirection, m_needle.accessSegment(segment).getEndTangent(), m_defaultSpringStiffness);
        m_springs.insert(m_springs.begin()+segment, spg);
    }
    else
    {
        usVirtualSpring spg(m_needle.accessSegment(segment).getEndPoint(), m_needle.accessSegment(segment).getEndTangent(), m_defaultSpringStiffness);
        m_springs.insert(m_springs.begin()+segment, spg);
    }

}

void usNeedleInsertionModelVirtualSprings::addInsertionPointAtTipHard()
{
    this->addInsertionPointOnSegmentHard(m_needle.getNbSegments()-1, m_needle.accessLastSegment().getParametricLength());
}

void usNeedleInsertionModelVirtualSprings::addInsertionPointOnSegment(int segment, double s)
{
    if(segment<0 || segment>=m_needle.getNbSegments()) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::addInsertionPointOnSegment: bad segment index");
    if( (s<=0) || ( (segment!=m_needle.getNbSegments()-1) && (s>m_needle.accessSegment(segment).getParametricLength()) ) ) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::addInsertionPointOnSegment: bad segment length parameter");

    double Lseg = m_needle.accessSegment(segment).getParametricLength();
    double l2 = Lseg - s;

    if(segment == m_needle.getNbSegments()-1) // spring added on last segment
    {
        double K = 0;
        if(segment == 0) // if first spring added
        {
            K = m_stiffnessPerUnitLength*l2;
        }
        else
        {
            double L1 = m_needle.accessSegment(segment-1).getParametricLength(); // length of tissue before current last spring
            double L = m_needle.accessLastSegment().getParametricLength() + m_bevelLength; // length of tissue after current last spring
            if(segment == 1) L1 = 0;
            K = m_springs.at(segment-1).getStiffness() * (L-s/2) / (L1/2 + L);
            m_springs.at(segment-1).addStiffness(-K);
        }

        this->addInsertionPointOnSegmentHard(segment,s);
        m_springs.back().setStiffness(K);
    }
    else // spring added on intermediate segment => interpolation
    {
        double c = s/Lseg;
        vpColVector x0 = m_springs.at(segment-1).getPosition();
        vpColVector x1 = m_springs.at(segment).getPosition();
        vpColVector d0 = m_springs.at(segment-1).getDirection();
        vpColVector d1 = m_springs.at(segment).getDirection();

        vpColVector P = c*x1 + (1-c)*x0;
        vpColVector D = c*d1 + (1-c)*d0;

        double L1 = m_needle.accessSegment(segment-1).getParametricLength();
        if(segment-1 == 0) L1 = 0;
        double L = m_needle.accessSegment(segment).getParametricLength();
        double L2 = m_needle.accessSegment(segment+1).getParametricLength();
        if(segment+1 == m_needle.getNbSegments()-1) L2 = 2*(m_needle.accessLastSegment().getParametricLength() + m_bevelLength );
        double K1 = m_springs.at(segment-1).getStiffness() * l2 / (L1 + L);
        double K2 = m_springs.at(segment).getStiffness() * s / (L + L2);
        m_springs.at(segment-1).addStiffness(-K1);
        m_springs.at(segment).addStiffness(-K2);

        this->addInsertionPointOnSegmentHard(segment,s);
        m_springs.at(segment).setPosition(P);
        m_springs.at(segment).setDirection(D);
        m_springs.at(segment).setStiffness(K1+K2);
    }
}

int usNeedleInsertionModelVirtualSprings::addInsertionPoint(usVirtualSpring spg)
{
    int segment = 0;

    while(segment<m_needle.getNbSegments() && !usGeometryTools::DoesSegmentCrossPlaneDirect(m_needle.accessSegment(segment), spg) ) segment++;

    if(segment == 0 && m_springs.size()!=0) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::addInsertionPoint: cannot add spring on first segment");
    if(segment==m_needle.getNbSegments()) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::addInsertionPoint: spring does not cross the needle");

    double Lseg = m_needle.accessSegment(segment).getParametricLength();
    double s = -1;
    if(usGeometryTools::DoesSegmentCrossPlane(m_needle.accessSegment(segment), spg))
    {
        usGeometryTools::getPlaneCurveCrossingPoint(m_needle.accessSegment(segment), spg, -1, &s);
    }
    else s = Lseg/2;

    double l2 = Lseg - s;

    if(segment == m_needle.getNbSegments()-1) // spring added on last segment
    {
        double K = 0;
        if(segment == 0) // if first spring added
        {
            K = 2*m_stiffnessPerUnitLength*(l2+m_bevelLength);
        }
        else
        {
            double L1 = m_needle.accessSegment(segment-1).getParametricLength(); // length of tissue before current last spring
            double L = m_needle.accessLastSegment().getParametricLength() + m_bevelLength; // length of tissue after current last spring
            if(segment == 1) L1 = 0;
            K = m_springs.at(segment-1).getStiffness() * (L-s/2) / (L1/2 + L);
            m_springs.at(segment-1).addStiffness(-K);
        }
        spg.setStiffness(K);
    }
    else // spring added on intermediate segment => interpolation
    {
        double L1 = m_needle.accessSegment(segment-1).getParametricLength();
        if(segment-1 == 0) L1 = 0;
        double L = m_needle.accessSegment(segment).getParametricLength();
        double L2 = m_needle.accessSegment(segment+1).getParametricLength();
        if(segment+1 == m_needle.getNbSegments()-1) L2 = 2*(m_needle.accessLastSegment().getParametricLength() + m_bevelLength );
        double K1 = m_springs.at(segment-1).getStiffness() * l2 / (L1 + L);
        double K2 = m_springs.at(segment).getStiffness() * s / (L + L2);
        m_springs.at(segment-1).addStiffness(-K1);
        m_springs.at(segment).addStiffness(-K2);
        spg.setStiffness(K1+K2);
    }

    this->addInsertionPointOnSegmentHard(segment,s);
    m_springs.at(segment) = spg;
    
    return segment;
}

int usNeedleInsertionModelVirtualSprings::addInsertionPoint(const vpColVector &p, const vpColVector &d)
{
    if(p.size() != 3 || d.size() != 3) throw vpException(vpException::dimensionError, "usNeedleInsertionModelVirtualSprings::addInsertionPoint: invalid vector dimension");

    unsigned int index = 0;
    while(index<m_springs.size() && usGeometryTools::IsPointInFrontOfPlane(p, m_springs.at(index))) index++;

    if(index == 0 && m_springs.size()!=0)
    {
        std::cout << "Warning in usNeedleInsertionModelVirtualSprings::addInsertionPoint: add spring on first segment" << std::endl;
    }

    usVirtualSpring spg(p, d, 0);

    double Lseg = m_needle.accessSegment(index).getParametricLength();
    double s = 0;

    if(usGeometryTools::IsPointInFrontOfPlane(m_needle.accessSegment(index).getStartPoint(), spg) && !usGeometryTools::IsPointInFrontOfPlane(m_needle.accessSegment(index).getEndPoint(), spg))
    {
        spg.setDirection(-spg.getDirection());
        s = Lseg/2;
    }
    else if(usGeometryTools::IsPointInFrontOfPlane(m_needle.accessSegment(index).getStartPoint(), spg) && usGeometryTools::IsPointInFrontOfPlane(m_needle.accessSegment(index).getEndPoint(), spg))
    {
        s = m_interTipSpringDistance;

        vpColVector P = m_needle.accessSegment(index).getPoint(s);
        spg.setDirection(vpColVector::crossProd(P-p, vpColVector::crossProd( d, P-p)).normalize());
    }
    else if(!usGeometryTools::IsPointInFrontOfPlane(m_needle.accessSegment(index).getStartPoint(), spg) && !usGeometryTools::IsPointInFrontOfPlane(m_needle.accessSegment(index).getEndPoint(), spg))
    {
        s = Lseg - m_interTipSpringDistance;

        vpColVector P = m_needle.accessSegment(index).getPoint(s);
        spg.setDirection(vpColVector::crossProd(P-p, vpColVector::crossProd( d, P-p)).normalize());
    }
    else
    {
        s = Lseg/2;
    }

    double l2 = Lseg - s;

    int segment = index;

    if(segment == m_needle.getNbSegments()-1) // spring added on last segment
    {
        double K = 0;
        if(segment == 0) // if first spring added
        {
            K = 2*m_stiffnessPerUnitLength*(l2+m_bevelLength);
        }
        else
        {
            double L1 = m_needle.accessSegment(segment-1).getParametricLength(); // length of tissue before current last spring
            double L = m_needle.accessLastSegment().getParametricLength() + m_bevelLength; // length of tissue after current last spring
            if(segment == 1) L1 = 0;
            double K = m_springs.at(segment-1).getStiffness() * (L-s/2) / (L1/2 + L);
            m_springs.at(segment-1).addStiffness(-K);
        }
        spg.setStiffness(K);
    }
    else // spring added on intermediate segment => interpolation
    {
        double L1 = m_needle.accessSegment(segment-1).getParametricLength();
        if(segment-1 == 0) L1 = 0;
        double L = m_needle.accessSegment(segment).getParametricLength();
        double L2 = m_needle.accessSegment(segment+1).getParametricLength();
        if(segment+1 == m_needle.getNbSegments()-1) L2 = 2*(m_needle.accessLastSegment().getParametricLength() + m_bevelLength );
        double K1 = m_springs.at(segment-1).getStiffness() * l2 / (L1 + L);
        double K2 = m_springs.at(segment).getStiffness() * s / (L + L2);
        m_springs.at(segment-1).addStiffness(-K1);
        m_springs.at(segment).addStiffness(-K2);
        spg.setStiffness(K1+K2);
    }

    this->addInsertionPointOnSegmentHard(segment,s);
    m_springs.at(segment) = spg;
    
    return segment;
}

void usNeedleInsertionModelVirtualSprings::addInsertionPointAtTip()
{
    if(m_needle.getNbSegments()==1) this->addInsertionPointOnSegment(m_needle.getNbSegments()-1, m_needle.accessLastSegment().getParametricLength());
    else this->addInsertionPointOnSegment(m_needle.getNbSegments()-1, 0.98*m_needle.accessLastSegment().getParametricLength());
}

void usNeedleInsertionModelVirtualSprings::removeInsertionPointsHard(int first, int last)
{
    if(last == -1) last = first; // remove only one point

    if(first <0 || (last<first) || last >= (int)m_springs.size()) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::removeInsertionPointHard: bad index, cannot remove spring");

    // Remark: erase doesn't erase the last element => add 1 to second argument
    m_springs.erase(m_springs.begin()+first, m_springs.begin()+last+1);

    double l = m_needle.accessSegment(first).getParametricLength();
    for(int i=first+1 ; i<=last+1 ; i++)
    {
        l += m_needle.accessSegment(i).getParametricLength();
    }
    m_needle.accessSegment(first).setParametricLength(l);
    m_needle.removeSegments(first+1, last+1);
}

void usNeedleInsertionModelVirtualSprings::removeLastInsertionPointHard()
{
    this->removeInsertionPointsHard(m_springs.size()-1);
}

void usNeedleInsertionModelVirtualSprings::removeInsertionPoints(int first, int last)
{
    if(last == -1) last = first; // remove only one point

    if(first <0 || (last<first) || last >= (int)m_springs.size()) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::removeInsertionPoint: bad index, cannot remove spring");
    if( (first == 0) && (last != ((int)m_springs.size()-1)) ) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::removeInsertionPoint: cannot remove first insertion point without removing all insertion points");

    // Remark: erase doesn't erase the last element => add 1 to second argument

    // if the last spring is not removed we can simply fusion the springs
    if(last < (int)m_springs.size()-1)
    {
        this->fusionSprings(first-1, last+1);
    }
    // else if all springs must be removed
    else if(first == 0)
    {
        for(int i=first ; i<=last ; i++)
        {
            if(!m_springs.at(i).IsPositionUpdateAllowed()) m_inactiveMeasureSprings.push_back(m_springs.at(i));
        }
        this->removeInsertionPointsHard(first, last);
    }
    // else we can fusion springs and remove the last
    else
    {
        if(last > first) this->fusionSprings(first-1, last);

        double L1 = m_needle.accessSegment(m_needle.getNbSegments() - 3).getParametricLength(); // length of tissue before last spring that will remain
        double L = m_needle.accessSegment(m_needle.getNbSegments() - 2).getParametricLength(); // length of tissue before current last spring
        double L2 = m_needle.accessLastSegment().getParametricLength() + m_bevelLength; // length of tissue after current last spring
        if(m_springs.size() == 2) L1 = 0;
        m_springs.at(m_springs.size() - 2).setStiffness( (m_springs.at(m_springs.size() - 2).getStiffness() + m_springs.back().getStiffness())* (L1/2 + L2) / (L1/2 + L + L2) );

        if(!m_springs.back().IsPositionUpdateAllowed()) m_inactiveMeasureSprings.push_back(m_springs.back());
        this->removeLastInsertionPointHard();
    }
}

void usNeedleInsertionModelVirtualSprings::removeLastInsertionPoint()
{
    this->removeInsertionPoints(m_springs.size()-1);
}

void usNeedleInsertionModelVirtualSprings::removeAutoAddedSprings()
{
    for(int spg=m_springs.size()-1 ; spg>0 ; spg--)
    {
        if(m_springs.at(spg).IsPositionUpdateAllowed()) this->removeInsertionPoints(spg);
    }
    m_tipSpringsIndex = m_springs.size()-1;
}

void usNeedleInsertionModelVirtualSprings::fusionSprings(int firstSpring, int lastSpring)
{
    int nbSprings = lastSpring-firstSpring+1;
    if(firstSpring<0 || nbSprings<3 || lastSpring>=(int)m_springs.size()) throw vpException(vpException::badValue, "usNeedleInsertionModelVirtualSprings::fusionSprings: bad springs indexes");

    if(nbSprings == 3)
    {
        double L1 = m_needle.accessSegment(firstSpring).getParametricLength();
        if(firstSpring==0) L1 = 0;
        double l = m_needle.accessSegment(firstSpring+1).getParametricLength();
        double l2 = m_needle.accessSegment(lastSpring).getParametricLength();
        double L2 = m_needle.accessSegment(lastSpring+1).getParametricLength();

        if(lastSpring == (int)m_springs.size()-1) L2 = 2*(m_needle.accessLastSegment().getParametricLength()+m_bevelLength);

        double K0 = m_springs.at(firstSpring).getStiffness();
        double K1 = m_springs.at(firstSpring+1).getStiffness();
        double K2 = m_springs.at(lastSpring).getStiffness();
        double w0 = K0 * l2 / (L1 + l);
        double w2 = K2 * l / (l2 + L2);

        m_springs.at(firstSpring).addStiffness( K1 * w0 / (w0 + w2) );
        m_springs.at(lastSpring).addStiffness( K1 * w2 / (w0 + w2) );


        if(!m_springs.at(firstSpring+1).IsPositionUpdateAllowed()) m_inactiveMeasureSprings.push_back(m_springs.at(firstSpring+1));
        this->removeInsertionPointsHard(firstSpring+1);
    }
    else
    {
        int middle = (firstSpring + lastSpring) / 2;
        this->fusionSprings(middle, lastSpring);
        this->fusionSprings(firstSpring, middle+1);
    }
}

void usNeedleInsertionModelVirtualSprings::updateSpringsStiffness()
{
    for(unsigned int i=0 ; i<m_springs.size() ; i++)
    {
        double L1 = m_needle.accessSegment(i).getParametricLength()/2;
        double L2 = m_needle.accessSegment(i+1).getParametricLength()/2;

        if(i==0) L1 = 0;
        if((int)i+1==(int)m_needle.getNbSegments()-1) L2 = m_needle.accessLastSegment().getParametricLength();//+m_bevelLength;

        m_springs.at(i).setStiffness(m_stiffnessPerUnitLength * (L1+L2));
    }
}

void usNeedleInsertionModelVirtualSprings::updateCutAngle()
{
    if(m_springs.size()>0)
    {
        vpColVector z = m_springs.back().getDirection();
        if(m_springs.size()>1) z = (m_springs.back().getPosition()-m_springs.at(m_springs.size()-2).getPosition()).normalize();

        vpHomogeneousMatrix worldMtip(m_needle.getWorldMtip());
        vpColVector ztip(worldMtip.getCol(2), 0,3);
        vpColVector xtip(worldMtip.getCol(0), 0,3);
        
        ztip = (ztip - vpColVector::dotProd(ztip, xtip) * xtip).normalize();
        z = (z - vpColVector::dotProd(z, xtip) * xtip).normalize();

        double vect =  vpColVector::crossProd(z, ztip).euclideanNorm();
        double dot = vpColVector::dotProd(z, ztip);
        m_cutAngle = 180/M_PI*atan2(vect, dot);
    }
    else
    {
        m_cutAngle = 0;
    }
    //m_cutAngle = 0;
}

void usNeedleInsertionModelVirtualSprings::updateTipForce()
{
    double bevel_rad = atan2(m_needle.getOuterDiameter(), m_bevelLength);
    double cut_rad = M_PI/180*m_cutAngle;

    double a = m_needle.getOuterDiameter() / tan(bevel_rad);
    double b = m_needle.getOuterDiameter() / sin(bevel_rad);

    m_tipForce = 0.5*m_stiffnessPerUnitLength *
                (   a*a * tan(cut_rad) / 2
                  - b*b * cos(bevel_rad) * tan(bevel_rad-cut_rad) / 2
                );

    m_tipMoment = 0.5*m_stiffnessPerUnitLength *
                ( - a*a*a * tan(cut_rad) / 6
                  + b*b * tan(bevel_rad-cut_rad) / 2 * (a * cos(bevel_rad) / 3 - m_needle.getOuterDiameter() * sin(bevel_rad) / 6)
                );
}

void usNeedleInsertionModelVirtualSprings::updateInsertionDirections()
{
    for(int i=0 ; i< m_tipSpringsIndex ; i++)
    {
        if(!m_springs.at(i).IsDirectionUpdateAllowed()) continue;
        m_springs.at(i).setDirection(m_needle.accessSegment(i).getEndTangent());
    }

    for(unsigned int i=m_tipSpringsIndex ; i< m_springs.size() ; i++)
    {
        if(!m_springs.at(i).IsDirectionUpdateAllowed()) continue;
        m_springs.at(i).setDirection(m_needle.accessSegment(i).getEndTangent());
    }
}

#ifdef VISP_HAVE_EIGEN3
void usNeedleInsertionModelVirtualSprings::solveSegmentsParametersSparseEigen()
{
    int nbSeg = m_needle.getNbSegments();

    // Set system to solve :
    //
    //    M * a = b
    //
    //  a : 12 nbSeg parameters
    //  b : 12 nbSeg constraints
    //  M : 12 nbSeg x 12 nbSeg matrix
    //

    typedef Eigen::Triplet<double> T;
    std::vector<T> L;
    L.reserve(144/10*nbSeg*nbSeg);

    Eigen::VectorXd a = Eigen::VectorXd::Zero(12*nbSeg);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(12*nbSeg);

    int line = 0;
    int col = 0;

    double EI = m_needle.getEI();
    vpPoseVector basePose(m_needle.getBasePose());
    vpHomogeneousMatrix worldMbase(m_needle.getWorldMbase());
    vpHomogeneousMatrix worldMtip(m_needle.getWorldMtip());

    //Continuity
    for(int n=0 ; n<nbSeg-1 ; n++)
    {
        double s = m_needle.accessSegment(n).getParametricLength();

        for(int dim=0 ; dim<3 ; dim++)
        {
            // Order 0 (continuity) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;
            L.push_back(T(line,col  ,1));
            L.push_back(T(line,col+1,s));
            L.push_back(T(line,col+2,s*s));
            L.push_back(T(line,col+3,s*s*s));

            col = 12*(n+1)+4*dim;
            L.push_back(T(line,col,-1));

            line++;

            //Order 1 (continuity of first derivative) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;

            L.push_back(T(line,col+1  ,1));
            L.push_back(T(line,col+2,2*s));
            L.push_back(T(line,col+3,3*s*s));

            col = 12*(n+1)+4*dim;
            L.push_back(T(line,col+1,-1));

            line++;

            //Order 2 (continuity of second derivative) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;

            L.push_back(T(line,col+2  ,2));
            L.push_back(T(line,col+3,6*s));

            col = 12*(n+1)+4*dim;
            L.push_back(T(line,col+2,-2));

            line++;
        }
    }

    vpColVector tipForceVector(3, 0);
    vpColVector tipMomentVector(3, 0);

    if(m_needle.getNbSegments()>1 && m_IsInserting)
    {
        vpColVector xtip(3);
        xtip[0] = worldMtip[0][0];
        xtip[1] = worldMtip[1][0];
        xtip[2] = worldMtip[2][0];

        vpColVector z = m_springs.back().getDirection();
        vpColVector y = vpColVector::crossProd(z,xtip);
        y.normalize();
        tipForceVector = -y;
        tipMomentVector = -y;

        tipForceVector *= m_tipForce;
        tipMomentVector *= m_tipMoment;
    }

    for(int dim=0 ; dim<3 ; dim++)
    {
        // Base conditions : 6 constraints
        col = 4*dim;

        L.push_back(T(line,col,1));
        b(line) = basePose[dim];
        line ++;

        L.push_back(T(line,col+1,1));
        b(line) = worldMbase[dim][2];
        line++;

        // Tip conditions : 6 constraints
        col = 12*(nbSeg-1)+4*dim;

        L.push_back(T(line,col+3,6*EI));
        b(line) = -tipForceVector[dim];

        line ++;

        L.push_back(T(line,col+3  ,6*m_needle.accessLastSegment().getParametricLength()*EI));
        L.push_back(T(line,col+2,2*EI));
        b(line) = -tipMomentVector[dim];
        line++;
    }

    // Springs : 3 * (nbSeg-1) constraints
    for(int n=0 ; n<nbSeg-1 ; n++)
    {
        // Force calculation
        double px_n = m_springs.at(n).getPosition()[0];
        double py_n = m_springs.at(n).getPosition()[1];
        double pz_n = m_springs.at(n).getPosition()[2];
        double dx_n = m_springs.at(n).getDirection()[0];
        double dy_n = m_springs.at(n).getDirection()[1];
        double dz_n = m_springs.at(n).getDirection()[2];

        // Build an orthonormal base for the plan normal to the spring direction
        vpColVector n1m_n(3);
        if( (fabs(dx_n) > std::numeric_limits<double>::epsilon() ) || (fabs(dy_n)  > std::numeric_limits<double>::epsilon()) )
        {
            vpColVector z(3);
            z[0] = 0;
            z[1] = 0;
            z[2] = 1;
            n1m_n = vpColVector::crossProd(m_springs.at(n).getDirection(), z);
        }
        else
        {
            vpColVector y(3);
            y[0] = 0;
            y[1] = 1;
            y[2] = 0;
            n1m_n = vpColVector::crossProd(m_springs.at(n).getDirection(), y);
        }
        n1m_n.normalize();

        double n1x_n = n1m_n[0];
        double n1y_n = n1m_n[1];
        double n1z_n = n1m_n[2];

        vpColVector n2m_n = vpColVector::cross(m_springs.at(n).getDirection(), n1m_n);
        n2m_n.normalize();

        double n2x_n = n2m_n[0];
        double n2y_n = n2m_n[1];
        double n2z_n = n2m_n[2];

        col = 12*n;

        L.push_back(T(line  ,col+3  ,6*EI * n1x_n));
        L.push_back(T(line  ,col+3+4,6*EI * n1y_n));
        L.push_back(T(line  ,col+3+8,6*EI * n1z_n));
        L.push_back(T(line+1,col+3  ,6*EI * n2x_n));
        L.push_back(T(line+1,col+3+4,6*EI * n2y_n));
        L.push_back(T(line+1,col+3+8,6*EI * n2z_n));
        for(int i=0 ; i<nbSeg-1-n ; i++)
        {
            double K = m_springs.at(n+i).getStiffness();
            vpColVector p = m_springs.at(n+i).getPosition();
            double dot1 = vpColVector::dotProd(p,n1m_n);
            L.push_back(T(line,col+12*(i+1)  ,-K*n1x_n));
            L.push_back(T(line,col+12*(i+1)+4,-K*n1y_n));
            L.push_back(T(line,col+12*(i+1)+8,-K*n1z_n));
            b(line) += -K*dot1;

            double dot2 = vpColVector::dotProd(p,n2m_n);
            L.push_back(T(line+1,col+12*(i+1)  ,-K*n2x_n));
            L.push_back(T(line+1,col+12*(i+1)+4,-K*n2y_n));
            L.push_back(T(line+1,col+12*(i+1)+8,-K*n2z_n));
            b(line+1) += -K*dot2;
        }
        b(line) += -vpColVector::dotProd(tipForceVector, n1m_n);
        b(line+1) += -vpColVector::dotProd(tipForceVector, n2m_n);

        line+=2;

        // Points stay in the normal plan of the spring
        col = 12*(n+1);
        double dotProd = dx_n*px_n+dy_n*py_n+dz_n*pz_n;

        L.push_back(T(line,col  ,dx_n));
        L.push_back(T(line,col+4,dy_n));
        L.push_back(T(line,col+8,dz_n));
        b(line) = dotProd;
        line++;
    }

    Eigen::SparseMatrix<double> M(12*nbSeg, 12*nbSeg);
    M.setFromTriplets(L.begin(), L.end());

    Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
    solver.compute(M);

    a = solver.solve(b);

    // Update needle parameters
    vpMatrix coef(3,4);
    for(int n=0 ; n<nbSeg ; n++)
    {

        for(int dim=0 ; dim<3 ; dim++)
        {
            int seg = 12*n + 4*dim;
            coef[dim][0] = a(seg  );
            coef[dim][1] = a(seg+1);
            coef[dim][2] = a(seg+2);
            coef[dim][3] = a(seg+3);

            m_needle.accessSegment(n).setPolynomialCoefficients(coef);
        }
    }

    // Compute new tip pose
    vpRotationMatrix worldRbase(worldMbase);

    vpRotationMatrix worldRtip(worldRbase);

    for(int i=0 ; i<m_needle.getNbSegments() ; i++)
    {
        vpColVector vect =  vpColVector::crossProd(m_needle.accessSegment(i).getStartTangent(), m_needle.accessSegment(i).getEndTangent());
        double dot = vpColVector::dotProd(m_needle.accessSegment(i).getStartTangent(), m_needle.accessSegment(i).getEndTangent());
        double theta = atan2(vect.euclideanNorm(), dot);
        vect.normalize();
        vect = theta * vect;
        vpThetaUVector thetaU(vect[0], vect[1], vect[2]);
        vpRotationMatrix Ri(thetaU);
        worldRtip = Ri * worldRtip;
    }

    vpColVector t = m_needle.accessLastSegment().getEndPoint();
    vpTranslationVector translation(t[0], t[1], t[2]);

    vpPoseVector tipPose(translation, worldRtip);
    m_needle.setTipPose(tipPose);
}

#elif VISP_HAVE_OPENCV
void usNeedleInsertionModelVirtualSprings::solveSegmentsParametersOpenCV()
{
    int nbSeg = m_needle.getNbSegments();

    // Set system to solve :
    //
    //    M * a = b
    //
    //  a : 12 nbSeg parameters
    //  b : 12 nbSeg constraints
    //  M : 12 nbSeg x 12 nbSeg matrix
    //

    cv::Mat M = cv::Mat::zeros(12*nbSeg, 12*nbSeg, CV_64F);
    cv::Mat b = cv::Mat::zeros(12*nbSeg, 1, CV_64F);
    cv::Mat a = cv::Mat::zeros(12*nbSeg, 1, CV_64F);

    int line = 0;
    int col = 0;

    //Continuity
    for(int n=0 ; n<nbSeg-1 ; n++)
    {
        double s = m_needle.accessSegment(n).getParametricLength();

        for(int dim=0 ; dim<3 ; dim++)
        {
            // Order 0 (continuity) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;
            M.at<double>(line, col) = s*s*s;
            M.at<double>(line, col+1) = s*s;
            M.at<double>(line, col+2) = s;
            M.at<double>(line, col+3) = 1;

            col = 12*(n+1)+4*dim;
            M.at<double>(line, col  ) = 0;
            M.at<double>(line, col+1) = 0;
            M.at<double>(line, col+2) = 0;
            M.at<double>(line, col+3) = -1;

            line++;

            //Order 1 (continuity of first derivative) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;

            M.at<double>(line, col  ) = 3*s*s;
            M.at<double>(line, col+1) = 2*s;
            M.at<double>(line, col+2) = 1;
            M.at<double>(line, col+3) = 0;

            col = 12*(n+1)+4*dim;
            M.at<double>(line, col  ) = 0;
            M.at<double>(line, col+1) = 0;
            M.at<double>(line, col+2) = -1;
            M.at<double>(line, col+3) = 0;

            line++;

            //Order 2 (continuity of second derivative) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;

            M.at<double>(line, col  ) = 6*s;
            M.at<double>(line ,col+1) = 2;
            M.at<double>(line, col+2) = 0;
            M.at<double>(line, col+3) = 0;

            col = 12*(n+1)+4*dim;
            M.at<double>(line, col  ) = 0;
            M.at<double>(line, col+1) = -2;
            M.at<double>(line, col+2) = 0;
            M.at<double>(line, col+3) = 0;

            line++;
        }
    }

    vpColVector tipForceVector(3, 0);
    vpColVector tipMomentVector(3, 0);

    if(m_needle.getNbSegments()>1 && m_IsInserting)
    {
        vpColVector xtip(3);
        xtip[0] = m_worldMtip[0][0];
        xtip[1] = m_worldMtip[1][0];
        xtip[2] = m_worldMtip[2][0];

        vpColVector z = m_springs.back().getDirection();
        vpColVector y = vpColVector::crossProd(z,xtip);
        y.normalize();
        tipForceVector = y;
        tipMomentVector = y;

        tipForceVector *= m_tipForce;
        tipMomentVector *= m_tipMoment;
    }

    for(int dim=0 ; dim<3 ; dim++)
    {
        // Base conditions : 6 constraints
        col = 4*dim;

        M.at<double>(line, col+3) = 1;
        b.at<double>(line,0) = m_basePose[dim];
        m_inputBasePositionIndex[dim] = line;
        line ++;

        M.at<double>(line, col+2) = 1;
        b.at<double>(line, 0) = m_worldMbase[dim][2];
        m_inputBaseDirectionIndex[dim] = line;
        line++;

        // Tip conditions : 6 constraints
        col = 12*(nbSeg-1)+4*dim;

        M.at<double>(line, col) = 6*m_EI;
        b.at<double>(line, 0) = -tipForceVector[dim];

        line ++;

        M.at<double>(line, col) = 6*m_needle.accessLastSegment().getParametricLength();
        M.at<double>(line, col+1) = 2;
        b.at<double>(line) = -tipMomentVector[dim];
        line++;
    }

    // Springs : 3 * (nbSeg-1) constraints
    for(int n=0 ; n<nbSeg-1 ; n++)
    {
        // Force calculation
        double px_n = m_springs.at(n).getPosition()[0];
        double py_n = m_springs.at(n).getPosition()[1];
        double pz_n = m_springs.at(n).getPosition()[2];
        double dx_n = m_springs.at(n).getDirection()[0];
        double dy_n = m_springs.at(n).getDirection()[1];
        double dz_n = m_springs.at(n).getDirection()[2];

        // Build an orthonormal base for the plan normal to the spring direction
        vpColVector n1m_n(3);
        if( (fabs(dx_n) > std::numeric_limits<double>::epsilon() ) || (fabs(dy_n)  > std::numeric_limits<double>::epsilon()) )
        {
            vpColVector z(3);
            z[0] = 0;
            z[1] = 0;
            z[2] = 1;
            n1m_n = vpColVector::crossProd(m_springs.at(n).getDirection(), z);
        }
        else
        {
            vpColVector y(3);
            y[0] = 0;
            y[1] = 1;
            y[2] = 0;
            n1m_n = vpColVector::crossProd(m_springs.at(n).getDirection(), y);
        }
        n1m_n.normalize();

        double n1x_n = n1m_n[0];
        double n1y_n = n1m_n[1];
        double n1z_n = n1m_n[2];

        vpColVector n2m_n = vpColVector::cross(m_springs.at(n).getDirection(), n1m_n);
        n2m_n.normalize();

        double n2x_n = n2m_n[0];
        double n2y_n = n2m_n[1];
        double n2z_n = n2m_n[2];

        col = 12*n;

        M.at<double>(line, col) = 6*m_EI * n1x_n;
        M.at<double>(line, col+4) = 6*m_EI * n1y_n;
        M.at<double>(line, col+8) = 6*m_EI * n1z_n;
        M.at<double>(line+1, col) = 6*m_EI * n2x_n;
        M.at<double>(line+1, col+4) = 6*m_EI * n2y_n;
        M.at<double>(line+1, col+8) = 6*m_EI * n2z_n;
        for(int i=0 ; i<nbSeg-1-n ; i++)
        {
            double K = m_springs.at(n+i).getStiffness();
            vpColVector p = m_springs.at(n+i).getPosition();

            M.at<double>(line, col+12*(i+1)+3) = -K*n1x_n;
            M.at<double>(line, col+12*(i+1)+3+4) = -K*n1y_n;
            M.at<double>(line, col+12*(i+1)+3+8) = -K*n1z_n;
            b.at<double>(line, 0) += -K*(p[0]*n1x_n+p[1]*n1y_n+p[2]*n1z_n);

            M.at<double>(line+1, col+12*(i+1)+3) = -K*n2x_n;
            M.at<double>(line+1, col+12*(i+1)+3+4) = -K*n2y_n;
            M.at<double>(line+1, col+12*(i+1)+3+8) = -K*n2z_n;
            b.at<double>(line+1, 0) += -K*(p[0]*n2x_n+p[1]*n2y_n+p[2]*n2z_n);

        }
        b.at<double>(line, 0) += -vpColVector::dotProd(tipForceVector, n1m_n);
        b.at<double>(line+1, 0) += -vpColVector::dotProd(tipForceVector, n2m_n);
        line+=2;

        // Points stay in the normal plan of the spring
        col = 12*(n+1);
        double dotProd = dx_n*px_n+dy_n*py_n+dz_n*pz_n;

        M.at<double>(line, col+0+3) = dx_n;
        M.at<double>(line, col+4+3) = dy_n;
        M.at<double>(line, col+8+3) = dz_n;
        b.at<double>(line, 0) = dotProd;
        line++;
    }

    // Solve system

    cv::solve(M,b,a);

    // Update needle parameters
    vpMatrix coef(3,4);
    for(int n=0 ; n<nbSeg ; n++)
    {

        for(int dim=0 ; dim<3 ; dim++)
        {
            int seg = 12*n + 4*dim;
            coef[dim][0] = a.at<double>(seg  );
            coef[dim][1] = a.at<double>(seg+1);
            coef[dim][2] = a.at<double>(seg+2);
            coef[dim][3] = a.at<double>(seg+3);

            m_needle.accessSegment(n).setPolynomialCoefficients(coef);
            m_needle.accessSegment(n).update();
        }
    }

    // Compute new tip pose
    vpRotationMatrix worldRbase(m_worldMbase);

    vpRotationMatrix worldRtip(worldRbase);

    for(int i=0 ; i<m_needle.getNbSegments() ; i++)
    {
        vpColVector vect =  vpColVector::crossProd(m_needle.accessSegment(i).getStartTangent(), m_needle.accessSegment(i).getEndTangent());
        double dot = vpColVector::dotProd(m_needle.accessSegment(i).getStartTangent(), m_needle.accessSegment(i).getEndTangent());
        double theta = atan2(vect.euclideanNorm(), dot);
        vect.normalize();
        vect = theta * vect;
        vpThetaUVector thetaU(vect[0], vect[1], vect[2]);
        vpRotationMatrix Ri(thetaU);
        worldRtip = Ri * worldRtip;
    }

    vpColVector t = m_needle.accessLastSegment().getEndPoint();
    vpTranslationVector translation(t[0], t[1], t[2]);

    m_worldMtip.buildFrom(translation, worldRtip);
    m_tipPose.buildFrom(m_worldMtip);
}

#else
void usNeedleInsertionModelVirtualSprings::solveSegmentsParametersViSP()
{
    int nbSeg = m_needle.getNbSegments();

    // Set system to solve :
    //
    //    M * a = b
    //
    //  a : 12 nbSeg parameters
    //  b : 12 nbSeg constraints
    //  M : 12 nbSeg x 12 nbSeg matrix
    //

    vpMatrix M(12*nbSeg,12*nbSeg,0);
    vpColVector a(12*nbSeg,0);
    vpColVector b(12*nbSeg,0);

    int line = 0;
    int col = 0;

    //Continuity
    for(int n=0 ; n<nbSeg-1 ; n++)
    {
        double s = m_needle.accessSegment(n).getParametricLength();

        for(int dim=0 ; dim<3 ; dim++)
        {
            // Order 0 (continuity) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;
            M[line][col  ] = s*s*s;
            M[line][col+1] = s*s;
            M[line][col+2] = s;
            M[line][col+3] = 1;

            col = 12*(n+1)+4*dim;
            M[line][col  ] = 0;
            M[line][col+1] = 0;
            M[line][col+2] = 0;
            M[line][col+3] = -1;

            line++;

            //Order 1 (continuity of first derivative) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;

            M[line][col  ] = 3*s*s;
            M[line][col+1] = 2*s;
            M[line][col+2] = 1;
            M[line][col+3] = 0;

            col = 12*(n+1)+4*dim;
            M[line][col  ] = 0;
            M[line][col+1] = 0;
            M[line][col+2] = -1;
            M[line][col+3] = 0;

            line++;

            //Order 2 (continuity of second derivative) : 3 * (nbSeg-1) constraints
            col = 12*n+4*dim;

            M[line][col  ] = 6*s;
            M[line][col+1] = 2;
            M[line][col+2] = 0;
            M[line][col+3] = 0;

            col = 12*(n+1)+4*dim;
            M[line][col  ] = 0;
            M[line][col+1] = -2;
            M[line][col+2] = 0;
            M[line][col+3] = 0;

            line++;
        }
    }

    vpColVector tipForceVector(3, 0);
    vpColVector tipMomentVector(3, 0);

    if(m_needle.getNbSegments()>1 && m_IsInserting)
    {
        vpColVector xtip(3);
        xtip[0] = m_worldMtip[0][0];
        xtip[1] = m_worldMtip[1][0];
        xtip[2] = m_worldMtip[2][0];

        vpColVector z = m_springs.back().getDirection();
        vpColVector y = vpColVector::crossProd(z,xtip);
        y.normalize();
        tipForceVector = y;
        tipMomentVector = y;

        tipForceVector *= m_tipForce;
        tipMomentVector *= m_tipMoment;
    }

    for(int dim=0 ; dim<3 ; dim++)
    {
        // Base conditions : 6 constraints
        col = 4*dim;

        M[line][col+3] = 1;
        b[line] = m_basePose[dim];
        m_inputBasePositionIndex[dim] = line;
        line ++;

        M[line][col+2] = 1;
        b[line] = m_worldMbase[dim][2];
        m_inputBaseDirectionIndex[dim] = line;
        line++;

        // Tip conditions : 6 constraints
        col = 12*(nbSeg-1)+4*dim;

        M[line][col] = 6*m_EI;
        b[line] = -tipForceVector[dim];

        line ++;

        M[line][col] = 6*m_needle.accessLastSegment().getParametricLength()*m_EI;
        M[line][col+1] = 2*m_EI;
        b[line] = -tipMomentVector[dim];
        line++;
    }

    // Springs : 3 * (nbSeg-1) constraints
    for(int n=0 ; n<nbSeg-1 ; n++)
    {
        // Force calculation
        double px_n = m_springs.at(n).getPosition()[0];
        double py_n = m_springs.at(n).getPosition()[1];
        double pz_n = m_springs.at(n).getPosition()[2];
        double dx_n = m_springs.at(n).getDirection()[0];
        double dy_n = m_springs.at(n).getDirection()[1];
        double dz_n = m_springs.at(n).getDirection()[2];

        // Build an orthonormal base for the plan normal to the spring direction
        vpColVector n1m_n(3);
        if( (fabs(dx_n) > std::numeric_limits<double>::epsilon() ) || (fabs(dy_n)  > std::numeric_limits<double>::epsilon()) )
        {
            vpColVector z(3);
            z[0] = 0;
            z[1] = 0;
            z[2] = 1;
            n1m_n = vpColVector::crossProd(m_springs.at(n).getDirection(), z);
        }
        else
        {
            vpColVector y(3);
            y[0] = 0;
            y[1] = 1;
            y[2] = 0;
            n1m_n = vpColVector::crossProd(m_springs.at(n).getDirection(), y);
        }
        n1m_n.normalize();

        double n1x_n = n1m_n[0];
        double n1y_n = n1m_n[1];
        double n1z_n = n1m_n[2];

        vpColVector n2m_n = vpColVector::cross(m_springs.at(n).getDirection(), n1m_n);
        n2m_n.normalize();

        double n2x_n = n2m_n[0];
        double n2y_n = n2m_n[1];
        double n2z_n = n2m_n[2];

        col = 12*n;

        M[line][col] = 6*m_EI * n1x_n;
        M[line][col+4] = 6*m_EI * n1y_n;
        M[line][col+8] = 6*m_EI * n1z_n;
        M[line+1][col] = 6*m_EI * n2x_n;
        M[line+1][col+4] = 6*m_EI * n2y_n;
        M[line+1][col+8] = 6*m_EI * n2z_n;
        for(int i=0 ; i<nbSeg-1-n ; i++)
        {
            double K = m_springs.at(n+i).getStiffness();
            vpColVector p = m_springs.at(n+i).getPosition();
            double dot1 = vpColVector::dotProd(p,n1m_n);
            M[line][col+12*(i+1)+3] = -K*n1x_n;
            M[line][col+12*(i+1)+3+4] = -K*n1y_n;
            M[line][col+12*(i+1)+3+8] = -K*n1z_n;
            b[line] += -K*dot1;

            double dot2 = vpColVector::dotProd(p,n2m_n);
            M[line+1][col+12*(i+1)+3] = -K*n2x_n;
            M[line+1][col+12*(i+1)+3+4] = -K*n2y_n;
            M[line+1][col+12*(i+1)+3+8] = -K*n2z_n;
            b[line+1] += -K*dot2;
        }
        b[line] += -vpColVector::dotProd(tipForceVector, n1m_n);
        b[line+1] += -vpColVector::dotProd(tipForceVector, n2m_n);

        line+=2;

        // Points stay in the normal plan of the spring
        col = 12*(n+1);
        double dotProd = dx_n*px_n+dy_n*py_n+dz_n*pz_n;

        M[line][col+0+3] = dx_n;
        M[line][col+4+3] = dy_n;
        M[line][col+8+3] = dz_n;
        b[line] = dotProd;
        line++;
    }

    // Solve system

    vpMatrix Minv = M.inverseByLU();
    a = Minv * b;

    // Update needle parameters
    vpMatrix coef(3,4);
    for(int n=0 ; n<nbSeg ; n++)
    {

        for(int dim=0 ; dim<3 ; dim++)
        {
            int seg = 12*n + 4*dim;
            coef[dim][0] = a[seg  ];
            coef[dim][1] = a[seg+1];
            coef[dim][2] = a[seg+2];
            coef[dim][3] = a[seg+3];

            m_needle.accessSegment(n).setPolynomialCoefficients(coef);
            m_needle.accessSegment(n).update();
        }
    }

    // Compute new tip pose
    vpRotationMatrix worldRbase(m_worldMbase);

    vpRotationMatrix worldRtip(worldRbase);

    for(int i=0 ; i<m_needle.getNbSegments() ; i++)
    {
        vpColVector vect =  vpColVector::crossProd(m_needle.accessSegment(i).getStartTangent(), m_needle.accessSegment(i).getEndTangent());
        double dot = vpColVector::dotProd(m_needle.accessSegment(i).getStartTangent(), m_needle.accessSegment(i).getEndTangent());
        double theta = atan2(vect.euclideanNorm(), dot);
        vect.normalize();
        vect = theta * vect;
        vpThetaUVector thetaU(vect[0], vect[1], vect[2]);
        vpRotationMatrix Ri(thetaU);
        worldRtip = Ri * worldRtip;
    }

    vpColVector t = m_needle.accessLastSegment().getEndPoint();
    vpTranslationVector translation(t[0], t[1], t[2]);

    m_worldMtip.buildFrom(translation, worldRtip);
    m_tipPose.buildFrom(m_worldMtip);
}
#endif

void usNeedleInsertionModelVirtualSprings::solveSegmentsParameters()
{
#ifdef VISP_HAVE_EIGEN3
    this->solveSegmentsParametersSparseEigen();
#elif VISP_HAVE_OPENCV
    this->solveSegmentsParametersOpenCV();
#else
    this->solveSegmentsParametersViSP();
#endif
}

void usNeedleInsertionModelVirtualSprings::computeSegmentsLengths()
{
    // Go along each segment and update the value of the segment length between successive springs
    // Except for the last segment (need to use AddRemoveSprings)
    // If no insertion points, the length of the needle is known
    if(m_springs.size() == 0)
    {
        m_needle.accessSegment(0).setParametricLength(m_needle.getFullLength());
        m_LastSegmentLengthComputed = true;
        return;
    }

    for(int i=0 ; i<m_needle.getNbSegments()-1 ; i++)
    {
        double l = m_needle.accessSegment(i).getLength();
        m_needle.accessSegment(i).setParametricLength(l);
    }
}

bool usNeedleInsertionModelVirtualSprings::addRemoveSprings()
{
    // If the tip is reached before the last spring, springs need to be removed
    // If the last segment length is higher than the specified distance between springs, a spring is added

    // If needle is not yet inserted
    if(m_springs.size() == 0)
    {
        // If the tissue surface has been defined
        if(m_tissueSurface.getDirection().euclideanNorm()!=0)
        {
            double s = m_needle.getFullLength();
            if(usGeometryTools::DoesSegmentCrossPlane(m_needle.accessSegment(0), m_tissueSurface)) usGeometryTools::getPlaneCurveCrossingPoint(m_needle.accessSegment(0), m_tissueSurface, -1, &s);

            if( (s>=0) && (s < m_needle.getFullLength()-m_interTipSpringDistance) )
            {
                this->addInsertionPointOnSegment(m_needle.getNbSegments()-1, s);
                m_LastSegmentLengthComputed = true;
                return true;
            }
            else
            {
                m_needle.accessLastSegment().setParametricLength(m_needle.getFullLength());
                m_LastSegmentLengthComputed = true;
                return false;
            }
        }
        else
        {
            m_needle.accessLastSegment().setParametricLength(m_needle.getFullLength());
            m_LastSegmentLengthComputed = true;
            return false;
        }
    }

    double totalLength = m_needle.accessSegment(0).getParametricLength();
    int nbTipSprings = 0;
    int segment = 1;

    while( (totalLength < m_needle.getFullLength()) && (nbTipSprings <= m_nbMaxTipSprings) && (segment < (m_needle.getNbSegments()-1)) )
    {
        totalLength += m_needle.accessSegment(segment).getParametricLength();
        if(segment >= m_tipSpringsIndex) nbTipSprings++;
        segment++;
    }

    // if we stopped because total length has been reached
    // it means that the needle was retracted and we need to remove the last springs
    if(totalLength >= m_needle.getFullLength())
    {
        // if the needle is still inserted
        if(segment-1 > 0)
        {
            double lastSegmentLength = m_needle.getFullLength() - (totalLength - m_needle.accessSegment(segment-1).getParametricLength());
            int lastValidSpringIndex = segment-2;

            if(m_AutomaticSpringAddition)
            {
                // Add spring at needle tip if it is not on the current last segment
                if( (lastValidSpringIndex < m_tipSpringsIndex) && (lastSegmentLength > m_interTipSpringDistance) )
                {
                    this->addInsertionPointOnSegment(lastValidSpringIndex+1, lastSegmentLength-m_interTipSpringDistance);
                    lastValidSpringIndex++;
                    m_tipSpringsIndex++;
                    m_needle.accessSegment(lastValidSpringIndex+1).setParametricLength(m_interTipSpringDistance);
                }

                // Add springs to reach the minimum number of tip springs (to avoid large discontinuity when removing last springs)
                if(m_tipSpringsIndex > lastValidSpringIndex) m_tipSpringsIndex = lastValidSpringIndex;
                while( (lastValidSpringIndex - m_tipSpringsIndex + 1 < m_nbMinTipSprings) && (m_tipSpringsIndex>0) )
                {
                    double l = m_needle.accessSegment(m_tipSpringsIndex).getParametricLength();
                    if(l>1.5*m_interTipSpringDistance)
                    {
                        this->addInsertionPointOnSegment(m_tipSpringsIndex, l - m_interTipSpringDistance);
                        lastValidSpringIndex++;
                    }
                    else m_tipSpringsIndex--;
                }
            }
            else m_tipSpringsIndex = lastValidSpringIndex;

            // Remove all springs where the needle is not anymore
            this->removeInsertionPoints(lastValidSpringIndex+1, m_springs.size()-1);
        }
        // else the needle has been fully removed
        else
        {
            this->removeInsertionPoints(0, m_springs.size()-1);
            m_needle.accessLastSegment().setParametricLength(m_needle.getFullLength());
            m_tipSpringsIndex = 0;
        }

        m_LastSegmentLengthComputed = true;
        return true;
    }
    // if we stopped because tip length is higher than the threshold
    // it means that we can remove a spring that is far from the tip
    else if( nbTipSprings > m_nbMaxTipSprings )
    {
        // Keep spring if it leads to a segment that is too long or if it is a measure spring and increase tip spring index instead
        if( (m_tipSpringsIndex==0) || (m_needle.accessSegment(m_tipSpringsIndex).getParametricLength()+m_needle.accessSegment(m_tipSpringsIndex+1).getParametricLength() >= m_interSpringDistance) || (!m_springs.at(m_tipSpringsIndex).IsPositionUpdateAllowed()) )
        {
            //this->fusionSprings(m_tipSpringsIndex, m_tipSpringsIndex+2);
            m_tipSpringsIndex++;
        }
        //Remove beginning of the tip
        else this->fusionSprings(m_tipSpringsIndex-1, m_tipSpringsIndex+1);
        return true;
    }
    // if we reached the last segment we can deduce its length
    else
    {
        double lastSegmentLength = m_needle.getFullLength() - totalLength;

        // If the last segment is too long
        if( m_AutomaticSpringAddition && (lastSegmentLength > m_interTipSpringDistance) )
        {
            // If its length corresponds to the curvilinear parameter we can add a spring (last segment has been measured and the system recomputed)
            if(m_LastSegmentLengthComputed)
            {
                m_needle.accessLastSegment().setParametricLength(lastSegmentLength);
                this->addInsertionPointOnSegment(m_needle.getNbSegments()-1, 0.9*m_interTipSpringDistance);
                return true;
            }
            // Else we update its length and the system must be recomputed
            else
            {
                m_needle.accessLastSegment().setParametricLength(lastSegmentLength);
                m_LastSegmentLengthComputed = true;
                return true;
            }
        }
        // Else we can update its length
        else
        {
            m_needle.accessLastSegment().setParametricLength(lastSegmentLength);
            m_LastSegmentLengthComputed = true;
            return false;
        }
    }
}

bool usNeedleInsertionModelVirtualSprings::checkInactiveMeasureSprings()
{
    std::vector<int> activated;

    for(unsigned int spg=0 ; spg<m_inactiveMeasureSprings.size() ; spg++)
    {
        for(int seg=0 ; seg<m_needle.getNbSegments() ; seg++)
        {
            if(usGeometryTools::DoesSegmentCrossPlaneDirect(m_needle.accessSegment(seg), m_inactiveMeasureSprings.at(spg)))
            {
                activated.push_back(spg);
                break;
            }
        }
    }
    for(int i=activated.size()-1 ; i>=0 ; i--)
    {
        int j = this->addInsertionPoint(m_inactiveMeasureSprings.at(activated.at(i)));
        if(j<=m_tipSpringsIndex) m_tipSpringsIndex++;
        m_inactiveMeasureSprings.erase(m_inactiveMeasureSprings.begin()+activated.at(i), m_inactiveMeasureSprings.begin()+activated.at(i)+1);
    }
    return (activated.size()!=0);
}

void usNeedleInsertionModelVirtualSprings::addMeasureSpring(const vpColVector &p, const vpColVector &d)
{
    if(p.size() != 3 || d.size() != 3) throw vpException(vpException::dimensionError, "usNeedleInsertionModelVirtualSprings::addMeasureSpring: invalid vector dimension");

    if(m_tissueSurface.getDirection().euclideanNorm()!=0 && !usGeometryTools::IsPointInFrontOfPlane(p,m_tissueSurface))
    {
        std::cerr << "Warning in usNeedleInsertionModelVirtualSprings::addMeasureSpring: cannot add spring outside of tissue" << std::endl;
        return;
    }

    usVirtualSpring spg(p, d, 0);
    spg.AllowPositionUpdate(false);
    spg.AllowDirectionUpdate(true);
    spg.AllowStiffnessUpdate(true);

    m_inactiveMeasureSprings.push_back(spg);

}

bool usNeedleInsertionModelVirtualSprings::updateState()
{
    // Get needle length before motion
    double l = m_needle.accessSegment(0).getParametricLength();

    // Solve system assuming same insertion direction as previous one
    bool needRecomputation = false;

    this->updateCutAngle();
    this->updateInsertionDirections();
    this->updateTipForce();
    this->updateSpringsStiffness();
    this->solveSegmentsParameters(); // solve the system
    m_LastSegmentLengthComputed = false; // set flag to specify that the curvilinear parameter of the last segment does certainly not correspond to the length
    this->computeSegmentsLengths(); // measure the length of each segment except the last one
    needRecomputation = this->addRemoveSprings();

    // Check insertion direction (insertion and removal)
    if(m_insertionBehavior == InsertionType::ForceInsert) m_IsInserting = true;
    else if(m_insertionBehavior == InsertionType::ForceRemove) m_IsInserting = false;
    else if(m_springs.size()>0)
    {
        double insertionStep = l - m_needle.accessSegment(0).getParametricLength();

        // Insertion
        if(insertionStep>1e-5 && (!m_IsInserting)) m_IsInserting = true;
        // Removal
        else if(insertionStep<-1e-5 && m_IsInserting) m_IsInserting = false;
    }
    else m_IsInserting = false;

    int i = 0;
    while( i<10 && needRecomputation )
    {
        this->updateCutAngle();
        this->updateInsertionDirections();
        this->updateTipForce();
        this->updateSpringsStiffness();
        this->solveSegmentsParameters();//this->displayDebug();
        this->computeSegmentsLengths();
        needRecomputation = this->addRemoveSprings();

        i++;
    }

    this->updateCutAngle();
    this->updateInsertionDirections();
    this->updateTipForce();
    this->updateSpringsStiffness();
    this->solveSegmentsParameters();

    needRecomputation = this->checkInactiveMeasureSprings();
    if(needRecomputation)
    {
        if(m_AutomaticSpringAddition) this->removeAutoAddedSprings();

        while( i<300 && needRecomputation )
        {
            this->updateCutAngle();
            this->updateInsertionDirections();
            this->updateTipForce();
            this->updateSpringsStiffness();
            this->solveSegmentsParameters();//this->displayDebug();
            this->computeSegmentsLengths();
            needRecomputation = this->addRemoveSprings();

            i++;
        }

        this->updateCutAngle();
        this->updateInsertionDirections();
        this->updateTipForce();
        this->updateSpringsStiffness();
        this->solveSegmentsParameters();
    }

    return true;
}

void usNeedleInsertionModelVirtualSprings::showInsertionPoints() const
{
    std::cout << "Needle " << this << ": " << m_springs.size() <<" InsertionPoints: \n";
    for(unsigned int i=0 ; i<m_springs.size() ; i++)
    {
        std::cout << "\t Direction " << i << ":\n";
        vpColVector d = m_springs.at(i).getPosition();
        for(int j=0 ; j<3 ; j++)
        {
            std::cout << "\t\t " << d[j] << "\n";
        }
    }
    std::cout << std::endl;
}

void usNeedleInsertionModelVirtualSprings::showInsertionDirections() const
{
    std::cout << "Needle " << this << ": " << m_springs.size() <<" InsertionDirections: \n";
    for(unsigned int i=0 ; i<m_springs.size() ; i++)
    {
        std::cout << "\t Direction " << i << ":\n";
        vpColVector d = m_springs.at(i).getDirection();
        for(int j=0 ; j<3 ; j++)
        {
            std::cout << "\t\t " << d[j] << "\n";
        }
    }
    std::cout << std::endl;
}

void usNeedleInsertionModelVirtualSprings::showStiffnesses() const
{
    std::cout << "Needle " << this << ": " << m_springs.size() <<" Springs: \n";
    for(unsigned int i=0 ; i<m_springs.size() ; i++)
    {
        std::cout << "\t Stiffness " << i << ": " << m_springs.at(i).getStiffness() << "\n";
    }
    std::cout << std::endl;
}

std::ostream &operator<<(std::ostream &s, const usNeedleInsertionModelVirtualSprings &needle)
{
    s << "usNeedleInsertionModelVirtualSprings\n";

    s << needle.m_needle;

    s << needle.m_tipForce << '\n';
    s << needle.m_tipMoment << '\n';
    s << needle.m_cutAngle << '\n';

    s << needle.m_defaultSpringStiffness << '\n';
    s << needle.m_stiffnessPerUnitLength << '\n';

    int n = needle.m_springs.size();
    s << n << '\n';
    for(int i=0 ; i<n ; i++) s << needle.m_springs.at(i);
    n = needle.m_inactiveAutoAddedSprings.size();
    s << n << '\n';
    for(int i=0 ; i<n ; i++) s << needle.m_inactiveAutoAddedSprings.at(i);
    n = needle.m_inactiveMeasureSprings.size();
    s << n << '\n';
    for(int i=0 ; i<n ; i++) s << needle.m_inactiveMeasureSprings.at(i);
    s << needle.m_tissueSurface;

    s << needle.m_interSpringDistance << '\n';
    s << needle.m_interTipSpringDistance << '\n';

    s << needle.m_IsStateConsistent << '\n';

    s << needle.m_LastSegmentLengthComputed << '\n';

    s << (int)needle.m_insertionBehavior << '\n';
    s << needle.m_IsInserting << '\n';
    s << needle.m_AllowSpringAddition << '\n';
    s << needle.m_AllowSpringRemoval << '\n';
    s << needle.m_AutomaticSpringAddition << '\n';

    s << needle.m_tipSpringsIndex << '\n';
    s << needle.m_nbMinTipSprings << '\n';
    s << needle.m_nbMaxTipSprings << '\n';

    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usNeedleInsertionModelVirtualSprings &needle)
{
    char c[26];
    s >> c;
    if(strcmp(c,"usNeedleInsertionModelVirtualSprings"))
    {
        vpException e(vpException::ioError, "operator>>=(std::istream&, usNeedleInsertionModelVirtualSprings&): Stream does not contain usNeedleInsertionModelVirtualSprings data");
        throw e;
    }

    s >> needle.m_needle;
    s >> needle.m_tipForce;
    s >> needle.m_tipMoment;
    s >> needle.m_cutAngle;

    s >> needle.m_defaultSpringStiffness;
    s >> needle.m_stiffnessPerUnitLength;

    int n = 0;
    s >> n;
    needle.m_springs.clear();
    needle.m_springs.resize(n);
    for(int i=0 ; i<n ; i++) s >> needle.m_springs.at(i);
    s >> n;
    needle.m_inactiveAutoAddedSprings.clear();
    needle.m_inactiveAutoAddedSprings.resize(n);
    for(int i=0 ; i<n ; i++) s >> needle.m_inactiveAutoAddedSprings.at(i);
    s >> n;
    needle.m_inactiveMeasureSprings.clear();
    needle.m_inactiveMeasureSprings.resize(n);
    for(int i=0 ; i<n ; i++) s >> needle.m_inactiveMeasureSprings.at(i);
    s >> needle.m_tissueSurface;

    s >> needle.m_interSpringDistance;
    s >> needle.m_interTipSpringDistance;

    s >> needle.m_IsStateConsistent;

    s >> needle.m_LastSegmentLengthComputed;

    int b = 0;
    s >> b;
    needle.m_insertionBehavior = (usNeedleInsertionModelVirtualSprings::InsertionType)b;
    s >> needle.m_IsInserting;
    s >> needle.m_AllowSpringAddition;
    s >> needle.m_AllowSpringRemoval;
    s >> needle.m_AutomaticSpringAddition;

    s >> needle.m_tipSpringsIndex;
    s >> needle.m_nbMinTipSprings;
    s >> needle.m_nbMaxTipSprings;
    return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleInsertionModelVirtualSprings &needle)
{
    s.write("usNeedleInsertionModelVirtualSprings",26);
    s <<= needle.m_needle;
    s.write((char*)&(needle.m_tipForce), sizeof(double));
    s.write((char*)&(needle.m_tipMoment), sizeof(double));
    s.write((char*)&(needle.m_cutAngle), sizeof(double));

    s.write((char*)&(needle.m_defaultSpringStiffness), sizeof(double));
    s.write((char*)&(needle.m_stiffnessPerUnitLength), sizeof(double));

    int n = needle.m_springs.size();
    s.write((char*)&n, sizeof(int));
    for(int i=0 ; i<n ; i++) s <<= needle.m_springs.at(i);
    n = needle.m_inactiveAutoAddedSprings.size();
    s.write((char*)&n, sizeof(int));
    for(int i=0 ; i<n ; i++) s <<= needle.m_inactiveAutoAddedSprings.at(i);
    n = needle.m_inactiveMeasureSprings.size();
    s.write((char*)&n, sizeof(int));
    for(int i=0 ; i<n ; i++) s <<= needle.m_inactiveMeasureSprings.at(i);
    s <<= needle.m_tissueSurface;

    s.write((char*)&(needle.m_interSpringDistance), sizeof(double));
    s.write((char*)&(needle.m_interTipSpringDistance), sizeof(double));

    s.write((char*)&(needle.m_IsStateConsistent), sizeof(bool));

    s.write((char*)&(needle.m_LastSegmentLengthComputed), sizeof(double));

    s.write((char*)&(needle.m_insertionBehavior), sizeof(int));
    s.write((char*)&(needle.m_IsInserting), sizeof(bool));
    s.write((char*)&(needle.m_AllowSpringAddition), sizeof(bool));
    s.write((char*)&(needle.m_AllowSpringRemoval), sizeof(bool));
    s.write((char*)&(needle.m_AutomaticSpringAddition), sizeof(bool));

    s.write((char*)&(needle.m_tipSpringsIndex), sizeof(int));
    s.write((char*)&(needle.m_nbMinTipSprings), sizeof(int));
    s.write((char*)&(needle.m_nbMaxTipSprings), sizeof(int));

    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usNeedleInsertionModelVirtualSprings &needle)
{
    char c[26];
    s.read(c,26);
    if(strcmp(c,"usNeedleInsertionModelVirtualSprings"))
    {
        vpException e(vpException::ioError, "operator>>=(std::istream&, usNeedleInsertionModelVirtualSprings&): Stream does not contain usNeedleInsertionModelVirtualSprings data");
        throw e;
    }
    s >>= needle.m_needle;
    s.read((char*)&(needle.m_tipForce), sizeof(double));
    s.read((char*)&(needle.m_tipMoment), sizeof(double));
    s.read((char*)&(needle.m_cutAngle), sizeof(double));

    s.read((char*)&(needle.m_defaultSpringStiffness), sizeof(double));
    s.read((char*)&(needle.m_stiffnessPerUnitLength), sizeof(double));

    int n =0;
    s.read((char*)&n, sizeof(int));
    needle.m_springs.clear();
    needle.m_springs.resize(n);
    for(int i=0 ; i<n ; i++) s >>= needle.m_springs.at(i);
    s.read((char*)&n, sizeof(int));
    needle.m_inactiveAutoAddedSprings.clear();
    needle.m_inactiveAutoAddedSprings.resize(n);
    for(int i=0 ; i<n ; i++) s >>= needle.m_inactiveAutoAddedSprings.at(i);
    s.read((char*)&n, sizeof(int));
    needle.m_inactiveMeasureSprings.clear();
    needle.m_inactiveMeasureSprings.resize(n);
    for(int i=0 ; i<n ; i++) s >>= needle.m_inactiveMeasureSprings.at(i);
    s >>= needle.m_tissueSurface;

    s.read((char*)&(needle.m_interSpringDistance), sizeof(double));
    s.read((char*)&(needle.m_interTipSpringDistance), sizeof(double));

    s.read((char*)&(needle.m_IsStateConsistent), sizeof(bool));

    s.read((char*)&(needle.m_LastSegmentLengthComputed), sizeof(double));

    int b = 0;
    s.read((char*)&b, sizeof(int));
    needle.m_insertionBehavior = (usNeedleInsertionModelVirtualSprings::InsertionType)b;

    s.read((char*)&(needle.m_IsInserting), sizeof(bool));
    s.read((char*)&(needle.m_AllowSpringAddition), sizeof(bool));
    s.read((char*)&(needle.m_AllowSpringRemoval), sizeof(bool));
    s.read((char*)&(needle.m_AutomaticSpringAddition), sizeof(bool));

    s.read((char*)&(needle.m_tipSpringsIndex), sizeof(int));
    s.read((char*)&(needle.m_nbMinTipSprings), sizeof(int));
    s.read((char*)&(needle.m_nbMaxTipSprings), sizeof(int));
    return s;
}
