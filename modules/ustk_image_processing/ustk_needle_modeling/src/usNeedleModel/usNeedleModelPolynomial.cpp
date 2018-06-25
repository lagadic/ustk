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

#include <visp3/ustk_needle_modeling/usNeedleModelPolynomial.h>

#include<iomanip>

#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRowVector.h>
#include <visp3/core/vpTime.h>


usNeedleModelPolynomial::usNeedleModelPolynomial():
    usNeedleModelBaseTip(),
    usPolynomialCurve3D(),

    m_outerDiameter(0.001),
    m_insideDiameter(0),
    m_needleYoungModulus(200000000000)
{
    this->init();
}

usNeedleModelPolynomial::usNeedleModelPolynomial(const usNeedleModelPolynomial &needle):
    usNeedleModelBaseTip(needle),
    usPolynomialCurve3D(needle),

    m_outerDiameter(needle.m_outerDiameter),
    m_insideDiameter(needle.m_insideDiameter),
    m_needleYoungModulus(needle.m_needleYoungModulus)
{

}

usNeedleModelPolynomial::~usNeedleModelPolynomial()
{

}

const usNeedleModelPolynomial& usNeedleModelPolynomial::operator=(const usNeedleModelPolynomial &needle)
{
    this->usNeedleModelBaseTip::operator=(needle);

    m_outerDiameter = needle.m_outerDiameter;
    m_insideDiameter = needle.m_insideDiameter;
    m_needleYoungModulus = needle.m_needleYoungModulus;

    return *this;
}


void usNeedleModelPolynomial::loadPreset(const NeedlePreset preset)
{
    switch(preset)
    {
        case NeedlePreset::BiopsyNeedle :
        {
            this->setParametricLength(0.126);
            this->setOuterDiameter(0.0007);
            this->setInsideDiameter(0.00048);
            this->setNeedleYoungModulus(200*pow(10,9));
            break;
        }
        case NeedlePreset::BiopsyCannula :
        {
            this->setParametricLength(0.146);
            this->setOuterDiameter(0.00048);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(200*pow(10,9));
            break;
        }
        case NeedlePreset::AbayazidRRM13 :
        {
            this->setParametricLength(0.1);
            this->setOuterDiameter(0.0005);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(75*pow(10,9));
            break;
        }
        case NeedlePreset::MisraRSRO10_PlastisolA :
        {
            this->setParametricLength(0.1);
            this->setOuterDiameter(0.00046);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(0.5*pow(10,9));
            break;
        }
        case NeedlePreset::RoesthuisAM12 :
        {
            this->setParametricLength(0.1);
            this->setOuterDiameter(0.0008);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(75*pow(10,9));
            break;
        }
        case NeedlePreset::SteelSoftTissue :
        {
            this->setParametricLength(0.1);
            this->setOuterDiameter(0.001);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(200*pow(10,9));
            break;
        }
    }
}

void usNeedleModelPolynomial::setOuterDiameter(double diameter)
{
    if(diameter > 0) m_outerDiameter = diameter;
}

double usNeedleModelPolynomial::getOuterDiameter() const
{
    return m_outerDiameter;
}

void usNeedleModelPolynomial::setInsideDiameter(double diameter)
{
    if(diameter >= 0) m_insideDiameter = diameter;
}

double usNeedleModelPolynomial::getInsideDiameter() const
{
    return m_insideDiameter;
}

void usNeedleModelPolynomial::setNeedleYoungModulus(double E)
{
    if(E > 0) m_needleYoungModulus = E;
}

double usNeedleModelPolynomial::getNeedleYoungModulus() const
{
    return m_needleYoungModulus;
}

double usNeedleModelPolynomial::getEI() const
{
    return m_needleYoungModulus * M_PI/64 * (pow(m_outerDiameter, 4) - pow(m_insideDiameter, 4));
}

void usNeedleModelPolynomial::init()
{
    this->setBoundaries(0,0.1);
    vpMatrix M(3,4,0);
    M[2][1] = 1;
    this->setPolynomialCoefficients(M);
}

vpColVector usNeedleModelPolynomial::getNeedlePoint(double l) const
{
    if(l<0 || l>m_endParameter) throw vpException(vpException::badValue, "usNeedleModelPolynomial::getNeedlePoint: length out of needle");

    return this->getPoint(l);
}

vpColVector usNeedleModelPolynomial::getNeedleDirection(double l) const
{
    if(l<0 || l>m_endParameter) throw vpException(vpException::badValue, "usNeedleModelPolynomial::getNeedleDirection: length out of needle");

    return this->getTangent(l);
}

double usNeedleModelPolynomial::getDistanceFromPoint(const vpColVector &P, double start, double stop, double threshold) const
{
    if(P.size() != 3) throw vpException(vpException::dimensionError, "usNeedleModelPolynomial::getDistanceFromPoint: invalid point dimension");

    if(start<0) start = 0;
    if(stop == -1 || stop>m_endParameter) stop = m_endParameter;
    if(stop<0) stop = 0;
    if(start>stop) throw vpException(vpException::badValue, "usNeedleModelPolynomial::getDistanceFromPoint: invalid start-stop parameters");

    double middle = (start+stop)/2;
    while( (stop-start) > threshold)
    {
        double d0 = (this->getNeedlePoint(start)-P).euclideanNorm();
        double d1 = (this->getNeedlePoint(middle)-P).euclideanNorm();
        double d2 = (this->getNeedlePoint(stop)-P).euclideanNorm();

        if(d0<=d1 && d0<d2) stop = middle;
        else if(d2<d0 && d2<=d1) start = middle;
        else
        {
            start = (start+middle)/2;
            stop = (middle+stop)/2;
        }
        middle = (start+stop)/2;
    }

    double l = (this->getNeedlePoint(middle)-P).euclideanNorm();

    return l;
}

double usNeedleModelPolynomial::getBendingEnergy() const
{
    vpColVector Ls_pow(2*m_order-2);
    Ls_pow[0] = 1;
    vpColVector Le_pow(2*m_order-2);
    Le_pow[0] = 1;

    for(unsigned int i=1 ; i<2*m_order-2 ; i++)
    {
        Ls_pow[i] = m_startParameter * Ls_pow[i-1];
        Le_pow[i] = m_endParameter * Le_pow[i-1];
    }

    vpMatrix dotProd = m_polynomialCoefficients.t()*m_polynomialCoefficients;

    double E = 0;

    for(unsigned int i=2 ; i<=m_order ; i++)
    {
        for(unsigned int j=2 ; j<=m_order ; j++)
        {
            E += i*(i-1)*j*(j-1) / (i+j-3) * dotProd[i][j] *(Le_pow[i+j-3] - Ls_pow[i+j-3]);
        }
    }

    E *= 0.5*this->getEI();

    return E;
}

vpColVector usNeedleModelPolynomial::getBaseStaticTorsor() const
{
    vpColVector d2X_dl2 = this->getDerivative(0,2);

    double EI = this->getEI();

    vpColVector Moment = EI * vpColVector::crossProd(this->getDerivative(0,1), d2X_dl2);

    vpColVector d3X_dl3 =  this->getDerivative(0,3);

    vpColVector Force = EI * ( d3X_dl3 - vpColVector::dotProd(d3X_dl3, this->getStartTangent()) * this->getStartTangent());

    vpColVector Torsor(6);
    Torsor[0] = Force[0];
    Torsor[1] = Force[1];
    Torsor[2] = Force[2];
    Torsor[3] = Moment[0];
    Torsor[4] = Moment[1];
    Torsor[5] = Moment[2];

    return Torsor;
}

double usNeedleModelPolynomial::getCurvatureFromNeedleShape(double start, double end, vpColVector &center3D, vpColVector &direction3D) const
{
    if(start<0) start = 0;
    if(start>m_endParameter) start = m_endParameter;
    if(end<0) end = 0;
    if(end>m_endParameter) end = m_endParameter;
    if(start>end)
    {
        double tmp = start;
        start = end;
        end = tmp;
    }

    int subSegmentsNumber = 100;
    int nbPoints =  subSegmentsNumber+1;

    if(nbPoints < 3)
    {
        return 0;
    }

    // Create data matrix with centered vectors
    vpMatrix M(nbPoints,3);
    vpRowVector mean(3,0);

    double step = this->getParametricLength()/subSegmentsNumber;
    for(int i=0 ; i<nbPoints; i++) M.insert(this->getPoint(m_startParameter+i*step).t(),i,0);

    for(int i=0 ; i<nbPoints; i++) mean += M.getRow(i);
    mean /= nbPoints;

    for(int i=0 ; i<nbPoints; i++)
    {
        M[i][0] -= mean[0];
        M[i][1] -= mean[1];
        M[i][2] -= mean[2];
    }

    // Reduction to two principal components using singular value decomposition
    vpMatrix U(M);
    vpColVector w;
    vpMatrix V;
    U.svd(w, V);

    vpMatrix S;
    S.diag(w);

    U.resize(nbPoints, 2, false);

    S.resize(2, 2, false);

    vpMatrix P = U*S;

    // 2D nonlinear least square fitting (Coope93)
    vpColVector d(nbPoints);
    for(int i=0 ; i<nbPoints; i++)
    {
        d[i] = pow( P.t().getCol(i).euclideanNorm(), 2);
    }

    vpColVector x(nbPoints, 1);
    vpMatrix B(nbPoints, 3);
    B.insert(P,0,0);
    B.insert(x,0,2);

    vpColVector y = B.pseudoInverse(0) * d;

    vpColVector center(2);
    center[0] = y[0]/2;
    center[1] = y[1]/2;

    double r = sqrt(y[2] + pow(center.euclideanNorm(), 2));

    // Check validity

    vpColVector cp = P.getRow(0).t() - center;
    for(int i=1 ; i<nbPoints ; i++)
    {
        double dot = vpColVector::dotProd(cp, P.getRow(i).t() - center);
        if(dot<0) return 0;
    }

    if(direction3D.size() == 3)
    {
        direction3D = V.getCol(2);
    }
    else if(direction3D.size() == 4)
    {
        direction3D.insert(0, V.getCol(2));
        direction3D[3] = 0;
    }

    if(center3D.size() == 3)
    {
        V.resize(3, 2, false);
        center3D = V*center;
    }
    else if(center3D.size() == 4)
    {
        V.resize(3, 2, false);
        center3D.insert(0, V*center+mean.t());
        center3D[3] = 1;
    }

    return 1.0/r;
}

std::ostream &operator<<(std::ostream &s, const usNeedleModelPolynomial &needle)
{
    s << "usNeedleModelPolynomial\n";
    s << *((usNeedleModelBaseTip*)(&needle));
    // TODO DATA SAVE s << *((usPolynomialCurve3D*)(&needle));
    s << needle.m_outerDiameter << '\n';
    s << needle.m_insideDiameter << '\n';
    s << needle.m_needleYoungModulus << '\n';

    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usNeedleModelPolynomial &needle)
{
    char c[26];
    s >> c;
    if(strcmp(c,"usNeedleModelPolynomial"))
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleModelPolynomial data");
        throw e;
    }
    s >> (*(usNeedleModelBaseTip*)(&needle));
    // TODO DATAT SAVE s >> (*(usPolynomialCurve3D*)(&needle));
    s >> needle.m_outerDiameter;
    s >> needle.m_insideDiameter;
    s >> needle.m_needleYoungModulus;

    return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleModelPolynomial &needle)
{
    s.write("usNeedleModelPolynomial",22);
    s <<= *((usNeedleModelBaseTip*)(&needle));
    // TODO DATAT SAVE s <<= *((usPolynomialCurve3D*)(&needle));
    s.write((char*)&(needle.m_outerDiameter), sizeof(double));
    s.write((char*)&(needle.m_insideDiameter), sizeof(double));
    s.write((char*)&(needle.m_needleYoungModulus), sizeof(double));

    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usNeedleModelPolynomial &needle)
{
    char c[22];
    s.read(c,22);
    if(strcmp(c,"usNeedleModelPolynomial"))
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleModelPolynomial data");
        throw e;
    }
    s >>= *((usNeedleModelBaseTip*)(&needle));
    // TODO DATAT SAVE s >>= *((usPolynomialCurve3D*)(&needle));
    s.read((char*)&(needle.m_outerDiameter), sizeof(double));
    s.read((char*)&(needle.m_insideDiameter), sizeof(double));
    s.read((char*)&(needle.m_needleYoungModulus), sizeof(double));

    return s;
}
