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

#include <visp3/ustk_needle_modeling/usNeedleModelSpline.h>

#include<iomanip>

#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRowVector.h>
#include <visp3/core/vpTime.h>

//#define DISPLAY_TIMING
//#define DISPLAY_STIFFNESS
//#define DISPLAYm_length


usNeedleModelSpline::usNeedleModelSpline():
    usNeedleModelBaseTip(),
    usBSpline3D(),

    m_length(0.1),
    m_outerDiameter(0.001),
    m_insideDiameter(0),
    m_needleYoungModulus(200000000000)
{
    this->init();
}

usNeedleModelSpline::usNeedleModelSpline(const usNeedleModelSpline &needle):
    usNeedleModelBaseTip(needle),
    usBSpline3D(needle),

    m_length(needle.m_length),
    m_outerDiameter(needle.m_outerDiameter),
    m_insideDiameter(needle.m_insideDiameter),
    m_needleYoungModulus(needle.m_needleYoungModulus)
{

}

usNeedleModelSpline::~usNeedleModelSpline()
{

}

const usNeedleModelSpline& usNeedleModelSpline::operator=(const usNeedleModelSpline &needle)
{
    this->usNeedleModelBaseTip::operator=(needle);
    this->usBSpline3D::operator=(needle);

    m_length = needle.m_length;
    m_outerDiameter = needle.m_outerDiameter;
    m_insideDiameter = needle.m_insideDiameter;
    m_needleYoungModulus = needle.m_needleYoungModulus;

    return *this;
}


void usNeedleModelSpline::loadPreset(const NeedlePreset preset)
{
    switch(preset)
    {
        case NeedlePreset::BiopsyNeedle :
        {
            this->setLength(0.126);
            this->setOuterDiameter(0.0007);
            this->setInsideDiameter(0.00048);
            this->setNeedleYoungModulus(200e9);
            break;
        }
        case NeedlePreset::BiopsyCannula :
        {
            this->setLength(0.146);
            this->setOuterDiameter(0.00048);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(200e9);
            break;
        }
        case NeedlePreset::Symmetric :
        {
            this->setLength(0.109);
            this->setOuterDiameter(0.00097);
            this->setInsideDiameter(0);
            this->setNeedleYoungModulus(200e9);
            break;
        }
        case NeedlePreset::AbayazidRRM13 :
        {
            this->setLength(0.1);
            this->setOuterDiameter(0.0005);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(75e9);
            break;
        }
        case NeedlePreset::MisraRSRO10_PlastisolA :
        {
            this->setLength(0.15);
            this->setOuterDiameter(0.00046);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(50e9);
            break;
        }
        case NeedlePreset::RoesthuisAM12 :
        {
            this->setLength(0.1);
            this->setOuterDiameter(0.0008);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(75e9);
            break;
        }
        case NeedlePreset::SteelSoftTissue :
        {
            this->setLength(0.1);
            this->setOuterDiameter(0.001);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(200e9);
            break;
        }
        case NeedlePreset::SRL_ActuatedFBG :
        {
            this->setLength(0.1);
            this->setOuterDiameter(0.002);
            this->setInsideDiameter(0.0);
            this->setNeedleYoungModulus(8062500000); // 0 -> 0.5mm = Nitinol (75GPa) + 0.5mm -> 1mm = PEEK (3.6GPa)
            break;
        }
        case NeedlePreset::SRL_BiopsySimple :
        {
            this->setLength(0.127);
            this->setOuterDiameter(0.00075);
            this->setInsideDiameter(0.);
            this->setNeedleYoungModulus(200e9);
            break;
        }
        case NeedlePreset::SRL_BiopsyNID :
        {
            this->setLength(0.164);
            this->setOuterDiameter(0.00055);
            this->setInsideDiameter(0.0005);
            this->setNeedleYoungModulus(200e9);
            break;
        }
    }
}

void usNeedleModelSpline::setLength(double length)
{
    if(length>=0) m_length = length;
}

double usNeedleModelSpline::getLength() const
{
    return m_length;
}

void usNeedleModelSpline::setOuterDiameter(double diameter)
{
    if(diameter > 0) m_outerDiameter = diameter;
}

double usNeedleModelSpline::getOuterDiameter() const
{
    return m_outerDiameter;
}

void usNeedleModelSpline::setInsideDiameter(double diameter)
{
    if(diameter >= 0) m_insideDiameter = diameter;
}

double usNeedleModelSpline::getInsideDiameter() const
{
    return m_insideDiameter;
}

void usNeedleModelSpline::setNeedleYoungModulus(double E)
{
    if(E > 0) m_needleYoungModulus = E;
}

double usNeedleModelSpline::getNeedleYoungModulus() const
{
    return m_needleYoungModulus;
}

double usNeedleModelSpline::getEI() const
{
    return m_needleYoungModulus * M_PI/64 * (pow(m_outerDiameter, 4) - pow(m_insideDiameter, 4));
}

void usNeedleModelSpline::init()
{
    m_spline.clear();
    usPolynomialCurve3D poly;
    poly.setBoundaries(0,m_length);
    vpMatrix M(3,4,0);
    M[2][1] = 1;
    poly.setPolynomialCoefficients(M);
    m_spline.push_back(poly);
}

vpColVector usNeedleModelSpline::getNeedlePoint(double l) const
{
    if(l<0 || l>m_length) throw vpException(vpException::badValue, "usNeedleModelSpline::getNeedlePoint: length out of needle");

    return this->getPoint(l);
}

vpColVector usNeedleModelSpline::getNeedleDirection(double l) const
{
    if(l<0 || l>m_length) throw vpException(vpException::badValue, "usNeedleModelSpline::getNeedleDirection: length out of needle");

    return this->getTangent(l);
}

double usNeedleModelSpline::getDistanceFromPoint(const vpColVector &P, double start, double stop, double threshold) const
{
#ifdef DISPLAY_TIMING
double t0 = vpTime::measureTimeMs();
#endif

    if(P.size() != 3) throw vpException(vpException::dimensionError, "usNeedleModelSpline::getDistanceFromPoint: invalid point dimension");

    if(start<0) start = 0;
    if(stop == -1 || stop>m_length) stop = m_length;
    if(stop<0.) stop = 0.;
    if(start>stop) throw vpException(vpException::badValue, "usNeedleModelSpline::getDistanceFromPoint: invalid start-stop parameters");

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

#ifdef DISPLAY_TIMING
std::cout << "usNeedleModelSpline::getDistanceFromPoint: timing: " << vpTime::measureTimeMs() - t0 << " ms" << std::endl;
#endif

    return l;
}

double usNeedleModelSpline::getBendingEnergy() const
{
    double E = 0;

    if(m_spline.front().getOrder()<2)
    {
        for(unsigned int n=0 ; n<m_spline.size()-1 ; n++)
        {
            const usPolynomialCurve3D &poly = m_spline.at(n);
            const usPolynomialCurve3D &poly2 = m_spline.at(n+1);

            vpColVector dX_dl = poly.getDerivative(poly.getStartParameter(),1);
            vpColVector dX_dl_2 = poly2.getDerivative(poly2.getStartParameter(),1);

            double norm = dX_dl.euclideanNorm();
            double curvature = vpColVector::crossProd(dX_dl,dX_dl_2).euclideanNorm()/pow(norm,3);

            E += pow(curvature, 2);
        }
    }
    else
    {
        for(unsigned int n=0 ; n<m_spline.size() ; n++)
        {
            const usPolynomialCurve3D &poly = m_spline.at(n);

            int m = poly.getOrder();

            double Ls = poly.getStartParameter();
            double Le = poly.getEndParameter();

            vpColVector Ls_pow(2*m-2);
            Ls_pow[0] = 1;
            vpColVector Le_pow(2*m-2);
            Le_pow[0] = 1;

            for(int i=1 ; i<2*m-2 ; i++)
            {
                Ls_pow[i] = Ls * Ls_pow[i-1];
                Le_pow[i] = Le * Le_pow[i-1];
            }

            vpMatrix dotProd = poly.getPolynomialCoefficients().AtA();

            for(int i=2 ; i<=m ; i++)
            {
                for(int j=2 ; j<=m ; j++)
                {
                    E += i*(i-1)*j*(j-1) / (i+j-3) * dotProd[i][j] *(Le_pow[i+j-3] - Ls_pow[i+j-3]);
                }
            }/**/

            /* Trapeze integration method
            int subSegmentsNumber = 100;
            double l = poly.getStartParameter();
            double dl = poly.getParametricLength()/subSegmentsNumber;
            double dE = pow(poly.getCurvature(l),2) / 2;
            l += dl;
            for(int k=1 ; k<subSegmentsNumber ; k++)
            {
                dE += pow(poly.getCurvature(l),2);
                l += dl;
            }
            dE += pow(poly.getCurvature(l),2) / 2;
            E += dl*dE;*/
        }
    }

    E *= 0.5*this->getEI();

    return E;
}

vpColVector usNeedleModelSpline::getBaseStaticTorsor() const
{
    const usPolynomialCurve3D &poly = m_spline.front();
    vpColVector d2X_dl2 = poly.getDerivative(0,2);

    double EI = this->getEI();

    vpColVector Moment = -EI * vpColVector::crossProd(poly.getStartTangent(), d2X_dl2);

    vpColVector d3X_dl3 =  poly.getDerivative(0,3);

    vpColVector Force = EI * ( d3X_dl3 - vpColVector::dotProd(d3X_dl3, poly.getStartTangent()) * poly.getStartTangent());

    vpColVector Torsor(6);
    Torsor[0] = Force[0];
    Torsor[1] = Force[1];
    Torsor[2] = Force[2];
    Torsor[3] = Moment[0];
    Torsor[4] = Moment[1];
    Torsor[5] = Moment[2];

    return Torsor;
}

double usNeedleModelSpline::getCurvatureFromNeedleShape(double start, double end, vpColVector &center3D, vpColVector &direction3D) const
{
    if(start<0) start = 0;
    if(start>m_length) start = m_length;
    if(end<0) end = 0;
    if(end>m_length) end = m_length;
    if(start>end)
    {
        double tmp = start;
        start = end;
        end = tmp;
    }

    unsigned int seg = 0;
    double L = 0;

    while(L<=start && seg<m_spline.size())
    {
        L += m_spline.at(seg).getParametricLength();
        seg++;
    }
    int nStart = seg-1;

    while(L<=end && seg<m_spline.size())
    {
        L += m_spline.at(seg).getParametricLength();
        seg++;
    }

    int nEnd = seg-2;

    int nbPoints =  nEnd - nStart + 1;

    if(nbPoints < 3)
    {
        //std::cout << "usNeedleModelSpline::getCurvatureFromNeedleShape: not enough points" << std::endl;
        //std::cout << "first = " << first << " last = " << last << " nbPoints = " << nbPoints << std::endl;
        return 0;
    }

    // Create data matrix with centered vectors
    vpMatrix M(nbPoints,3);
    vpRowVector mean(3,0);

    for(int i=0 ; i<nbPoints; i++) M.insert(m_spline.at(nStart+i).getPoint(m_spline.at(nStart+i).getEndParameter()).t(),i,0);

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
/*
    usPolynomialCurve3D seg(m_spline.back());

    vpColVector dX_dl = seg.getDerivative(seg.getParametricLength(),2);
    if(dX_dl.euclideanNorm()<std::numeric_limits<double>::epsilon()) return 0;

    vpColVector d2X_dl2 = seg.getDerivative(seg.getParametricLength(),2);

    double k = vpColVector::crossProd(dX_dl, d2X_dl2).euclideanNorm() / pow(dX_dl.euclideanNorm(),3);

    center3D = seg.getEndPoint() + 1/k * ( d2X_dl2 - 1/pow(dX_dl.euclideanNorm(),2) * vpColVector::dotProd(dX_dl, d2X_dl2)*dX_dl);
    direction3D = vpColVector::crossProd(dX_dl, d2X_dl2).normalize();
    return k;*/
}

void usNeedleModelSpline::showNeedlePoints() const
{
    std::cout << "Needle " << this << ": " << m_spline.size()+1 <<" NeedlePoints: \n";
    for(unsigned int i=0 ; i<m_spline.size() ; i++)
    {
        std::cout << "\t Point " << i << ":\n";
        vpColVector p = m_spline.at(i).getPoint(m_spline.at(i).getStartParameter());
        for(int j=0 ; j<3 ; j++)
        {
            std::cout << "\t\t " << p[j] << "\n";
        }
    }
    std::cout << "\t Point " << m_spline.size() << ":\n";
    vpColVector p = m_spline.back().getPoint(m_spline.back().getEndParameter());
    for(int j=0 ; j<3 ; j++)
    {
        std::cout << "\t\t " << p[j] << "\n";
    }
    std::cout << std::endl;
}

void usNeedleModelSpline::showNeedleDirections() const
{
    std::cout << "Needle " << this << ": " << m_spline.size() <<" NeedleDirections: \n";
    for(unsigned int i=0 ; i<m_spline.size() ; i++)
    {
        std::cout << "\t Direction " << i << ":\n";
        vpColVector d = m_spline.at(i).getTangent(m_spline.at(i).getStartParameter());
        for(int j=0 ; j<3 ; j++)
        {
            std::cout << "\t\t " << d[j] << "\n";
        }
    }
    std::cout << "\t Direction " << m_spline.size() << ":\n";
    vpColVector d = m_spline.back().getTangent(m_spline.back().getEndParameter());
    for(int j=0 ; j<3 ; j++)
    {
        std::cout << "\t\t " << d[j] << "\n";
    }
    std::cout << std::endl;
}

void usNeedleModelSpline::showNeedleSegmentCoef() const
{
    std::cout << "Needle " << this << ": " << m_spline.size() <<" NeedleSegmentCoef: \n";
    for(unsigned int i=0 ; i<m_spline.size() ; i++)
    {
        std::cout << "\t Segment " << i << ":\n";
        vpMatrix m = m_spline.at(i).getPolynomialCoefficients();
        for(int j=0 ; j<3 ; j++)
        {
            std::cout << "\t\t " << std::setw(15) << m[j][0] << " " << std::setw(15) << m[j][1]<< " " << std::setw(15) << m[j][2]<< " " << std::setw(15) << m[j][3] << "\n";
        }
    }
    std::cout << std::endl;
}

void usNeedleModelSpline::showNeedleSegmentLength() const
{
    std::cout << "Needle " << this << ": " << m_spline.size() <<" NeedleSegmentLength: \n";
    for(unsigned int i=0 ; i<m_spline.size() ; i++)
    {
        std::cout << "\t Segment " << i << ": " << m_spline.at(i).getParametricLength() << ":\n";
    }
    std::cout << std::endl;
}

std::ostream &operator<<(std::ostream &s, const usNeedleModelSpline &needle)
{
    s << *((usNeedleModelBaseTip*)(&needle));

    s << "usNeedleModelSpline\n";
    s << needle.m_length << '\n';
    s << needle.m_outerDiameter << '\n';
    s << needle.m_insideDiameter << '\n';
    s << needle.m_needleYoungModulus << '\n';

    int n = needle.m_spline.size();
    s << n << '\n';
    // TODO DATA SAVING for(int i=0 ; i<n ; i++) s << needle.m_spline.at(i);

    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usNeedleModelSpline &needle)
{
    s >> (*(usNeedleModelBaseTip*)(&needle));

    char c[26];
    s >> c;
    if(strcmp(c,"usNeedleModelSpline"))
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleModelSpline data");
        throw e;
    }
    s >> needle.m_length;
    s >> needle.m_outerDiameter;
    s >> needle.m_insideDiameter;
    s >> needle.m_needleYoungModulus;

    int n = 0;
    s >> n;
    needle.m_spline.clear();
    needle.m_spline.resize(n);
    // TODO DATA SAVING for(int i=0 ; i<n ; i++) s >> needle.m_spline.at(i);

    return s;
}

std::ostream &operator<<=(std::ostream &s, const usNeedleModelSpline &needle)
{
    s <<= *((usNeedleModelBaseTip*)(&needle));

    s.write("usNeedleModelSpline",18);
    s.write((char*)&(needle.m_length), sizeof(double));
    s.write((char*)&(needle.m_outerDiameter), sizeof(double));
    s.write((char*)&(needle.m_insideDiameter), sizeof(double));
    s.write((char*)&(needle.m_needleYoungModulus), sizeof(double));

    int n = needle.m_spline.size();
    s.write((char*)&n, sizeof(int));
    // TODO DATA SAVING for(int i=0 ; i<n ; i++) s <<= needle.m_spline.at(i);

    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usNeedleModelSpline &needle)
{
    s >>= *((usNeedleModelBaseTip*)(&needle));

    char c[18];
    s.read(c,18);
    if(strcmp(c,"usNeedleModelSpline"))
    {
        vpException e(vpException::ioError, "Stream does not contain usNeedleModelSpline data");
        throw e;
    }
    s.read((char*)&(needle.m_length), sizeof(double));
    s.read((char*)&(needle.m_outerDiameter), sizeof(double));
    s.read((char*)&(needle.m_insideDiameter), sizeof(double));
    s.read((char*)&(needle.m_needleYoungModulus), sizeof(double));

    int n =0;
    s.read((char*)&n, sizeof(int));
    needle.m_spline.clear();
    needle.m_spline.resize(n);
    // TODO DATA SAVING for(int i=0 ; i<n ; i++) s >>= needle.m_spline.at(i);

    return s;
}
