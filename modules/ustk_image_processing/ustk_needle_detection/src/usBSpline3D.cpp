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

#include <visp3/ustk_needle_detection/usBSpline3D.h>

#include<iomanip>

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_EIGEN3
#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/SparseLU>
#endif

#include <visp3/core/vpException.h>


usBSpline3D::usBSpline3D()
{
    
}

usBSpline3D::usBSpline3D(const usBSpline3D &spline):
    m_spline(spline.m_spline)
{

}

usBSpline3D::~usBSpline3D()
{

}

const usBSpline3D& usBSpline3D::operator=(const usBSpline3D &spline)
{
    m_spline = spline.m_spline;
    return *this;
}

usBSpline3D *usBSpline3D::clone() const
{
    return new usBSpline3D(*this);
}

int usBSpline3D::getNbSegments() const
{
    return m_spline.size();
}

double usBSpline3D::getParametricLength() const
{
    double l = 0;
    for(unsigned int i=0 ; i<m_spline.size() ; i++)
    {
        l += m_spline.at(i).getParametricLength();
    }
    return l;
}

double usBSpline3D::getLength(int nbSubSeg) const
{
    double l = 0;
    for(unsigned int i=0 ; i<m_spline.size() ; i++)
    {
        l += m_spline.at(i).getLength(nbSubSeg);
    }
    return l;
}

void usBSpline3D::addSegment(const usPolynomialCurve3D &seg)
{
    m_spline.push_back(seg);
    m_spline.back().changeCoefficientsToFitBoundaries(0, seg.getParametricLength());
}

void usBSpline3D::insertSegment(int i, const usPolynomialCurve3D &seg)
{
    m_spline.insert(m_spline.begin()+i+1, seg);
}

void usBSpline3D::setSegment(int i, const usPolynomialCurve3D &poly)
{
    m_spline.at(i) = poly;
}

void usBSpline3D::removeLastSegment()
{
    m_spline.pop_back();
}

void usBSpline3D::removeSegment(int i)
{
    m_spline.erase(m_spline.begin()+i,m_spline.begin()+i+1);
}

void usBSpline3D::removeSegments(int i, int j)
{
    m_spline.erase(m_spline.begin()+i,m_spline.begin()+j+1);
}

void usBSpline3D::clear()
{
    m_spline.clear();
}

void usBSpline3D::defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &lengths, int order)
{
    if(points.size()<2) throw vpException(vpException::dimensionError, "usBSpline3D::defineFromPoints: should have at least two points to define the curve");
    if(order<1) throw vpException(vpException::dimensionError, "usBSpline3D::defineFromPoints: order should be greater than zero");
        
    unsigned int nbSeg = points.size()-1;
    
    if(lengths.size() != nbSeg) throw vpException(vpException::dimensionError, "usBSpline3D::defineFromPoints: mismathcing number of segments and points");
       
#ifdef VISP_HAVE_EIGEN3
    int nbSegCoef = order+1;
    int nbCoef = nbSegCoef*nbSeg;
    

    typedef Eigen::Triplet<double> T;
    std::vector<T> L;

    int nbHardConstraints = (nbSeg+1) + (nbSeg-1);
    if(order>1) nbHardConstraints += (nbSeg-1);
    if(order>2) nbHardConstraints += (nbSeg-1);

    L.reserve((nbHardConstraints+nbCoef)*(nbHardConstraints+nbCoef)/10);

    Eigen::MatrixX3d A = Eigen::MatrixX3d::Zero(nbHardConstraints+nbCoef,3);
    Eigen::MatrixX3d B = Eigen::MatrixX3d::Zero(nbHardConstraints+nbCoef,3);

    //Hard constraints

    // Start conditions
    int line = nbCoef;

    // Points
    int startIndex = 0;
    for(unsigned int i=0 ; i<nbSeg ; i++)
    {
        L.push_back(T(line      , startIndex, 1));
        L.push_back(T(startIndex, line      ,-1));
        for(int dim=0 ; dim<3 ; dim++) B(line, dim) = points.at(i)[dim];
        line++;

        double segLength = lengths.at(i);
        double *tmp = new double[nbSegCoef];

        tmp[0] = 1;
        for(int j=1 ; j<nbSegCoef ; j++) tmp[j] = segLength*tmp[j-1];

        for(int j=0 ; j<nbSegCoef ; j++)
        {
            L.push_back(T(line        , startIndex+j, tmp[j]));
            L.push_back(T(startIndex+j, line        ,-tmp[j]));
        }
        for(int dim=0 ; dim<3 ; dim++) B(line, dim) = points.at(i+1)[dim];
        line++;

        delete[] tmp;
        startIndex += nbSegCoef;
    }

    // Continuity

    startIndex = 0;
    for(unsigned int i=0 ; i<nbSeg-1 ; i++)
    {
        double segLength = lengths.at(i);
        double *tmp = new double[nbSegCoef];

        if(order>1) // Order 1
        {
            tmp[0] = 0;
            tmp[1] = 1;
            for(int j=2 ; j<nbSegCoef ; j++) tmp[j] = segLength*tmp[j-1];
            for(int j=1 ; j<nbSegCoef ; j++) tmp[j] *= j;

            for(int j=1 ; j<nbSegCoef ; j++)
            {
                L.push_back(T(line        , startIndex+j, tmp[j]));
                L.push_back(T(startIndex+j, line        ,-tmp[j]));
            }
            L.push_back(T(line                  , startIndex+nbSegCoef+1,-1));
            L.push_back(T(startIndex+nbSegCoef+1, line                  , 1));
            line++;
        }
        if(order>2) // Order 2
        {
            tmp[0] = 0;
            tmp[1] = 0;
            tmp[2] = 1;
            for(int j=3 ; j<nbSegCoef ; j++) tmp[j] = segLength*tmp[j-1];
            for(int j=2 ; j<nbSegCoef ; j++) tmp[j] *= j*(j-1);

            for(int j=2 ; j<nbSegCoef ; j++)
            {
                L.push_back(T(line        , startIndex+j, tmp[j]));
                L.push_back(T(startIndex+j, line        ,-tmp[j]));
            }
            L.push_back(T(line                  , startIndex+nbSegCoef+2,-2));
            L.push_back(T(startIndex+nbSegCoef+2, line                  , 2));
            line++;
        }
        delete[] tmp;
        startIndex += nbSegCoef;
    }

    // Optimization matrix

    line = 0;
    startIndex = 0;
    for(unsigned int i=0 ; i<nbSeg ; i++)
    {
        double segLength = lengths.at(i);

        for(int j=2; j<nbSegCoef ; j++)
        {
            for(int k=2 ; k<nbSegCoef ; k++)
            {
                double c = pow(segLength,j+k-3)*j*(j-1)*k*(k-1)/(j+k-3);
                L.push_back(T(line+j, startIndex+k, c));
            }
        }
        line += nbSegCoef;
        startIndex += nbSegCoef;
    }

    try
    {
        Eigen::SparseMatrix<double> M(nbHardConstraints+nbCoef, nbHardConstraints+nbCoef);
        M.setFromTriplets(L.begin(), L.end());

        Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
        solver.compute(M);

        A = solver.solve(B);
    }
    catch(std::exception &e)
    {
        throw vpException(vpException::fatalError,"usBSpline3D::defineFromPoints: %s\n", e.what());
    }

    m_spline.clear();
    startIndex = 0;
    for(unsigned int i=0 ; i<nbSeg ; i++)
    {
        usPolynomialCurve3D seg(order);
        vpMatrix m(3,nbSegCoef);
        for(int j=0 ; j<nbSegCoef ; j++)
        {
            for(int dim=0 ; dim<3 ; dim++)
            {
                m[dim][j] = A(startIndex+j, dim);
            }
        }
        seg.setPolynomialCoefficients(m);
        seg.setParametricLength(lengths.at(i));
        //seg.setMaxCurvilinearCoordinate(seg.getLength());
        m_spline.push_back(seg);
        startIndex += nbSegCoef;
    }
#else
    throw vpException(vpException::functionNotImplementedError, "usBSpline3D::defineFromPoints: not implemented without Eigen3");
#endif
}

const usPolynomialCurve3D &usBSpline3D::accessSegment(int i) const
{
    return m_spline.at(i);
}

const usPolynomialCurve3D &usBSpline3D::accessLastSegment() const
{
    return m_spline.back();
}

usPolynomialCurve3D &usBSpline3D::accessSegment(int i)
{
    return m_spline.at(i);
}

usPolynomialCurve3D &usBSpline3D::accessLastSegment()
{
    return m_spline.back();
}

usBSpline3D usBSpline3D::getSubSpline(double a, double b) const
{
    usBSpline3D s;
    usPolynomialCurve3D p;

    if(a<=b)
    {
        unsigned int i = 0;
        double t = 0;
        double ta = 0;
        double tb = 0;

        while(t+m_spline.at(i).getParametricLength()<a && i+1<m_spline.size())
        {
            ta = 0;
            t += m_spline.at(i).getParametricLength();
            i++;
        }
        ta = a-t;
        t = a;

        while(t<b)
        {
            if(t+(m_spline.at(i).getParametricLength()-ta) > b || i+1>=m_spline.size())
            {
                tb = ta + b-t;
                p = m_spline.at(i).getSubPolynomialCurve(ta, tb);
                ta = tb;
            }
            else
            {
                tb = m_spline.at(i).getParametricLength();
                p = m_spline.at(i).getSubPolynomialCurve(ta, tb);
                ta = 0;
                i++;
            }

            p.changeCoefficientsToFitBoundaries(0,p.getParametricLength());
            s.addSegment(p);
            t += p.getParametricLength();
        }

    }
    else s = *this;

    return s;
}

bool usBSpline3D::move(const vpHomogeneousMatrix &H)
{
    for(unsigned int i=0 ; i<m_spline.size() ; i++)
    {
        m_spline.at(i).move(H);
    }
    return true;
}

bool usBSpline3D::move(double x, double y, double z, double tx, double ty, double tz)
{
    return this->move(vpHomogeneousMatrix(x,y,z,tx,ty,tz));
}


vpColVector usBSpline3D::getPoint(double l) const
{
    if(l<=0) return m_spline.front().getPoint(l);

    unsigned int seg = 0;
    double L = 0;

    while(L<=l && seg<m_spline.size())
    {
        L += m_spline.at(seg).getParametricLength();
        seg++;
    }
    seg--;
    double s = l - (L - m_spline.at(seg).getParametricLength());

    vpColVector P = m_spline.at(seg).getPoint(s);

    return P;
}

vpColVector usBSpline3D::getTangent(double param) const
{
    unsigned int seg = 0;
    double L = 0;

    while(L<=param && seg<m_spline.size())
    {
        L += m_spline.at(seg).getParametricLength();
        seg++;
    }
    seg--;
    double s = param - (L - m_spline.at(seg).getParametricLength());

    vpColVector D = m_spline.at(seg).getTangent(s);

    return D;
}

double usBSpline3D::getDistanceFromPoint(const vpColVector &P, double start, double stop, double threshold) const
{
    if(P.size() != 3) throw vpException(vpException::dimensionError, "usBSpline3D::getDistanceFromPoint: invalid point dimension");

    if(start<0) start = 0;
    double length = this->getParametricLength();
    if(stop == -1 || stop>length) stop = length;
    if(stop<0) stop = 0;
    if(start>stop) throw vpException(vpException::badValue, "usBSpline3D::getDistanceFromPoint: invalid start-stop parameters");

    double middle = (start+stop)/2;
    while( (stop-start) > threshold)
    {
        double d0 = (this->getPoint(start)-P).euclideanNorm();
        double d1 = (this->getPoint(middle)-P).euclideanNorm();
        double d2 = (this->getPoint(stop)-P).euclideanNorm();

        if(d0<=d1 && d0<d2) stop = middle;
        else if(d2<d0 && d2<=d1) start = middle;
        else
        {
            start = (start+middle)/2;
            stop = (middle+stop)/2;
        }
        middle = (start+stop)/2;
    }

    double l = (this->getPoint(middle)-P).euclideanNorm();

    return l;
}

bool usBSpline3D::getParametersFromLength(double l, int &index, double &param) const
{
    if(l<0) return false;

    unsigned int seg = 0;
    double L = 0;

    while(L<=l && seg<m_spline.size())
    {
        L += m_spline.at(seg).getParametricLength();
        seg++;
    }

    if(L<=l && seg==m_spline.size()) return false;

    seg--;
    L -= m_spline.at(seg).getParametricLength();

    index = seg;
    param = l - L;
    return true;
}

double usBSpline3D::getCurvatureFromShape(double start, double end, vpColVector &center3D, vpColVector &direction3D) const
{
    if(start<0) start = 0;
    double length = this->getParametricLength();
    if(start>length) start = length;
    if(end<0) end = 0;
    if(end>length) end = length;
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
        //std::cout << "usBSpline3D::getCurvatureFromShape: not enough points" << std::endl;
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

    /*vpColVector cp = P.getRow(0).t() - center;
    for(int i=1 ; i<nbPoints ; i++)
    {
        double dot = vpColVector::dotProd(cp, P.getRow(i).t() - center);
        if(dot<0) return 0;
    }*/

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

std::ostream &operator<<(std::ostream &s, const usBSpline3D &needle)
{
    s << "usBSpline3D\n";

    unsigned int n = needle.m_spline.size();
    s << n << '\n';
    for(unsigned int i=0 ; i<n ; i++) s << needle.m_spline.at(i);

    s.flush();
    return s;
}
