#ifndef usBSpline3D_h
#define usBSpline3D_h

#include <iostream>
#include <vector>

#include <visp3/core/vpColor.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/ustk_needle_detection/usPolynomialCurve3D.h>


class usBSpline3D
{
protected:

    //! Model Parameters

        //! Needle

        std::vector<usPolynomialCurve3D> m_spline;

public:

    //! Constructors, destructor

    usBSpline3D();
    usBSpline3D(const usBSpline3D &spline);
    virtual ~usBSpline3D();
    const usBSpline3D &operator=(const usBSpline3D &spline);

    virtual usBSpline3D* clone() const {return new usBSpline3D(*this);} // Polymorph copy method

    //! Parameters setters and getters

        int getNbSegments() const;
        double getParametricLength() const;
        double getLength(int nbSubSeg=50) const;

    //! Spline definition

        void addSegment(const usPolynomialCurve3D &seg);
        void insertSegment(int i, const usPolynomialCurve3D &seg);
        void setSegment(int i, const usPolynomialCurve3D &poly);
        void removeLastSegment();
        void removeSegment(int i);
        void removeSegments(int i, int j);
        void clear();

        void defineFromPoints(const std::vector<vpColVector> &points, const std::vector<double> &lengths, int order = 3);

        const usPolynomialCurve3D &accessSegment(int i) const;
        const usPolynomialCurve3D &accessLastSegment() const;
        usPolynomialCurve3D &accessSegment(int i);
        usPolynomialCurve3D &accessLastSegment();
        usBSpline3D getSubSpline(double a, double b) const;

    //! Move

        bool move(const vpHomogeneousMatrix &H);
        bool move(double x, double y, double z, double tx, double ty, double tz);

    //! Measure model information

        //! Needle position

        vpColVector getPoint(double param) const;
        vpColVector getTangent(double param) const;
        double getDistanceFromPoint(const vpColVector &point, double start = 0, double stop = -1, double threshold = 1e-5) const;
        bool getParametersFromLength(double l, int &index, double &param) const;

        //! Curvature

        double getCurvatureFromShape(double start, double end, vpColVector &center3D, vpColVector &direction3D) const;
};

#endif // usBSpline3D_h
