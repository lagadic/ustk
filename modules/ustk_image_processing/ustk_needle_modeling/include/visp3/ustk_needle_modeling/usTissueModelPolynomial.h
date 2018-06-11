#ifndef __usTissueModelPolynomial_h
#define __usTissueModelPolynomial_h

#include <iostream>

#include <visp3/ustk_needle_detection/usOrientedPlane3D.h>
#include <visp3/ustk_needle_detection/usPolynomialCurve3D.h>


class VISP_EXPORT usTissueModelPolynomial
{
public:

    //! Tissue parameters

    usOrientedPlane3D m_surface;
    usPolynomialCurve3D m_path;

public:

    //! Constructors, destructor

    usTissueModelPolynomial();
    usTissueModelPolynomial(const usTissueModelPolynomial &tissue);
    virtual ~usTissueModelPolynomial();
    const usTissueModelPolynomial &operator=(const usTissueModelPolynomial &tissue);

    virtual usTissueModelPolynomial* clone() const; // Polymorph copy method

    //! Parameters setters and getters

    const usOrientedPlane3D &accessSurface() const;
    usOrientedPlane3D &accessSurface();
    const usPolynomialCurve3D &accessPath() const;
    usPolynomialCurve3D accessPath();

    bool moveInWorldFrame(const vpHomogeneousMatrix &H);
    bool moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz);
    bool move(const vpHomogeneousMatrix &H);
    bool move(double x, double y, double z, double tx, double ty, double tz);
    bool setPose(const vpPoseVector &p);
    vpPoseVector getPose() const;
        
    //! Data saving

        //! Text
        friend std::ostream &operator<<(std::ostream &s, const usTissueModelPolynomial &tissue);
        friend std::istream &operator>>(std::istream &s, usTissueModelPolynomial &tissue);
        //! Binary
        friend std::ostream &operator<<=(std::ostream &s, const usTissueModelPolynomial &tissue);
        friend std::istream &operator>>=(std::istream &s, usTissueModelPolynomial &tissue);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usTissueModelPolynomial &tissue);
VISP_EXPORT std::istream &operator>>(std::istream &s, usTissueModelPolynomial &tissue);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usTissueModelPolynomial &tissue);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usTissueModelPolynomial &tissue);

#endif // __usTissueModelPolynomial_h
