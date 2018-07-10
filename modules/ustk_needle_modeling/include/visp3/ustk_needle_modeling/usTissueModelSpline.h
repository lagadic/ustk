#ifndef __usTissueModelSpline_h
#define __usTissueModelSpline_h

#include <iostream>

#include <visp3/ustk_core/usBSpline3D.h>
#include <visp3/ustk_core/usOrientedPlane3D.h>


class VISP_EXPORT usTissueModelSpline
{
public:

    //! Tissue parameters

    usOrientedPlane3D m_surface;
    usBSpline3D m_path;

public:

    //! Constructors, destructor

    usTissueModelSpline();
    usTissueModelSpline(const usTissueModelSpline &tissue);
    virtual ~usTissueModelSpline();
    const usTissueModelSpline &operator=(const usTissueModelSpline &tissue);

    virtual usTissueModelSpline* clone() const; // Polymorph copy method

    //! Parameters setters and getters

    const usOrientedPlane3D &accessSurface() const;
    usOrientedPlane3D &accessSurface();
    const usBSpline3D &accessPath() const;
    usBSpline3D &accessPath();

    bool moveInWorldFrame(const vpHomogeneousMatrix &H);
    bool moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz);
    bool move(const vpHomogeneousMatrix &H);
    bool move(double x, double y, double z, double tx, double ty, double tz);
    bool setPose(const vpPoseVector &p);
    vpPoseVector getPose() const;
        
    //! Data saving

        //! Text
        friend std::ostream &operator<<(std::ostream &s, const usTissueModelSpline &tissue);
        friend std::istream &operator>>(std::istream &s, usTissueModelSpline &tissue);
        //! Binary
        friend std::ostream &operator<<=(std::ostream &s, const usTissueModelSpline &tissue);
        friend std::istream &operator>>=(std::istream &s, usTissueModelSpline &tissue);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usTissueModelSpline &tissue);
VISP_EXPORT std::istream &operator>>(std::istream &s, usTissueModelSpline &tissue);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usTissueModelSpline &tissue);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usTissueModelSpline &tissue);

#endif // __usTissueModelSpline_h
