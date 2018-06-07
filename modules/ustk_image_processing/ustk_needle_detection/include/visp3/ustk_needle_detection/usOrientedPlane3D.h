#ifndef __usOrientedPlane3D_h_
#define __usOrientedPlane3D_h_

#include <iostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>


class VISP_EXPORT usOrientedPlane3D
{
protected:

    vpColVector m_direction;
    vpPoseVector m_pose;

public:

    //! Constructors, destructor

    usOrientedPlane3D();
    usOrientedPlane3D(const usOrientedPlane3D &plane);
    usOrientedPlane3D(const vpPoseVector &pose);
    usOrientedPlane3D(const vpColVector &p, const vpColVector &d);
    virtual ~usOrientedPlane3D();
    const usOrientedPlane3D &operator=(const usOrientedPlane3D &plane);

    //! Parameters setters and getters

    void setPose(const vpPoseVector &pose);
    vpPoseVector getPose() const;

    void setPosition(const vpColVector &P);
    vpColVector getPosition() const;

    void setDirection(const vpColVector &D);
    vpColVector getDirection() const;

    void moveInLocalFrame(const vpHomogeneousMatrix &H);
    void moveInLocalFrame(double x, double y, double z, double tx, double ty, double tz);
    void moveInWorldFrame(const vpHomogeneousMatrix &H);
    void moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz);

    //! Data saving

        //! Text
        friend std::ostream &operator<<(std::ostream &s, const usOrientedPlane3D &plane);
        friend std::istream &operator>>(std::istream &s, usOrientedPlane3D &plane);
        //! Binary
        friend std::ostream &operator<<=(std::ostream &s, const usOrientedPlane3D &plane);
        friend std::istream &operator>>=(std::istream &s, usOrientedPlane3D &plane);
};


VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usOrientedPlane3D &plane);
VISP_EXPORT std::istream &operator>>(std::istream &s, usOrientedPlane3D &plane);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usOrientedPlane3D &plane);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usOrientedPlane3D &plane);

#endif // __usOrientedPlane3D_h_
