#ifndef __usVirtualSpring_h
#define __usVirtualSpring_h

#include <iostream>

#include <visp3/core/vpColVector.h>

#include <visp3/ustk_needle_detection/usOrientedPlane3D.h>


class VISP_EXPORT usVirtualSpring : public usOrientedPlane3D
{
protected:

    double m_stiffness;

    bool m_IsActive;
    bool m_IsRemovable;
    bool m_AllowPositionUpdate;
    bool m_AllowDirectionUpdate;
    bool m_AllowStiffnessUpdate;

public:

    //! Constructors, destructor

    usVirtualSpring();
    usVirtualSpring(const usVirtualSpring &spring);
    usVirtualSpring(const vpColVector &p, const vpColVector &d, double K);
    virtual ~usVirtualSpring();
    const usVirtualSpring &operator=(const usVirtualSpring &spring);

    //! Parameters setters and getters

    void setStiffness(double K);
    double getStiffness() const;
    void addStiffness(double dK);

    void Activate();
    void Deactivate();
    bool IsActive() const;
    void AllowRemoval(bool flag);
    bool IsRemovable() const;
    void AllowPositionUpdate(bool flag);
    bool IsPositionUpdateAllowed() const;
    void AllowDirectionUpdate(bool flag);
    bool IsDirectionUpdateAllowed() const;
    void AllowStiffnessUpdate(bool flag);
    bool IsStiffnessUpdateAllowed() const;

    //! Data saving

        //! Text
        friend std::ostream &operator<<(std::ostream &s, const usVirtualSpring &spg);
        friend std::istream &operator>>(std::istream &s, usVirtualSpring &spg);
        //! Binary
        friend std::ostream &operator<<=(std::ostream &s, const usVirtualSpring &spg);
        friend std::istream &operator>>=(std::istream &s, usVirtualSpring &spg);
};

VISP_EXPORT std::ostream &operator<<(std::ostream &s, const usVirtualSpring &spg);
VISP_EXPORT std::istream &operator>>(std::istream &s, usVirtualSpring &spg);

VISP_EXPORT std::ostream &operator<<=(std::ostream &s, const usVirtualSpring &spg);
VISP_EXPORT std::istream &operator>>=(std::istream &s, usVirtualSpring &spg);

#endif // __usVirtualSpring_h
