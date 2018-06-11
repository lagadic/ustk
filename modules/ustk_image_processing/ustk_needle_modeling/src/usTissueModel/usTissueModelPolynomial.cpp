
#include <visp3/ustk_needle_modeling/usTissueModelPolynomial.h>


usTissueModelPolynomial::usTissueModelPolynomial():
    m_surface(),
    m_path()
{
}

usTissueModelPolynomial::usTissueModelPolynomial(const usTissueModelPolynomial &tissue):
    m_surface(tissue.m_surface),
    m_path(tissue.m_path)
{
}

usTissueModelPolynomial::~usTissueModelPolynomial()
{
}

const usTissueModelPolynomial& usTissueModelPolynomial::operator=(const usTissueModelPolynomial &tissue)
{
    m_surface = tissue.m_surface;
    m_path = tissue.m_path;

    return *this;
}

usTissueModelPolynomial* usTissueModelPolynomial::clone() const
{
    return new usTissueModelPolynomial(*this);
}

const usOrientedPlane3D &usTissueModelPolynomial::accessSurface() const
{
    return m_surface;
}

usOrientedPlane3D &usTissueModelPolynomial::accessSurface()
{
    return m_surface;
}

const usPolynomialCurve3D &usTissueModelPolynomial::accessPath() const
{
    return m_path;
}

usPolynomialCurve3D usTissueModelPolynomial::accessPath()
{
    return m_path;
}

bool usTissueModelPolynomial::moveInWorldFrame(const vpHomogeneousMatrix &H)
{
    m_surface.moveInWorldFrame(H);
    m_path.move(H);

    return true;
}

bool usTissueModelPolynomial::moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz)
{
    return this->moveInWorldFrame(vpHomogeneousMatrix(x,y,z,tx,ty,tz));
}

bool usTissueModelPolynomial::move(const vpHomogeneousMatrix &H)
{
    vpPoseVector ptu(this->getPose());

    vpHomogeneousMatrix M(ptu);
    M = M*H*M.inverse();
    m_surface.moveInWorldFrame(M);
    m_path.move(M);

    return true;
}

bool usTissueModelPolynomial::move(double x, double y, double z, double tx, double ty, double tz)
{
    return this->move(vpHomogeneousMatrix(x,y,z,tx,ty,tz));
}

bool usTissueModelPolynomial::setPose(const vpPoseVector &p)
{
    vpPoseVector ptu(this->getPose());
    
    vpHomogeneousMatrix M(vpHomogeneousMatrix(p) * vpHomogeneousMatrix(ptu).inverse());
    m_surface.moveInWorldFrame(M);
    m_path.move(M);

    return true;
}

vpPoseVector usTissueModelPolynomial::getPose() const
{
    vpPoseVector ptu = m_surface.getPose();

    vpColVector p = m_path.getPoint(0);
    for(int i=0 ; i<3 ; i++) ptu[i] = p[i];

    return ptu;
}

std::ostream &operator<<(std::ostream &s, const usTissueModelPolynomial &tissue)
{
    s << "usTissueModelPolynomial\n";
    s << tissue.m_surface;
    // TODO DATA SAVING s << tissue.m_path;

    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usTissueModelPolynomial &tissue)
{
    char c[18];
    s >> c;
    if(strcmp(c,"usTissueModelPolynomial"))
    {
        vpException e(vpException::ioError, "Stream does not contain usTissueModelPolynomial data");
        throw e;
    }
    s >> tissue.m_surface;
    // TODO DATA SAVING s >> tissue.m_path;
    return s;
}

std::ostream &operator<<=(std::ostream &s, const usTissueModelPolynomial &tissue)
{
    s.write("usTissueModelPolynomial",18);
    s <<= tissue.m_surface;
    // TODO DATA SAVING s <<= tissue.m_path;

    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usTissueModelPolynomial &tissue)
{
    char c[18];
    s.read(c,18);
    if(strcmp(c,"usTissueModelPolynomial"))
    {
        vpException e(vpException::ioError, "Stream does not contain usTissueModelPolynomial data");
        throw e;
    }
    s >>= tissue.m_surface;
    // TODO DATA SAVING s >>= tissue.m_path;
    return s;
}
