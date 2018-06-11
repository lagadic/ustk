
#include <visp3/ustk_needle_modeling/usTissueModelSpline.h>


usTissueModelSpline::usTissueModelSpline():
    m_surface(),
    m_path()
{
}

usTissueModelSpline::usTissueModelSpline(const usTissueModelSpline &tissue):
    m_surface(tissue.m_surface),
    m_path(tissue.m_path)
{
}

usTissueModelSpline::~usTissueModelSpline()
{
}

const usTissueModelSpline& usTissueModelSpline::operator=(const usTissueModelSpline &tissue)
{
    m_surface = tissue.m_surface;
    m_path = tissue.m_path;

    return *this;
}

usTissueModelSpline* usTissueModelSpline::clone() const
{
    return new usTissueModelSpline(*this);
}

const usOrientedPlane3D &usTissueModelSpline::accessSurface() const
{
    return m_surface;
}

usOrientedPlane3D &usTissueModelSpline::accessSurface()
{
    return m_surface;
}

const usBSpline3D &usTissueModelSpline::accessPath() const
{
    return m_path;
}

usBSpline3D usTissueModelSpline::accessPath()
{
    return m_path;
}

bool usTissueModelSpline::moveInWorldFrame(const vpHomogeneousMatrix &H)
{
    m_surface.moveInWorldFrame(H);
    m_path.move(H);

    return true;
}

bool usTissueModelSpline::moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz)
{
    return this->moveInWorldFrame(vpHomogeneousMatrix(x,y,z,tx,ty,tz));
}

bool usTissueModelSpline::move(const vpHomogeneousMatrix &H)
{
    vpPoseVector ptu(this->getPose());

    vpHomogeneousMatrix M(ptu);
    M = M*H*M.inverse();
    m_surface.moveInWorldFrame(M);
    m_path.move(M);

    return true;
}

bool usTissueModelSpline::move(double x, double y, double z, double tx, double ty, double tz)
{
    return this->move(vpHomogeneousMatrix(x,y,z,tx,ty,tz));
}

bool usTissueModelSpline::setPose(const vpPoseVector &p)
{
    vpPoseVector ptu(this->getPose());
    
    vpHomogeneousMatrix M(vpHomogeneousMatrix(p) * vpHomogeneousMatrix(ptu).inverse());
    m_surface.moveInWorldFrame(M);
    m_path.move(M);

    return true;
}

vpPoseVector usTissueModelSpline::getPose() const
{
    vpPoseVector ptu = m_surface.getPose();

    if(m_path.getNbSegments()>0)
    {
        vpColVector p = m_path.getPoint(0);
        for(int i=0 ; i<3 ; i++) ptu[i] = p[i];
    }
    
    return ptu;
}

std::ostream &operator<<(std::ostream &s, const usTissueModelSpline &tissue)
{
    s << "usTissueModelSpline\n";
    s << tissue.m_surface;
    // TODO DATA SAVING s << tissue.m_path;

    s.flush();
    return s;
}

std::istream &operator>>(std::istream &s, usTissueModelSpline &tissue)
{
    char c[18];
    s >> c;
    if(strcmp(c,"usTissueModelSpline"))
    {
        vpException e(vpException::ioError, "Stream does not contain usTissueModelSpline data");
        throw e;
    }
    s >> tissue.m_surface;
    // TODO DATA SAVING s >> tissue.m_path;
    return s;
}

std::ostream &operator<<=(std::ostream &s, const usTissueModelSpline &tissue)
{
    s.write("usTissueModelSpline",18);
    s <<= tissue.m_surface;
    // TODO DATA SAVING s <<= tissue.m_path;

    s.flush();
    return s;
}

std::istream &operator>>=(std::istream &s, usTissueModelSpline &tissue)
{
    char c[18];
    s.read(c,18);
    if(strcmp(c,"usTissueModelSpline"))
    {
        vpException e(vpException::ioError, "Stream does not contain usTissueModelSpline data");
        throw e;
    }
    s >>= tissue.m_surface;
    // TODO DATA SAVING s >>= tissue.m_path;
    return s;
}
