#include<visp3/ustk_core/usScanConverter3D.h>


usScanConverter3D::usScanConverter3D() :
    _VpreScan(),
    _VpostScan(),
    _resolution(DEFAULT_BSAMPLE_DISTANCE),
    _SweepInZdirection(true)
{
}

usScanConverter3D::usScanConverter3D(int X, int Y, int Z, int down) :
    usScanConverter3D()
{
    this->init(X,Y,Z,down);
}

void usScanConverter3D::init(int X, int Y, int Z, int down)
{
    _VpreScan.resize(X,Y,Z),
    _resolution = down*DEFAULT_BSAMPLE_DISTANCE;

    double xmin;
    double xmax;
    double ymax;
    double zmax;

    usScanConverter3D::convertPreScanCoordToPostScanCoord(0.0, Y, Z, &xmin , NULL, NULL);
    usScanConverter3D::convertPreScanCoordToPostScanCoord((double)X, Y/2.0, Z/2.0, &xmax , NULL, NULL);
    usScanConverter3D::convertPreScanCoordToPostScanCoord((double)X, Y, Z/2.0, NULL, &ymax, NULL);
    usScanConverter3D::convertPreScanCoordToPostScanCoord((double)X, Y/2.0, Z, NULL, NULL, &zmax);

    unsigned int nbX = ceil((xmax-xmin)/_resolution);
    unsigned int nbY= ceil(2*ymax/_resolution);
    unsigned int nbZ = ceil(2*zmax/_resolution);
    unsigned int nbXY = nbX*nbY;
    unsigned int XY = X*Y;

    _VpostScan.resize(nbX,nbY,nbZ);

    _lookupTable1.reserve(nbX*nbY*nbZ);
    _lookupTable2.reserve(nbX*nbY*nbZ);

    VoxelWeightAndIndex m;


    for(unsigned int x=0 ; x<nbX ; x++)
    {
        for(unsigned int y=0 ; y<nbY ; y++)
        {
            for(unsigned int z=0 ; z<nbZ ; z++)
            {
                double xx = xmin+_resolution*x;
                double yy = _resolution*y-ymax;
                double zz = _resolution*z-zmax;

                double i,j,k;
                usScanConverter3D::convertPostScanCoordToPreScanCoord(xx,yy,zz,&i,&j,&k,true);

                double ii = floor(i);
                double jj = floor(j);
                double kk = floor(k);

                if(ii>=0 && jj>=0 && kk>=0 && ii+1<X && jj+1<Y && kk+1<Z)
                {
                    m._outputIndex = x + nbX*y + nbXY*z;

                    double u = i - ii;
                    double v = j -jj;
                    double w = k - kk;
                    double u1 = 1-u;
                    double v1 = 1-v;
                    double w1 = 1-w;

                    m._W[0] = u1 * v1 * w1;
                    m._W[1] = u  * v1 * w1;
                    m._W[2] = u1 * v  * w1;
                    m._W[3] = u  * v  * w1;
                    m._W[4] = u1 * v1 * w;
                    m._W[5] = u  * v1 * w;
                    m._W[6] = u1 * v  * w;
                    m._W[7] = u  * v  * w;

                    m._inputIndex[0] = ii   + X* jj    + XY* kk;
                    m._inputIndex[1] = ii+1 + X* jj    + XY* kk;
                    m._inputIndex[2] = ii   + X*(jj+1) + XY* kk;
                    m._inputIndex[3] = ii+1 + X*(jj+1) + XY* kk;
                    m._inputIndex[4] = ii   + X* jj    + XY*(kk+1);
                    m._inputIndex[5] = ii+1 + X* jj    + XY*(kk+1);
                    m._inputIndex[6] = ii   + X*(jj+1) + XY*(kk+1);
                    m._inputIndex[7] = ii+1 + X*(jj+1) + XY*(kk+1);

                    _lookupTable1.push_back(m);
                }

                usScanConverter3D::convertPostScanCoordToPreScanCoord(xx,yy,zz,&i,&j,&k,false);

                ii = floor(i);
                jj = floor(j);
                kk = floor(k);

                if(ii>=0 && jj>=0 && kk>=0 && ii+1<X && jj+1<Y && kk+1<Z)
                {
                    VoxelWeightAndIndex m;

                    m._outputIndex = x + nbX*y + nbXY*z;

                    double u = i - ii;
                    double v = j -jj;
                    double w = k - kk;
                    double u1 = 1-u;
                    double v1 = 1-v;
                    double w1 = 1-w;

                    m._W[0] = u1 * v1 * w1;
                    m._W[1] = u  * v1 * w1;
                    m._W[2] = u1 * v  * w1;
                    m._W[3] = u  * v  * w1;
                    m._W[4] = u1 * v1 * w;
                    m._W[5] = u  * v1 * w;
                    m._W[6] = u1 * v  * w;
                    m._W[7] = u  * v  * w;

                    m._inputIndex[0] = ii   + X* jj    + XY* kk;
                    m._inputIndex[1] = ii+1 + X* jj    + XY* kk;
                    m._inputIndex[2] = ii   + X*(jj+1) + XY* kk;
                    m._inputIndex[3] = ii+1 + X*(jj+1) + XY* kk;
                    m._inputIndex[4] = ii   + X* jj    + XY*(kk+1);
                    m._inputIndex[5] = ii+1 + X* jj    + XY*(kk+1);
                    m._inputIndex[6] = ii   + X*(jj+1) + XY*(kk+1);
                    m._inputIndex[7] = ii+1 + X*(jj+1) + XY*(kk+1);

                    _lookupTable2.push_back(m);
                }
            }
        }
    }
}

usScanConverter3D::~usScanConverter3D()
{

}

void usScanConverter3D::setVolume(const usImagePreScan3D<unsigned char> &V)
{
    if(V.getDimX()!=_VpreScan.getDimX() || V.getDimY()!=_VpreScan.getDimY() || V.getDimZ()!=_VpreScan.getDimZ()) throw vpException(vpException::dimensionError, "usScanConverter3D::setVolume: volume dimensions does not correspond to initial dimensions");
    _VpreScan = V;
}

void usScanConverter3D::getVolume(usImagePostScan3D<unsigned char> &V)
{
    V = _VpostScan;
}

usImagePostScan3D<unsigned char> usScanConverter3D::getVolume()
{
    return _VpostScan;
}

void usScanConverter3D::convert()
{
    _VpostScan.initData(0);
    unsigned char *dataPost = _VpostScan.getData();
    const unsigned char *dataPre = _VpreScan.getConstData();

    if(_SweepInZdirection)
    {
        for(int i=_lookupTable1.size()-1 ; i>=0 ; i--)
        {
            double v = 0;
            for(int j=0 ; j<8 ; j++) v += _lookupTable1[i]._W[j] * dataPre[_lookupTable1[i]._inputIndex[j]];
            dataPost[_lookupTable1[i]._outputIndex] = v;
        }
    }
    else
    {
        for(int i=_lookupTable2.size()-1 ; i>=0 ; i--)
        {
            double v = 0;
            for(int j=0 ; j<8 ; j++) v += _lookupTable2[i]._W[j] * dataPre[_lookupTable2[i]._inputIndex[j]];
            dataPost[_lookupTable2[i]._outputIndex] = v;
        }
    }
}

void usScanConverter3D::convertPreScanCoordToPostScanCoord(double i, double j, double k, double *x, double *y, double *z, bool sweepInZdirection)
{
    const double Nframe = _VpreScan.getDimZ();
    const double Nline = _VpreScan.getDimY();
    const double offsetPhi = 0.5*DEFAULT_RAD_PER_LINE*(Nline-1);
    const double offsetTheta = 0.5*DEFAULT_RAD_PER_FRAME*Nframe;

    const double r = DEFAULT_PROBE_RADIUS + i * DEFAULT_BSAMPLE_DISTANCE;
    const double phi = j * DEFAULT_RAD_PER_LINE - offsetPhi;
    const double theta = (sweepInZdirection?1:-1) * (DEFAULT_RAD_PER_FRAME * Nframe * (j + Nline*k) / (Nframe*Nline-1) - offsetTheta);

    const double cPhi = cos(phi);

    if(x) *x = (r * cPhi - DEFAULT_CENTER_OFFSET) * cos(theta);
    if(y) *y = r * sin(phi);
    if(z) *z = (r * cPhi - DEFAULT_CENTER_OFFSET) * sin(theta);
}

void usScanConverter3D::convertPostScanCoordToPreScanCoord(double x, double y, double z, double *i, double *j, double *k, bool sweepInZdirection)
{
    const double Nframe = _VpreScan.getDimZ();
    const double Nline = _VpreScan.getDimY();
    const double rProbe = DEFAULT_CENTER_OFFSET + sqrt(x*x+z*z);
    const double r = sqrt(rProbe*rProbe + y*y);
    const double phi = atan(y/rProbe);
    const double theta = atan(z/x);

    if(i) *i = (r - DEFAULT_PROBE_RADIUS) / DEFAULT_BSAMPLE_DISTANCE;
    double jtmp = phi / DEFAULT_RAD_PER_LINE + 0.5*(Nline-1);
    if(j) *j = jtmp;
    if(k) *k = (Nframe*Nline-1) * (0.5/Nline + (sweepInZdirection?1:-1) * theta / (DEFAULT_RAD_PER_FRAME * Nframe*Nline)) - jtmp/Nline;
}


