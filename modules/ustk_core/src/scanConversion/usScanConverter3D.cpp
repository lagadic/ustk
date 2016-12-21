#include<visp3/ustk_core/usScanConverter3D.h>


/**
 * Default constructor.
 */
usScanConverter3D::usScanConverter3D() :
  _VpreScan(),
  _VpostScan(),
  _resolution(DEFAULT_BSAMPLE_DISTANCE),
  _SweepInZdirection(true)
{
}

/**
 * Initialisation constructor.
* @param X pre-scan image size along X axis (scanline number).
* @param Y pre-scan image size along Y axis (Bmode sample number).
* @param Z pre-scan image size along Z axis (frame number).
* @param down Downsampling factor (sample number divided by this number).
 */
usScanConverter3D::usScanConverter3D(int X, int Y, int Z, int down) :
  usScanConverter3D()
{
  //this->init(X,Y,Z,down);
}

/**
 * Initialisation method.
* @param X pre-scan image size along X axis (scanline number).
* @param Y pre-scan image size along Y axis (Bmode sample number).
* @param Z pre-scan image size along Z axis (frame number).
* @param down Downsampling factor (sample number divided by this number).
 */
void usScanConverter3D::init(const usImagePreScan3D<unsigned char> &V, int down)
{
  std::cout << "DEFAULT_PROBE_RADIUS : " << DEFAULT_PROBE_RADIUS << std::endl;
  std::cout << "DEFAULT_MOTOR_RADIUS : " << DEFAULT_MOTOR_RADIUS << std::endl;
  std::cout << "DEFAULT_BSAMPLE_DISTANCE : " << DEFAULT_BSAMPLE_DISTANCE << std::endl;
  std::cout << "DEFAULT_RAD_PER_FRAME : " << DEFAULT_RAD_PER_FRAME << std::endl;
  std::cout << "DEFAULT_RAD_PER_LINE : " << DEFAULT_RAD_PER_LINE << std::endl;

  std::cout <<   _VpreScan << std::endl;
  _VpreScan.setAxialResolution(DEFAULT_BSAMPLE_DISTANCE);
  std::cout <<  "axial res : " << _VpreScan.getAxialResolution() << std::endl;

  _VpreScan = V;
  _resolution = down * V.getAxialResolution();

  int X = V.getDimX();
  int Y = V.getDimY();
  int Z = V.getDimZ();

  std::cout << "X :" << X << std::endl;
  std::cout << "Y :" << Y << std::endl;
  std::cout << "Z :" << Z << std::endl;

  double xmax;
  double ymin;
  double ymax;
  double zmax;

  usScanConverter3D::convertPreScanCoordToPostScanCoord(X, 0.0, Z, NULL , &ymin, NULL);

  usScanConverter3D::convertPreScanCoordToPostScanCoord(X/2.0, (double)Y, Z/2.0, NULL , &ymax, NULL);

  usScanConverter3D::convertPreScanCoordToPostScanCoord((double)X, (double)Y, Z/2.0, &xmax, NULL, NULL);
  usScanConverter3D::convertPreScanCoordToPostScanCoord(X/2.0, (double)Y, Z, NULL, NULL, &zmax);

  std::cout << "xmax :" << xmax << std::endl;
  std::cout << "ymin :" << ymin << std::endl;
  std::cout << "ymax :" << ymax << std::endl;
  std::cout << "zmax :" << zmax << std::endl;

  unsigned int nbX = ceil(2*xmax/_resolution);
  unsigned int nbY = ceil((ymax-ymin)/_resolution);
  unsigned int nbZ = ceil(2*zmax/_resolution);

  std::cout << "nbX :" << nbX << std::endl;
  std::cout << "nbY :" << nbY << std::endl;
  std::cout << "nbY :" << nbY << std::endl;

  unsigned int nbXY = nbX*nbY;
  unsigned int XY = X*Y;

  _VpostScan.resize(nbX,nbY,nbZ);

  lut1Size = nbX*nbY*nbZ;
  lut2Size = nbX*nbY*nbZ;

  _lookupTable1 = (VoxelWeightAndIndex*)malloc(nbX*nbY*nbZ*sizeof(VoxelWeightAndIndex));
  _lookupTable2 = (VoxelWeightAndIndex*)malloc(nbX*nbY*nbZ*sizeof(VoxelWeightAndIndex));

  VoxelWeightAndIndex m;

  unsigned long lut1Index = 0;
  unsigned long lut2Index = 0;

  for(unsigned int x=0 ; x<nbX ; x++)
  {
    for(unsigned int y=0 ; y<nbY ; y++)
    {
      for(unsigned int z=0 ; z<nbZ ; z++)
      {
        double xx = _resolution*x-xmax;
        double yy = ymin+_resolution*y;
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

          _lookupTable1[lut1Index] = m;
          lut1Index++;
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

          _lookupTable2[lut2Index] = m;
          lut2Index++;
        }
      }
    }
  }
}

/**
 * Destructor.
 */
usScanConverter3D::~usScanConverter3D()
{

}

/**
 * Set pre-scan volume.
* @param V pre-scan image to convert into post-scan.
 */
void usScanConverter3D::setVolume(const usImagePreScan3D<unsigned char> &V)
{
  if(V.getDimX()!=_VpreScan.getDimX() || V.getDimY()!=_VpreScan.getDimY() || V.getDimZ()!=_VpreScan.getDimZ()) throw vpException(vpException::dimensionError, "usScanConverter3D::setVolume: volume dimensions does not correspond to initial dimensions");
  _VpreScan = V;
}

/**
 * Get post-scan volume.
* @param V post-scan image converted.
 */
void usScanConverter3D::getVolume(usImagePostScan3D<unsigned char> &V)
{
  V = _VpostScan;
}

/**
 * Get post-scan volume.
* @param V post-scan image converted.
 */
usImagePostScan3D<unsigned char> usScanConverter3D::getVolume()
{
  return _VpostScan;
}

/**
 * Conversion method : compute the scan-conversion 3D.
 */
void usScanConverter3D::convert()
{
  _VpostScan.initData(0);
  unsigned char *dataPost = _VpostScan.getData();
  const unsigned char *dataPre = _VpreScan.getConstData();

  if(_SweepInZdirection)
  {
    for(int i=lut1Size-1 ; i>=0 ; i--)
    {
      double v = 0;
      for(int j=0 ; j<8 ; j++) v += _lookupTable1[i]._W[j] * dataPre[_lookupTable1[i]._inputIndex[j]];
      dataPost[_lookupTable1[i]._outputIndex] = v;
    }
  }
  else
  {
    for(int i=lut2Size-1 ; i>=0 ; i--)
    {
      double v = 0;
      for(int j=0 ; j<8 ; j++) v += _lookupTable2[i]._W[j] * dataPre[_lookupTable2[i]._inputIndex[j]];
      dataPost[_lookupTable2[i]._outputIndex] = v;
    }
  }
}

/**
 * Converts the pre-scan coordinates into post-scan coordinates of the corresponding voxel.
* @param i Position in pre-scan image : scanline coordinate.
* @param j Position in pre-scan image : sample coordinate.
* @param k Position in pre-scan image : frame coordinate.
* @param [out] x Position in the post-scan image along x axis.
* @param [out] y Position in the post-scan image along y axis.
* @param [out] z Position in the post-scan image along z axis.
* @param sweepInZdirection Motor direction.
* @todo check sweepInZdirection parameter
 */
void usScanConverter3D::convertPreScanCoordToPostScanCoord(double i, double j, double k, double *x, double *y, double *z, bool sweepInZdirection)
{
  const double Nframe = _VpreScan.getFrameNumber();
  const double Nline = _VpreScan.getScanLineNumber();
  std::cout << "Nframe :" << Nframe << std::endl;
  std::cout << "Nline :" << Nline << std::endl;


  const double offsetPhi = 0.5*_VpreScan.getScanLinePitch()*(Nline-1);
  const double offsetTheta = 0.5*_VpreScan.getFramePitch()*Nframe;

  std::cout << "offsetPhi :" << offsetPhi << std::endl;
  std::cout << "offsetTheta :" << offsetTheta << std::endl;

  const double r = _VpreScan.getTransducerRadius() + j * _VpreScan.getAxialResolution();
  const double phi = i * _VpreScan.getScanLinePitch() - offsetPhi;
  const double theta = (sweepInZdirection?1:-1) * (_VpreScan.getFramePitch() * Nframe * (i + Nline*k) / (Nframe*Nline-1) - offsetTheta);

  std::cout << "r :" << r << std::endl;
  std::cout << "phi :" << phi << std::endl;
  std::cout << "theta :" << theta << std::endl;

  const double cPhi = cos(phi);

  if(x) *x = r * sin(phi);
  double radiusOffset = _VpreScan.getTransducerRadius() - _VpreScan.getMotorRadius();
  if(y) *y = (r * cPhi - radiusOffset) * cos(theta);
  if(z) *z = (r * cPhi - radiusOffset) * sin(theta);

  std::cout << "x :" << *x << std::endl;
  std::cout << "y :" << *y << std::endl;
  std::cout << "z :" << *z << std::endl;
}

/**
 * Converts the pre-scan coordinates into post-scan coordinates of the corresponding voxel.
* @param x Position in the post-scan image along x axis.
* @param y Position in the post-scan image along y axis.
* @param z Position in the post-scan image along z axis.
* @param [out] i Position in pre-scan image : scanline coordinate.
* @param [out] j Position in pre-scan image : sample coordinate.
* @param [out] k Position in pre-scan image : frame coordinate.
* @param sweepInZdirection Motor direction.
* @todo check sweepInZdirection parameter
 */
void usScanConverter3D::convertPostScanCoordToPreScanCoord(double x, double y, double z, double *i, double *j, double *k, bool sweepInZdirection)
{
  const double Nframe = _VpreScan.getFrameNumber();
  const double Nline = _VpreScan.getScanLineNumber();
  double radiusOffset = _VpreScan.getTransducerRadius() - _VpreScan.getMotorRadius();
  const double rProbe = radiusOffset + sqrt(y*y+z*z);
  const double r = sqrt(rProbe*rProbe + x*x);
  const double phi = atan(x/rProbe);
  const double theta = atan(z/y);


  std::cout << "r :" << r << std::endl;
  std::cout << "phi :" << phi << std::endl;
  std::cout << "theta :" << theta << std::endl;

  double itmp = phi / _VpreScan.getScanLinePitch() + 0.5*(Nline-1);
  if(j) *i = itmp;
  if(j) *j = (r - _VpreScan.getTransducerRadius()) / _VpreScan.getAxialResolution();
  if(k) *k = (Nframe*Nline-1) * (0.5/Nline + (sweepInZdirection?1:-1) * theta / (_VpreScan.getFramePitch() * Nframe*Nline)) - itmp/Nline;

  std::cout << "i :" << *i << std::endl;
  std::cout << "j :" << *j << std::endl;
  std::cout << "k :" << *k << std::endl;
}


