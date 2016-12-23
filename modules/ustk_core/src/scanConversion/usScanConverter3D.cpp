#include<visp3/ustk_core/usScanConverter3D.h>


/**
 * Default constructor.
 */
usScanConverter3D::usScanConverter3D() :
  _VpreScan(),
  _VpostScan(),
  _resolution(),
  _SweepInZdirection(true)
{
}

/**
 * Initialisation constructor.
 * @param V Pre-scan image to convert, with settings filled (transducer and motor).
 * @param [out] postScanImage Reference of post-scan image to write.
 * @param down Downsampling factor (sample number divided by this number).
 */
usScanConverter3D::usScanConverter3D(const usImagePreScan3D<unsigned char> &preScanImage, int down) :
  _VpreScan(),
  _VpostScan(),
  _resolution(),
  _SweepInZdirection(true)
{
  this->init(preScanImage, down);
}

/**
 * Initialisation method.
 * @param preScanImage Pre-scan image to convert, with settings filled (transducer and motor).
 * @param [out] postScanImage Reference of post-scan image to write.
 * @param down Down-sampling factor.
 */
void usScanConverter3D::init(const usImagePreScan3D<unsigned char> &preScanImage,int down)
{
  if(!preScanImage.isTransducerConvex() || !(preScanImage.getMotorType() == usMotorSettings::TiltingMotor))
    throw(vpException(vpException::functionNotImplementedError, "3D scan-conversion available only for convex transducer and tilting motor"));

  _VpreScan = preScanImage;
  _resolution = down * _VpreScan.getAxialResolution();

  int X = _VpreScan.getDimX();
  int Y = _VpreScan.getDimY();
  int Z = _VpreScan.getDimZ();

  double xmax;
  double ymin;
  double ymax;
  double zmax;

  usScanConverter3D::convertPreScanCoordToPostScanCoord(X, 0.0, Z, NULL , &ymin, NULL);
  usScanConverter3D::convertPreScanCoordToPostScanCoord(X/2.0, (double)Y, Z/2.0, NULL , &ymax, NULL);
  usScanConverter3D::convertPreScanCoordToPostScanCoord((double)X, (double)Y, Z/2.0, &xmax, NULL, NULL);
  usScanConverter3D::convertPreScanCoordToPostScanCoord(X/2.0, (double)Y, Z, NULL, NULL, &zmax);

  m_nbX = ceil(2*xmax/_resolution);
  m_nbY = ceil((ymax-ymin)/_resolution);
  m_nbZ = ceil(2*zmax/_resolution);

  unsigned int nbXY = m_nbX*m_nbY;
  unsigned int XY = X*Y;

  VoxelWeightAndIndex m;

  for(unsigned int x=0 ; x<m_nbX ; x++)
  {
    for(unsigned int y=0 ; y<m_nbY ; y++)
    {
      for(unsigned int z=0 ; z<m_nbZ ; z++)
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
          m._outputIndex = x + m_nbX*y + nbXY*z;

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

          m._outputIndex = x + m_nbX*y + nbXY*z;

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

/**
 * Destructor.
 */
usScanConverter3D::~usScanConverter3D()
{

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
 * @return Post-scan image converted.
 */
usImagePostScan3D<unsigned char> usScanConverter3D::getVolume()
{
  return _VpostScan;
}

/**
 * Conversion method : compute the scan-conversion 3D and write the post-scan image settings.
 */
void usScanConverter3D::convert( usImagePostScan3D<unsigned char> &postScanImage)
{
  postScanImage.resize(m_nbX,m_nbY,m_nbZ);
  unsigned char *dataPost = postScanImage.getData();
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

  //writing post-scan image settings
  postScanImage.setTransducerSettings(_VpreScan);
  postScanImage.setMotorSettings(_VpreScan);
  postScanImage.setElementSpacingX(_resolution);
  postScanImage.setElementSpacingY(_resolution);
  postScanImage.setElementSpacingZ(_resolution);
  postScanImage.setScanLineDepth(_resolution * _VpreScan.getBModeSampleNumber());
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

  const double offsetPhi = 0.5*_VpreScan.getScanLinePitch()*(Nline-1);
  const double offsetTheta = 0.5*_VpreScan.getFramePitch()*Nframe;

  const double r = _VpreScan.getTransducerRadius() + j * _VpreScan.getAxialResolution();
  const double phi = i * _VpreScan.getScanLinePitch() - offsetPhi;
  const double theta = (sweepInZdirection?1:-1) * (_VpreScan.getFramePitch() * Nframe * (i + Nline*k) / (Nframe*Nline-1) - offsetTheta);

  const double cPhi = cos(phi);

  if(x) *x = r * sin(phi);
  double radiusOffset = _VpreScan.getTransducerRadius() - _VpreScan.getMotorRadius();
  if(y) *y = (r * cPhi - radiusOffset) * cos(theta);
  if(z) *z = (r * cPhi - radiusOffset) * sin(theta);
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

  double itmp = phi / _VpreScan.getScanLinePitch() + 0.5*(Nline-1);
  if(j) *i = itmp;
  if(j) *j = (r - _VpreScan.getTransducerRadius()) / _VpreScan.getAxialResolution();
  if(k) *k = (Nframe*Nline-1) * (0.5/Nline + (sweepInZdirection?1:-1) * theta / (_VpreScan.getFramePitch() * Nframe*Nline)) - itmp/Nline;
}
