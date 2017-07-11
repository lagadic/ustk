#include<visp3/ustk_core/usPreScanToPostScan3DConverter.h>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

/**
 * Default constructor.
 */
usPreScanToPostScan3DConverter::usPreScanToPostScan3DConverter() :
  m_VpreScan(),
  m_VpostScan(),
  m_resolution(),
  m_SweepInZdirection(true)
{
}

/**
 * Initialisation constructor.
 * @param preScanImage Pre-scan image to convert, with settings filled (transducer and motor).
 * @param down Downsampling factor (sample number divided by this number).
 */
usPreScanToPostScan3DConverter::usPreScanToPostScan3DConverter(const usImagePreScan3D<unsigned char> &preScanImage, int down) :
  m_VpreScan(),
  m_VpostScan(),
  m_resolution(),
  m_SweepInZdirection(true)
{
  this->init(preScanImage, down);
}

/**
 * Initialisation method.
 * @param preScanImage Pre-scan image to convert, with settings filled (transducer and motor).
 * @param down Down-sampling factor.
 */
void usPreScanToPostScan3DConverter::init(const usImagePreScan3D<unsigned char> &preScanImage,int down)
{
  if(!preScanImage.isTransducerConvex() || !(preScanImage.getMotorType() == usMotorSettings::TiltingMotor))
    throw(vpException(vpException::functionNotImplementedError, "3D scan-conversion available only for convex transducer and tilting motor"));

  m_VpreScan = preScanImage;
  m_resolution = down * m_VpreScan.getAxialResolution();

  int X = m_VpreScan.getDimX();
  int Y = m_VpreScan.getDimY();
  int Z = m_VpreScan.getDimZ();

  double xmax;
  double ymin;
  double ymax;
  double zmax;

  usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord(X, 0.0, Z, NULL , &ymin, NULL);
  usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord(X/2.0, (double)Y, Z/2.0, NULL , &ymax, NULL);
  usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)X, (double)Y, Z/2.0, &xmax, NULL, NULL);
  usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord(X/2.0, (double)Y, Z, NULL, NULL, &zmax);

  m_nbX = (unsigned int) ceil(2*xmax/m_resolution);
  m_nbY = (unsigned int) ceil((ymax-ymin)/m_resolution);
  m_nbZ = (unsigned int) ceil(2*zmax/m_resolution);

  unsigned int nbXY = m_nbX*m_nbY;
  unsigned int XY = X*Y;

  usVoxelWeightAndIndex m;

  for(unsigned int x=0 ; x<m_nbX ; x++)
  {
    double xx = m_resolution*x-xmax;
    for(unsigned int y=0 ; y<m_nbY ; y++)
    {
      double yy = ymin+m_resolution*y;
/*  BUG WHEN USING OPENMP FOR INIT : some black voxels appears in the middle of the image with no reason
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif*/
      for(unsigned int z=0 ; z<m_nbZ ; z++)
      {
        double zz = m_resolution*z-zmax;
        double i,j,k;
        usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(xx,yy,zz,&i,&j,&k,true);

        double ii = floor(i);
        double jj = floor(j);
        double kk = floor(k);

        if(ii>=0 && jj>=0 && kk>=0 && ii+1<X && jj+1<Y && kk+1<Z)
        {
          m.m_outputIndex = x + m_nbX*y + nbXY*z;

          double u = i - ii;
          double v = j -jj;
          double w = k - kk;
          double u1 = 1-u;
          double v1 = 1-v;
          double w1 = 1-w;

          double v1w1 = v1 * w1;
          double vw1 = v * w1;
          double v1w = v1 * w;
          double vw = v * w;

          m.m_W[0] = u1 * v1w1;
          m.m_W[1] = u  * v1w1;
          m.m_W[2] = u1 * vw1;
          m.m_W[3] = u  * vw1;
          m.m_W[4] = u1 * v1w;
          m.m_W[5] = u  * v1w;
          m.m_W[6] = u1 * vw;
          m.m_W[7] = u  * vw;

          double Xjj = X * jj;
          double Xjj1 = X *(jj + 1);
          double XYKK = XY * kk;
          double XYKK1 = XY * (kk + 1);

          m.m_inputIndex[0] = (unsigned int) (ii   + Xjj  + XYKK);
          m.m_inputIndex[1] = (unsigned int)(ii+1 + Xjj  + XYKK);
          m.m_inputIndex[2] = (unsigned int)(ii   + Xjj1 + XYKK);
          m.m_inputIndex[3] = (unsigned int)(ii+1 + Xjj1 + XYKK);
          m.m_inputIndex[4] = (unsigned int)(ii   + Xjj  + XYKK1);
          m.m_inputIndex[5] = (unsigned int)(ii+1 + Xjj  + XYKK1);
          m.m_inputIndex[6] = (unsigned int)(ii   + Xjj1 + XYKK1);
          m.m_inputIndex[7] = (unsigned int)(ii+1 + Xjj1 + XYKK1);
/*#ifdef VISP_HAVE_OPENMP
#pragma omp critical
#endif*/
          m_lookupTable1.push_back(m);
        }

        usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(xx,yy,zz,&i,&j,&k,false);

        ii = floor(i);
        jj = floor(j);
        kk = floor(k);

        if(ii>=0 && jj>=0 && kk>=0 && ii+1<X && jj+1<Y && kk+1<Z)
        {
          usVoxelWeightAndIndex m;

          m.m_outputIndex = x + m_nbX*y + nbXY*z;

          double u = i - ii;
          double v = j -jj;
          double w = k - kk;
          double u1 = 1-u;
          double v1 = 1-v;
          double w1 = 1-w;

          double v1w1 = v1 * w1;
          double vw1 = v * w1;
          double v1w = v1 * w;
          double vw = v * w;

          m.m_W[0] = u1 * v1w1;
          m.m_W[1] = u  * v1w1;
          m.m_W[2] = u1 * vw1;
          m.m_W[3] = u  * vw1;
          m.m_W[4] = u1 * v1w;
          m.m_W[5] = u  * v1w;
          m.m_W[6] = u1 * vw;
          m.m_W[7] = u  * vw;

          double Xjj = X * jj;
          double Xjj1 = X *(jj + 1);
          double XYKK = XY * kk;
          double XYKK1 = XY * (kk + 1);

          m.m_inputIndex[0] = (unsigned int)(ii   + Xjj  + XYKK);
          m.m_inputIndex[1] = (unsigned int)(ii+1 + Xjj  + XYKK);
          m.m_inputIndex[2] = (unsigned int)(ii   + Xjj1 + XYKK);
          m.m_inputIndex[3] = (unsigned int)(ii+1 + Xjj1 + XYKK);
          m.m_inputIndex[4] = (unsigned int)(ii   + Xjj  + XYKK1);
          m.m_inputIndex[5] = (unsigned int)(ii+1 + Xjj  + XYKK1);
          m.m_inputIndex[6] = (unsigned int)(ii   + Xjj1 + XYKK1);
          m.m_inputIndex[7] = (unsigned int)(ii+1 + Xjj1 + XYKK1);
/*#ifdef VISP_HAVE_OPENMP
#pragma omp critical
#endif*/
          m_lookupTable2.push_back(m);

        }
      }
    }
  }
  std::cout << "LUT 1 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_lookupTable1.size() << std::endl;
  std::cout << "LUT 2 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_lookupTable2.size() << std::endl;
}

/**
 * Destructor.
 */
usPreScanToPostScan3DConverter::~usPreScanToPostScan3DConverter()
{

}

/**
 * Get post-scan volume.
 * @param V post-scan image converted.
 */
void usPreScanToPostScan3DConverter::getVolume(usImagePostScan3D<unsigned char> &V)
{
  V = m_VpostScan;
}

/**
 * Get post-scan volume.
 * @return Post-scan image converted.
 */
usImagePostScan3D<unsigned char> usPreScanToPostScan3DConverter::getVolume()
{
  return m_VpostScan;
}

/**
 * Conversion method : compute the scan-conversion 3D and write the post-scan image settings.
 * @param [out] postScanImage The result of the scan-conversion.
 * @param dataPreScan
 */
void usPreScanToPostScan3DConverter::convert( usImagePostScan3D<unsigned char> &postScanImage, const unsigned char *dataPreScan)
{
  postScanImage.resize(m_nbX,m_nbY,m_nbZ);
  unsigned char *dataPost = postScanImage.getData();
  const unsigned char *dataPre;
  if(dataPreScan==NULL)
    dataPre= m_VpreScan.getConstData();
  else
    dataPre = dataPreScan;

  if(m_SweepInZdirection)
  {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for(int i=(int)m_lookupTable1.size()-1 ; i>=0 ; i--)
    {
      double v = 0;
      for(int j=0 ; j<8 ; j++) v += m_lookupTable1[i].m_W[j] * dataPre[m_lookupTable1[i].m_inputIndex[j]];
      dataPost[m_lookupTable1[i].m_outputIndex] = (unsigned char) v;

    }
  }
  else
  {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for(int i= (int)m_lookupTable2.size()-1 ; i>=0 ; i--)
    {
      double v = 0;
      for(int j=0 ; j<8 ; j++) v += m_lookupTable2[i].m_W[j] * dataPre[m_lookupTable2[i].m_inputIndex[j]];
      dataPost[m_lookupTable2[i].m_outputIndex] = (unsigned char) v;
    }
  }

  //writing post-scan image settings
  postScanImage.setTransducerSettings(m_VpreScan);
  postScanImage.setMotorSettings(m_VpreScan);
  postScanImage.setElementSpacingX(m_resolution);
  postScanImage.setElementSpacingY(m_resolution);
  postScanImage.setElementSpacingZ(m_resolution);
  postScanImage.setScanLineDepth(m_resolution * m_VpreScan.getBModeSampleNumber());
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
void usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord(double i, double j, double k, double *x, double *y, double *z, bool sweepInZdirection)
{
  const double Nframe = m_VpreScan.getFrameNumber();
  const double Nline = m_VpreScan.getScanLineNumber();

  const double offsetPhi = 0.5*m_VpreScan.getScanLinePitch()*(Nline-1);
  const double offsetTheta = 0.5*m_VpreScan.getFramePitch()*Nframe;

  const double r = m_VpreScan.getTransducerRadius() + j * m_VpreScan.getAxialResolution();
  const double phi = i * m_VpreScan.getScanLinePitch() - offsetPhi;
  const double theta = (sweepInZdirection?1:-1) * (m_VpreScan.getFramePitch() * Nframe * (i + Nline*k) / (Nframe*Nline-1) - offsetTheta);

  const double cPhi = cos(phi);

  if(x) *x = r * sin(phi);
  double radiusOffset = m_VpreScan.getTransducerRadius() - m_VpreScan.getMotorRadius();
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
void usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(double x, double y, double z, double *i, double *j, double *k, bool sweepInZdirection)
{
  const double Nframe = m_VpreScan.getFrameNumber();
  const double Nline = m_VpreScan.getScanLineNumber();
  double radiusOffset = m_VpreScan.getTransducerRadius() - m_VpreScan.getMotorRadius();
  const double rProbe = radiusOffset + sqrt(y*y+z*z);
  const double r = sqrt(rProbe*rProbe + x*x);
  const double phi = atan(x/rProbe);
  const double theta = atan(z/y);

  double itmp = phi / m_VpreScan.getScanLinePitch() + 0.5*(Nline-1);
  if(j) *i = itmp;
  if(j) *j = (r - m_VpreScan.getTransducerRadius()) / m_VpreScan.getAxialResolution();
  if(k) *k = (Nframe*Nline-1) * (0.5/Nline + (sweepInZdirection?1:-1) * theta / (m_VpreScan.getFramePitch() * Nframe*Nline)) - itmp/Nline;
}
