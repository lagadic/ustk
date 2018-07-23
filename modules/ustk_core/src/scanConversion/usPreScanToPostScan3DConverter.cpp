/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

#include <visp3/ustk_core/usPreScanToPostScan3DConverter.h>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

/**
 * Default constructor.
 */
usPreScanToPostScan3DConverter::usPreScanToPostScan3DConverter()
  : m_VpreScan(), m_downSamplingFactor(1), m_resolution(), m_SweepInZdirection(true), m_initDone(false)
{
}

/**
 * Initialisation constructor.
 * @param preScanImage Pre-scan image to convert, with settings filled (transducer and motor).
 * @param down Downsampling factor (sample number divided by this number).
 */
usPreScanToPostScan3DConverter::usPreScanToPostScan3DConverter(const usImagePreScan3D<unsigned char> &preScanImage,
                                                               double down)
  : m_VpreScan(), m_downSamplingFactor(1), m_resolution(), m_SweepInZdirection(true), m_initDone(false)
{
  this->init(preScanImage, down);
}

/**
 * Initialisation method.
 * @param preScanImage Pre-scan image to convert, with settings filled (transducer and motor).
 * @param down Down-sampling factor.
 */
void usPreScanToPostScan3DConverter::init(const usImagePreScan3D<unsigned char> &preScanImage, double down)
{
  if (!preScanImage.isTransducerConvex() || !(preScanImage.getMotorType() == usMotorSettings::TiltingMotor))
    throw(vpException(vpException::functionNotImplementedError,
                      "3D scan-conversion available only for convex transducer and tilting motor"));

  if (down <= 0)
    throw(vpException(vpException::badValue, "downsampling factor should b positive"));
  
  
  // compare pre-scan image parameters, to avoid recomputing all the init process if parameters are the same
  if (((usMotorSettings)m_VpreScan) == ((usMotorSettings)preScanImage) &&
      ((usImagePreScanSettings)m_VpreScan) == ((usImagePreScanSettings)preScanImage) &&
      m_VpreScan.getWidth() == preScanImage.getWidth() && m_VpreScan.getHeight() == preScanImage.getHeight() &&
      m_VpreScan.getNumberOfFrames() == preScanImage.getNumberOfFrames() &&
      m_resolution == down * m_VpreScan.getAxialResolution() &&
      m_downSamplingFactor == down) {
    m_VpreScan = preScanImage; // update image content
    return;
  }

  m_VpreScan = preScanImage;
  m_resolution = down * m_VpreScan.getAxialResolution();
  m_downSamplingFactor = down;

  int X = m_VpreScan.getWidth();
  int Y = m_VpreScan.getHeight();
  int Z = m_VpreScan.getNumberOfFrames();

  double xmax;
  double ymin;
  double ymax;
  double zmax;

  usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord(0.0, X, Z, &ymin, NULL, NULL);
  usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, X / 2.0, Z / 2.0, &ymax, NULL, NULL);
  usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, (double)X, Z / 2.0, NULL, &xmax, NULL);
  usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, X / 2.0, Z, NULL, NULL, &zmax);

  m_nbX = (unsigned int)ceil(2 * xmax / m_resolution);
  m_nbY = (unsigned int)ceil((ymax - ymin) / m_resolution);
  m_nbZ = (unsigned int)ceil(2 * zmax / m_resolution);

  unsigned int nbXY = m_nbX * m_nbY;
  unsigned int XY = X * Y;
  
  try
  {
      long int LUTmaxSize = (long int)m_nbX*(long int)m_nbY*(long int)m_nbZ;
      if(m_lookupTable1.size()>0) m_lookupTable1.clear();
      if(m_lookupTable2.size()>0) m_lookupTable2.clear();
      // reserve to avoid reallocation during the LUT filling
      m_lookupTable1.reserve(LUTmaxSize);
      m_lookupTable2.reserve(LUTmaxSize);
  }
  catch(std::exception &e)
  {
      throw vpException(vpException::ioError, "usPreScanToPostScan3DConverter::init: %s \n Volume should maybe be downsampled", e.what());
  }

  for (unsigned int x = 0; x < m_nbX; x++) {
    double xx = m_resolution * x - xmax;
    for (unsigned int y = 0; y < m_nbY; y++) {
      double yy = ymin + m_resolution * y;
      #ifdef VISP_HAVE_OPENMP
      #pragma omp parallel for
      #endif
      for (unsigned int z = 0; z < m_nbZ; z++) {
        double zz = m_resolution * z - zmax;
        double i, j, k;
        usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k, true);

        double ii = floor(i);
        double jj = floor(j);
        double kk = floor(k);

        if (ii >= 0 && jj >= 0 && kk >= 0 && ii + 1 < X && jj + 1 < Y && kk + 1 < Z) {
          usVoxelWeightAndIndex m;
          
          m.m_outputIndex = x + m_nbX * y + nbXY * z;

          double u = i - ii;
          double v = j - jj;
          double w = k - kk;
          double u1 = 1 - u;
          double v1 = 1 - v;
          double w1 = 1 - w;

          double v1w1 = v1 * w1;
          double vw1 = v * w1;
          double v1w = v1 * w;
          double vw = v * w;

          m.m_W[0] = u1 * v1w1;
          m.m_W[1] = u * v1w1;
          m.m_W[2] = u1 * vw1;
          m.m_W[3] = u * vw1;
          m.m_W[4] = u1 * v1w;
          m.m_W[5] = u * v1w;
          m.m_W[6] = u1 * vw;
          m.m_W[7] = u * vw;

          double Xjj = X * jj;
          double Xjj1 = X * (jj + 1);
          double XYKK = XY * kk;
          double XYKK1 = XY * (kk + 1);

          m.m_inputIndex[0] = (unsigned int)(ii + Xjj + XYKK);
          m.m_inputIndex[1] = (unsigned int)(ii + 1 + Xjj + XYKK);
          m.m_inputIndex[2] = (unsigned int)(ii + Xjj1 + XYKK);
          m.m_inputIndex[3] = (unsigned int)(ii + 1 + Xjj1 + XYKK);
          m.m_inputIndex[4] = (unsigned int)(ii + Xjj + XYKK1);
          m.m_inputIndex[5] = (unsigned int)(ii + 1 + Xjj + XYKK1);
          m.m_inputIndex[6] = (unsigned int)(ii + Xjj1 + XYKK1);
          m.m_inputIndex[7] = (unsigned int)(ii + 1 + Xjj1 + XYKK1);
          #ifdef VISP_HAVE_OPENMP
          #pragma omp critical
          #endif
          m_lookupTable1.push_back(m);
        }

        usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k, false);

        ii = floor(i);
        jj = floor(j);
        kk = floor(k);

        if (ii >= 0 && jj >= 0 && kk >= 0 && ii + 1 < X && jj + 1 < Y && kk + 1 < Z) {
          usVoxelWeightAndIndex m;

          m.m_outputIndex = x + m_nbX * y + nbXY * z;

          double u = i - ii;
          double v = j - jj;
          double w = k - kk;
          double u1 = 1 - u;
          double v1 = 1 - v;
          double w1 = 1 - w;

          double v1w1 = v1 * w1;
          double vw1 = v * w1;
          double v1w = v1 * w;
          double vw = v * w;

          m.m_W[0] = u1 * v1w1;
          m.m_W[1] = u * v1w1;
          m.m_W[2] = u1 * vw1;
          m.m_W[3] = u * vw1;
          m.m_W[4] = u1 * v1w;
          m.m_W[5] = u * v1w;
          m.m_W[6] = u1 * vw;
          m.m_W[7] = u * vw;

          double Xjj = X * jj;
          double Xjj1 = X * (jj + 1);
          double XYKK = XY * kk;
          double XYKK1 = XY * (kk + 1);

          m.m_inputIndex[0] = (unsigned int)(ii + Xjj + XYKK);
          m.m_inputIndex[1] = (unsigned int)(ii + 1 + Xjj + XYKK);
          m.m_inputIndex[2] = (unsigned int)(ii + Xjj1 + XYKK);
          m.m_inputIndex[3] = (unsigned int)(ii + 1 + Xjj1 + XYKK);
          m.m_inputIndex[4] = (unsigned int)(ii + Xjj + XYKK1);
          m.m_inputIndex[5] = (unsigned int)(ii + 1 + Xjj + XYKK1);
          m.m_inputIndex[6] = (unsigned int)(ii + Xjj1 + XYKK1);
          m.m_inputIndex[7] = (unsigned int)(ii + 1 + Xjj1 + XYKK1);
          #ifdef VISP_HAVE_OPENMP
          #pragma omp critical
          #endif
          m_lookupTable2.push_back(m);
        }
      }
    }
  }
  std::cout << "LUT 1 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_lookupTable1.size() << std::endl;
  std::cout << "LUT 2 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_lookupTable2.size() << std::endl;
  m_initDone = true;
}

/**
 * Destructor.
 */
usPreScanToPostScan3DConverter::~usPreScanToPostScan3DConverter() {}

/**
 * Conversion method : compute the scan-conversion 3D and write the post-scan image settings.
 * @param [out] postScanImage The result of the scan-conversion.
 * @param [in] preScanImage Pre-scan image to convert.
 * @param [in] downSamplingFactor Down-sampling factor, to change the output resolution (optionnal).
 */
void usPreScanToPostScan3DConverter::convert(usImagePostScan3D<unsigned char> &postScanImage,
                                             const usImagePreScan3D<unsigned char> &preScanImage)
{
  init(preScanImage, m_downSamplingFactor);
  
  postScanImage.resize(m_nbY, m_nbX, m_nbZ);
  postScanImage.initData(0);
  unsigned char *dataPost = postScanImage.getData();
  const unsigned char *dataPre = preScanImage.getConstData();

  if (m_SweepInZdirection) {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = (int)m_lookupTable1.size() - 1; i >= 0; i--) {
      double v = 0;
      for (int j = 0; j < 8; j++)
        v += m_lookupTable1[i].m_W[j] * dataPre[m_lookupTable1[i].m_inputIndex[j]];
      dataPost[m_lookupTable1[i].m_outputIndex] = (unsigned char)v;
    }
  } else {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = (int)m_lookupTable2.size() - 1; i >= 0; i--) {
      double v = 0;
      for (int j = 0; j < 8; j++)
        v += m_lookupTable2[i].m_W[j] * dataPre[m_lookupTable2[i].m_inputIndex[j]];
      dataPost[m_lookupTable2[i].m_outputIndex] = (unsigned char)v;
    }
  }

  // writing post-scan image settings
  postScanImage.setTransducerSettings(m_VpreScan);
  postScanImage.setMotorSettings(m_VpreScan);
  postScanImage.setElementSpacingX(m_resolution);
  postScanImage.setElementSpacingY(m_resolution);
  postScanImage.setElementSpacingZ(m_resolution);
  postScanImage.setScanLineDepth(m_resolution * m_VpreScan.getBModeSampleNumber());
}

/**
 * Converts the pre-scan coordinates into post-scan coordinates of the corresponding voxel.
 * @param i_preScan Position in pre-scan image : sample coordinate.
 * @param j_preScan Position in pre-scan image : scanline  coordinate.
 * @param k_preScan Position in pre-scan image : frame coordinate.
 * @param [out] i_postScan Position in the post-scan image : sample coordinate.
 * @param [out] j_postScan Position in the post-scan image  : scanline  coordinate.
 * @param [out] k_postScan Position in the post-scan image : frame coordinate.
 * @param sweepInZdirection Motor direction.
 * @todo check sweepInZdirection parameter
 */
void usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord(double i_preScan, double j_preScan,
                                                                        double k_preScan, double *i_postScan,
                                                                        double *j_postScan, double *k_postScan,
                                                                        bool sweepInZdirection)
{
  const double Nframe = m_VpreScan.getFrameNumber();
  const double Nline = m_VpreScan.getScanLineNumber();

  const double offsetPhi = 0.5 * m_VpreScan.getScanLinePitch() * (Nline - 1);
  const double offsetTheta = 0.5 * m_VpreScan.getFramePitch() * Nframe;

  const double r = m_VpreScan.getTransducerRadius() + i_preScan * m_VpreScan.getAxialResolution();
  const double phi = j_preScan * m_VpreScan.getScanLinePitch() - offsetPhi;
  //const double theta = (sweepInZdirection ? 1 : -1) * (m_VpreScan.getFramePitch() * Nframe * (j_preScan + Nline * k_preScan) / (Nframe * Nline - 1) - offsetTheta);
  const double theta = (m_VpreScan.getFramePitch() * Nframe * ((sweepInZdirection ? j_preScan : Nline-1-j_preScan) + Nline * k_preScan) / (Nframe * Nline - 1) - offsetTheta);

  const double cPhi = cos(phi);

  if (j_postScan)
    *j_postScan = r * sin(phi);
  double radiusOffset = m_VpreScan.getTransducerRadius() - m_VpreScan.getMotorRadius();
  if (i_postScan)
    *i_postScan = (r * cPhi - radiusOffset) * cos(theta);
  if (k_postScan)
    *k_postScan = (r * cPhi - radiusOffset) * sin(theta);
}

/**
 * Converts the pre-scan coordinates into post-scan coordinates of the corresponding voxel.
 * @param i_postScan Position in the post-scan image : sample coordinate.
 * @param j_postScan Position in the post-scan image : scanline  coordinate.
 * @param k_postScan Position in the post-scan image : frame coordinate.
 * @param [out] i_preScan Position in pre-scan image : sample coordinate.
 * @param [out] j_preScan Position in pre-scan image : scanline coordinate.
 * @param [out] k_preScan Position in pre-scan image : frame coordinate.
 * @param sweepInZdirection Motor direction.
 * @todo check sweepInZdirection parameter
 */
void usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(double i_postScan, double j_postScan,
                                                                        double k_postScan, double *i_preScan,
                                                                        double *j_preScan, double *k_preScan,
                                                                        bool sweepInZdirection)
{
  const double Nframe = m_VpreScan.getFrameNumber();
  const double Nline = m_VpreScan.getScanLineNumber();
  double radiusOffset = m_VpreScan.getTransducerRadius() - m_VpreScan.getMotorRadius();
  const double rProbe = radiusOffset + sqrt(i_postScan * i_postScan + k_postScan * k_postScan);
  const double r = sqrt(rProbe * rProbe + j_postScan * j_postScan);
  const double phi = atan(j_postScan / rProbe);
  const double theta = atan(k_postScan / i_postScan);

  double jtmp = phi / m_VpreScan.getScanLinePitch() + 0.5 * (Nline - 1);
  if (j_preScan)
    *j_preScan = jtmp;
  if (i_preScan)
    *i_preScan = (r - m_VpreScan.getTransducerRadius()) / m_VpreScan.getAxialResolution();
  //if (k_preScan) {
  // *k_preScan =
  //      (Nframe * Nline - 1) *
  //          (0.5 / Nline + (sweepInZdirection ? 1 : -1) * theta / (m_VpreScan.getFramePitch() * Nframe * Nline)) -
  //      jtmp / Nline;
  //}
  if (k_preScan) {
    *k_preScan = (Nframe * Nline - 1) * (0.5 / Nline + theta / (m_VpreScan.getFramePitch() * Nframe * Nline)) - (sweepInZdirection ? jtmp : Nline-1-jtmp) / Nline;
  }
}
