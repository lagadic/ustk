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
 * Jason Chevrie
 * Marc Pouliquen
 *
 *****************************************************************************/

#include <visp3/ustk_core/usPreScanToPostScan3DConverter.h>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

#ifdef USTK_HAVE_CUDA
extern void GPUDirectConversionWrapper(unsigned char *dataPost, const unsigned char *dataPre, unsigned int m_nbX, unsigned int m_nbY, unsigned int m_nbZ, int X, int Y, int Z, double m_resolution, double xmax, double ymin, double zmax, unsigned int frameNumber, unsigned int scanLineNumber, double transducerRadius, double motorRadius, double scanLinePitch, double axialResolution, double framePitch, bool sweepInZdirection);
#endif

/**
 * Default constructor.
 */
usPreScanToPostScan3DConverter::usPreScanToPostScan3DConverter()
  : m_converterOptimizationMethod(SINGLE_THREAD_REDUCED_LOOKUP_TABLE),
    m_conversionOptimizationMethodUsedAtInit(SINGLE_THREAD_DIRECT_CONVERSION), m_GPULookupTables{NULL, NULL}, m_GPULookupTablesSize{0,0}, m_VpreScan(), m_downSamplingFactor(1),
    m_resolution(), m_SweepInZdirection(true), m_initDone(false)
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
      m_resolution == down * m_VpreScan.getAxialResolution() && m_downSamplingFactor == down &&
      m_converterOptimizationMethod == m_conversionOptimizationMethodUsedAtInit) {
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

  // empty the lookup tables
  if (m_lookupTables[0].size() > 0)
    std::vector<usVoxelWeightAndIndex>().swap(m_lookupTables[0]);
  if (m_lookupTables[1].size() > 0)
    std::vector<usVoxelWeightAndIndex>().swap(m_lookupTables[1]);
  if (m_reducedLookupTables[0].size() > 0)
    std::vector<usVoxelWeightAndIndexReducedMemory>().swap(m_reducedLookupTables[0]);
  if (m_reducedLookupTables[1].size() > 0)
    std::vector<usVoxelWeightAndIndexReducedMemory>().swap(m_reducedLookupTables[1]);
#ifdef USTK_HAVE_CUDA
  this->GPUFreeLookupTables();
#endif
  
  switch (m_converterOptimizationMethod) {
  case SINGLE_THREAD_DIRECT_CONVERSION: {
    break;
  }
  case MULTI_THREAD_DIRECT_CONVERSION: {
    break;
  }
  case GPU_DIRECT_CONVERSION: {
    break;
  }
  case SINGLE_THREAD_FULL_LOOKUP_TABLE: {
    try {
      long int LUTmaxSize = (long int)m_nbX * (long int)m_nbY * (long int)m_nbZ;
      // reserve to avoid reallocation during the LUT filling
      m_lookupTables[0].reserve(LUTmaxSize);
      m_lookupTables[1].reserve(LUTmaxSize);
    } catch (std::exception &e) {
      throw vpException(vpException::ioError, "usPreScanToPostScan3DConverter::init: using method "
                                              "SINGLE_THREAD_FULL_LOOKUP_TABLE leads to %s \n Use another optimization "
                                              "method or downsample the volume",
                        e.what());
    }
    for (unsigned int sweepingDirection = 0; sweepingDirection < 2; sweepingDirection++) {
      for (unsigned int x = 0; x < m_nbX; x++) {
        double xx = m_resolution * x - xmax;
        for (unsigned int y = 0; y < m_nbY; y++) {
          double yy = ymin + m_resolution * y;
          for (unsigned int z = 0; z < m_nbZ; z++) {
            double zz = m_resolution * z - zmax;
            double i, j, k;
            usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k,
                                                                               sweepingDirection == 0);

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

              m_lookupTables[sweepingDirection].push_back(m);
            }
          }
        }
      }
    }
    std::cout << "LUT 1 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_lookupTables[0].size() << std::endl;
    std::cout << "LUT 2 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_lookupTables[1].size() << std::endl;
    break;
  }
  case MULTI_THREAD_FULL_LOOKUP_TABLE: {
    try {
      long int LUTmaxSize = (long int)m_nbX * (long int)m_nbY * (long int)m_nbZ;
      // reserve to avoid reallocation during the LUT filling
      m_lookupTables[0].reserve(LUTmaxSize);
      m_lookupTables[1].reserve(LUTmaxSize);
    } catch (std::exception &e) {
      throw vpException(vpException::ioError, "usPreScanToPostScan3DConverter::init: using method "
                                              "MULTI_THREAD_FULL_LOOKUP_TABLE leads to %s \n Use another optimization "
                                              "method or downsample the volume",
                        e.what());
    }

    for (unsigned int sweepingDirection = 0 ; sweepingDirection < 2 ; sweepingDirection++) {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
      for (int x = 0; x < (int)m_nbX; x++) {
        double xx = m_resolution * x - xmax;
        for (unsigned int y = 0; y < m_nbY; y++) {
          double yy = ymin + m_resolution * y;
          for (unsigned int z = 0; z < m_nbZ; z++) {
            double zz = m_resolution * z - zmax;
            double i, j, k;
            usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k,
                                                                               sweepingDirection == 0);

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
              m_lookupTables[sweepingDirection].push_back(m);
            }
          }
        }
      }
    }
    std::cout << "LUT 1 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_lookupTables[0].size() << std::endl;
    std::cout << "LUT 2 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_lookupTables[1].size() << std::endl;
    break;
  }
  case GPU_FULL_LOOKUP_TABLE: {
#ifdef USTK_HAVE_CUDA
    this->GPUAllocateFullLookupTables();
    this->GPUFillFullLookupTables();
std::cout << "LUT 1 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_GPULookupTablesSize[0] << std::endl;
    std::cout << "LUT 2 size (bytes) : " << sizeof(usVoxelWeightAndIndex) * m_GPULookupTablesSize[1] << std::endl;
#else
    throw vpException(
        vpException::notImplementedError,
        "usPreScanToPostScan3DConverter::init: using method GPU_FULL_LOOKUP_TABLE is not implemented yet");
#endif
    break;
  }
  case SINGLE_THREAD_REDUCED_LOOKUP_TABLE: {
    try {
      long int LUTmaxSize = (long int)m_nbX * (long int)m_nbY * (long int)m_nbZ;
      // reserve to avoid reallocation during the LUT filling
      m_reducedLookupTables[0].reserve(LUTmaxSize);
      m_reducedLookupTables[1].reserve(LUTmaxSize);
    } catch (std::exception &e) {
      throw vpException(vpException::ioError, "usPreScanToPostScan3DConverter::init: using method "
                                              "SINGLE_THREAD_REDUCED_LOOKUP_TABLE leads to %s \n Use another "
                                              "optimization method or downsample the volume",
                        e.what());
    }
    for (unsigned int sweepingDirection = 0; sweepingDirection < 2; sweepingDirection++) {
      for (unsigned int x = 0; x < m_nbX; x++) {
        double xx = m_resolution * x - xmax;
        for (unsigned int y = 0; y < m_nbY; y++) {
          double yy = ymin + m_resolution * y;
          for (unsigned int z = 0; z < m_nbZ; z++) {
            double zz = m_resolution * z - zmax;
            double i, j, k;
            usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k,
                                                                               sweepingDirection == 0);

            double ii = floor(i);
            double jj = floor(j);
            double kk = floor(k);

            if (ii >= 0 && jj >= 0 && kk >= 0 && ii + 1 < X && jj + 1 < Y && kk + 1 < Z) {
              usVoxelWeightAndIndexReducedMemory m;

              m.m_outputIndex = x + m_nbX * y + nbXY * z;

              m.m_W[0] = i - ii;
              m.m_W[1] = j - jj;
              m.m_W[2] = k - kk;

              m.m_inputIndex = (unsigned int)(ii + X * jj + XY * kk);

              m_reducedLookupTables[sweepingDirection].push_back(m);
            }
          }
        }
      }
    }
    std::cout << "LUT 1 size (bytes) : " << sizeof(usVoxelWeightAndIndexReducedMemory) * m_reducedLookupTables[0].size()
              << std::endl;
    std::cout << "LUT 2 size (bytes) : " << sizeof(usVoxelWeightAndIndexReducedMemory) * m_reducedLookupTables[1].size()
              << std::endl;
    break;
  }
  case MULTI_THREAD_REDUCED_LOOKUP_TABLE: {
    try {
      long int LUTmaxSize = (long int)m_nbX * (long int)m_nbY * (long int)m_nbZ;
      // reserve to avoid reallocation during the LUT filling
      m_reducedLookupTables[0].reserve(LUTmaxSize);
      m_reducedLookupTables[1].reserve(LUTmaxSize);
    } catch (std::exception &e) {
      throw vpException(vpException::ioError, "usPreScanToPostScan3DConverter::init: using method "
                                              "MULTI_THREAD_REDUCED_LOOKUP_TABLE leads to %s \n Use another "
                                              "optimization method or downsample the volume",
                        e.what());
    }
    for (unsigned int sweepingDirection = 0; sweepingDirection < 2; sweepingDirection++) {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
      for (int x = 0; x < (int)m_nbX; x++) {
        double xx = m_resolution * x - xmax;
        for (unsigned int y = 0; y < m_nbY; y++) {
          double yy = ymin + m_resolution * y;
          for (unsigned int z = 0; z < m_nbZ; z++) {
            double zz = m_resolution * z - zmax;
            double i, j, k;
            usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k,
                                                                               sweepingDirection == 0);

            double ii = floor(i);
            double jj = floor(j);
            double kk = floor(k);

            if (ii >= 0 && jj >= 0 && kk >= 0 && ii + 1 < X && jj + 1 < Y && kk + 1 < Z) {
              usVoxelWeightAndIndexReducedMemory m;

              m.m_outputIndex = x + m_nbX * y + nbXY * z;

              m.m_W[0] = i - ii;
              m.m_W[1] = j - jj;
              m.m_W[2] = k - kk;

              m.m_inputIndex = (unsigned int)(ii + X * jj + XY * kk);
#ifdef VISP_HAVE_OPENMP
#pragma omp critical
#endif
              m_reducedLookupTables[sweepingDirection].push_back(m);
            }
          }
        }
      }
    }
    std::cout << "LUT 1 size (bytes) : " << sizeof(usVoxelWeightAndIndexReducedMemory) * m_reducedLookupTables[0].size()
              << std::endl;
    std::cout << "LUT 2 size (bytes) : " << sizeof(usVoxelWeightAndIndexReducedMemory) * m_reducedLookupTables[1].size()
              << std::endl;
    break;
  }
  case GPU_REDUCED_LOOKUP_TABLE: {
#ifdef USTK_HAVE_CUDA
    this->GPUAllocateReducedLookupTables();
    this->GPUFillReducedLookupTables();
    std::cout << "LUT 1 size (bytes) : " << sizeof(usVoxelWeightAndIndexReducedMemory) * m_GPULookupTablesSize[0] << std::endl;
    std::cout << "LUT 2 size (bytes) : " << sizeof(usVoxelWeightAndIndexReducedMemory) * m_GPULookupTablesSize[1] << std::endl;
#else
    throw vpException(
        vpException::notImplementedError,
        "usPreScanToPostScan3DConverter::init: using method GPU_REDUCED_LOOKUP_TABLE is not implemented yet");
#endif
    break;
  }
  }

  m_conversionOptimizationMethodUsedAtInit = m_converterOptimizationMethod;
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
 */
void usPreScanToPostScan3DConverter::convert(usImagePostScan3D<unsigned char> &postScanImage,
                                             const usImagePreScan3D<unsigned char> &preScanImage)
{
  init(preScanImage, m_downSamplingFactor);

  postScanImage.resize(m_nbY, m_nbX, m_nbZ);
  postScanImage.initData(0);
  (usTransducerSettings&)postScanImage = (const usTransducerSettings&)preScanImage;
  (usMotorSettings&)postScanImage = (const usMotorSettings&)preScanImage;
            
  unsigned char *dataPost = postScanImage.getData();
  const unsigned char *dataPre = preScanImage.getConstData();

  switch (m_converterOptimizationMethod) {
  case SINGLE_THREAD_DIRECT_CONVERSION: {
    int X = m_VpreScan.getWidth();
    int Y = m_VpreScan.getHeight();
    int Z = m_VpreScan.getNumberOfFrames();

    double xmax;
    double ymin;
    double ymax;
    double zmax;

    usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord(0.0, X, Z, &ymin, NULL, NULL);
    usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, X / 2.0, Z / 2.0, &ymax, NULL, NULL);
    usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, (double)X, Z / 2.0, NULL, &xmax,
                                                                       NULL);
    usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, X / 2.0, Z, NULL, NULL, &zmax);

    unsigned int nbXY = m_nbX * m_nbY;
    unsigned int XY = X * Y;

    for (unsigned int x = 0; x < m_nbX; x++) {
      double xx = m_resolution * x - xmax;
      for (unsigned int y = 0; y < m_nbY; y++) {
        double yy = ymin + m_resolution * y;
        for (unsigned int z = 0; z < m_nbZ; z++) {
          double zz = m_resolution * z - zmax;
          double i, j, k;
          usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k,
                                                                             m_SweepInZdirection);

          double ii = floor(i);
          double jj = floor(j);
          double kk = floor(k);

          if (ii >= 0 && jj >= 0 && kk >= 0 && ii + 1 < X && jj + 1 < Y && kk + 1 < Z) {

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

            double W[8] = {u1 * v1w1, u * v1w1, u1 * vw1, u * vw1, u1 * v1w, u * v1w, u1 * vw, u * vw};

            double Xjj = X * jj;
            double Xjj1 = X * (jj + 1);
            double XYKK = XY * kk;
            double XYKK1 = XY * (kk + 1);

            unsigned int index[8] = {(unsigned int)(ii + Xjj + XYKK),   (unsigned int)(ii + 1 + Xjj + XYKK),
                                     (unsigned int)(ii + Xjj1 + XYKK),  (unsigned int)(ii + 1 + Xjj1 + XYKK),
                                     (unsigned int)(ii + Xjj + XYKK1),  (unsigned int)(ii + 1 + Xjj + XYKK1),
                                     (unsigned int)(ii + Xjj1 + XYKK1), (unsigned int)(ii + 1 + Xjj1 + XYKK1)};

            double val = 0;
            for (int n = 0; n < 8; n++) 
              val += W[n] * dataPre[index[n]];
            
            dataPost[x + m_nbX * y + nbXY * z] = (unsigned char)val;
          }
        }
      }
    }
    break;
  }
  case MULTI_THREAD_DIRECT_CONVERSION: {
    int X = m_VpreScan.getWidth();
    int Y = m_VpreScan.getHeight();
    int Z = m_VpreScan.getNumberOfFrames();

    double xmax;
    double ymin;
    double ymax;
    double zmax;

    usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord(0.0, X, Z, &ymin, NULL, NULL);
    usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, X / 2.0, Z / 2.0, &ymax, NULL, NULL);
    usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, (double)X, Z / 2.0, NULL, &xmax,
                                                                       NULL);
    usPreScanToPostScan3DConverter::convertPreScanCoordToPostScanCoord((double)Y, X / 2.0, Z, NULL, NULL, &zmax);

    unsigned int nbXY = m_nbX * m_nbY;
    unsigned int XY = X * Y;

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int x = 0; x < (int)m_nbX; x++) {
      double xx = m_resolution * x - xmax;
      for (unsigned int y = 0; y < m_nbY; y++) {
        double yy = ymin + m_resolution * y;
        for (unsigned int z = 0; z < m_nbZ; z++) {
          double zz = m_resolution * z - zmax;
          double i, j, k;
          usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k,
                                                                             m_SweepInZdirection);

          double ii = floor(i);
          double jj = floor(j);
          double kk = floor(k);

          if (ii >= 0 && jj >= 0 && kk >= 0 && ii + 1 < X && jj + 1 < Y && kk + 1 < Z) {

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

            double W[8] = {u1 * v1w1, u * v1w1, u1 * vw1, u * vw1, u1 * v1w, u * v1w, u1 * vw, u * vw};

            double Xjj = X * jj;
            double Xjj1 = X * (jj + 1);
            double XYKK = XY * kk;
            double XYKK1 = XY * (kk + 1);

            unsigned int index[8] = {(unsigned int)(ii + Xjj + XYKK),   (unsigned int)(ii + 1 + Xjj + XYKK),
                                     (unsigned int)(ii + Xjj1 + XYKK),  (unsigned int)(ii + 1 + Xjj1 + XYKK),
                                     (unsigned int)(ii + Xjj + XYKK1),  (unsigned int)(ii + 1 + Xjj + XYKK1),
                                     (unsigned int)(ii + Xjj1 + XYKK1), (unsigned int)(ii + 1 + Xjj1 + XYKK1)};

            double val = 0;
            for (int n = 0; n < 8; n++) 
              val += W[n] * dataPre[index[n]];
#ifdef VISP_HAVE_OPENMP
#pragma omp critical
#endif
            dataPost[x + m_nbX * y + nbXY * z] = (unsigned char)val;
          }
        }
      }
    }
    break;
  }
  case GPU_DIRECT_CONVERSION: {
#ifdef USTK_HAVE_CUDA
    this->GPUDirectConversion(dataPost, dataPre);
#else
    throw vpException(
        vpException::notImplementedError,
        "usPreScanToPostScan3DConverter::convert: using method GPU_DIRECT_CONVERSION is not implemented yet");
#endif
    break;
  }
  case SINGLE_THREAD_FULL_LOOKUP_TABLE: {
    const unsigned int d = m_SweepInZdirection ? 0 : 1;
    for (int i = (int)m_lookupTables[d].size() - 1; i >= 0; i--) {
      double v = 0;
      for (int j = 0; j < 8; j++)
        v += m_lookupTables[d][i].m_W[j] * dataPre[m_lookupTables[d][i].m_inputIndex[j]];
      dataPost[m_lookupTables[d][i].m_outputIndex] = (unsigned char)v;
    }
    break;
  }
  case MULTI_THREAD_FULL_LOOKUP_TABLE: {
    const unsigned int d = m_SweepInZdirection ? 0 : 1;
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = (int)m_lookupTables[d].size() - 1; i >= 0; i--) {
      double v = 0;
      for (int j = 0; j < 8; j++)
        v += m_lookupTables[d][i].m_W[j] * dataPre[m_lookupTables[d][i].m_inputIndex[j]];
      dataPost[m_lookupTables[d][i].m_outputIndex] = (unsigned char)v;
    }
    break;
  }
  case GPU_FULL_LOOKUP_TABLE: {
#ifdef USTK_HAVE_CUDA
    this->GPUFullLookupTableConversion(dataPost, dataPre);
#else
    throw vpException(
        vpException::notImplementedError,
        "usPreScanToPostScan3DConverter::convert: using method GPU_FULL_LOOKUP_TABLE is not implemented yet");
#endif
    break;
  }
  case SINGLE_THREAD_REDUCED_LOOKUP_TABLE: {
    const unsigned int d = m_SweepInZdirection ? 0 : 1;
    unsigned int X = m_VpreScan.getWidth();
    unsigned int Y = m_VpreScan.getHeight();
    unsigned int XY = X * Y;
    for (int i = (int)m_reducedLookupTables[d].size() - 1; i >= 0; i--) {
      const usVoxelWeightAndIndexReducedMemory &m = m_reducedLookupTables[d][i];
      double u = m.m_W[0];
      double v = m.m_W[1];
      double w = m.m_W[2];
      double u1 = 1 - u;
      double v1 = 1 - v;
      double w1 = 1 - w;

      double v1w1 = v1 * w1;
      double vw1 = v * w1;
      double v1w = v1 * w;
      double vw = v * w;

      double W[8] = {u1 * v1w1, u * v1w1, u1 * vw1, u * vw1, u1 * v1w, u * v1w, u1 * vw, u * vw};

      unsigned int index[8] = {m.m_inputIndex,          m.m_inputIndex + 1,         m.m_inputIndex + X,
                               m.m_inputIndex + 1 + X,  m.m_inputIndex + XY,        m.m_inputIndex + 1 + XY,
                               m.m_inputIndex + X + XY, m.m_inputIndex + 1 + X + XY};

      double val = 0;
      for (int j = 0; j < 8; j++)
        val += W[j] * dataPre[index[j]];
      dataPost[m.m_outputIndex] = (unsigned char)val;
    }
    break;
  }
  case MULTI_THREAD_REDUCED_LOOKUP_TABLE: {
    const unsigned int d = m_SweepInZdirection ? 0 : 1;
    unsigned int X = m_VpreScan.getWidth();
    unsigned int Y = m_VpreScan.getHeight();
    unsigned int XY = X * Y;
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = (int)m_reducedLookupTables[d].size() - 1; i >= 0; i--) {
      const usVoxelWeightAndIndexReducedMemory &m = m_reducedLookupTables[d][i];
      double u = m.m_W[0];
      double v = m.m_W[1];
      double w = m.m_W[2];
      double u1 = 1 - u;
      double v1 = 1 - v;
      double w1 = 1 - w;

      double v1w1 = v1 * w1;
      double vw1 = v * w1;
      double v1w = v1 * w;
      double vw = v * w;

      double W[8] = {u1 * v1w1, u * v1w1, u1 * vw1, u * vw1, u1 * v1w, u * v1w, u1 * vw, u * vw};

      unsigned int index[8] = {m.m_inputIndex,          m.m_inputIndex + 1,         m.m_inputIndex + X,
                               m.m_inputIndex + 1 + X,  m.m_inputIndex + XY,        m.m_inputIndex + 1 + XY,
                               m.m_inputIndex + X + XY, m.m_inputIndex + 1 + X + XY};

      double val = 0;
      for (int j = 0; j < 8; j++)
        val += W[j] * dataPre[index[j]];
      dataPost[m.m_outputIndex] = (unsigned char)val;
    }
    break;
  }
  case GPU_REDUCED_LOOKUP_TABLE: {
#ifdef USTK_HAVE_CUDA
    this->GPUReducedLookupTableConversion(dataPost, dataPre);
#else
    throw vpException(
        vpException::notImplementedError,
        "usPreScanToPostScan3DConverter::convert: using method GPU_REDUCED_LOOKUP_TABLE is not implemented yet");
#endif
    break;
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
 * Choose the method used for the optimization of the conversion.
 * @param method optimization method.
 */
void usPreScanToPostScan3DConverter::setConverterOptimizationMethod(usConverterOptimizationMethod method)
{
  m_converterOptimizationMethod = method;

  switch (m_converterOptimizationMethod) {
  case SINGLE_THREAD_DIRECT_CONVERSION:
  case SINGLE_THREAD_FULL_LOOKUP_TABLE:
  case SINGLE_THREAD_REDUCED_LOOKUP_TABLE: {
    break;
  }
  case MULTI_THREAD_DIRECT_CONVERSION:
  case MULTI_THREAD_FULL_LOOKUP_TABLE:
  case MULTI_THREAD_REDUCED_LOOKUP_TABLE: {
#ifndef VISP_HAVE_OPENMP
    std::cout << "Warning in usPreScanToPostScan3DConverter::setConverterOptimizationMethod: OpenMP is not available "
                 "to use multi-thread optimization, will use single thread implementation instead."
              << std::endl;
#endif
    break;
  }
  case GPU_DIRECT_CONVERSION:
  case GPU_FULL_LOOKUP_TABLE:
  case GPU_REDUCED_LOOKUP_TABLE: {
#ifndef USTK_HAVE_CUDA
    throw vpException(vpException::notImplementedError, "usPreScanToPostScan3DConverter::"
                                                        "setConverterOptimizationMethod: cannot use "
                                                        "GPU conversion without CUDA");
#endif
    break;
  }
  }
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
  // const double theta = (sweepInZdirection ? 1 : -1) * (m_VpreScan.getFramePitch() * Nframe * (j_preScan + Nline *
  // k_preScan) / (Nframe * Nline - 1) - offsetTheta);
  const double theta =
      (m_VpreScan.getFramePitch() * Nframe *
           ((sweepInZdirection ? j_preScan : Nline - 1 - j_preScan) + Nline * k_preScan) / (Nframe * Nline - 1) -
       offsetTheta);

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
  // if (k_preScan) {
  // *k_preScan =
  //      (Nframe * Nline - 1) *
  //          (0.5 / Nline + (sweepInZdirection ? 1 : -1) * theta / (m_VpreScan.getFramePitch() * Nframe * Nline)) -
  //      jtmp / Nline;
  //}
  if (k_preScan) {
    *k_preScan = (Nframe * Nline - 1) * (0.5 / Nline + theta / (m_VpreScan.getFramePitch() * Nframe * Nline)) -
                 (sweepInZdirection ? jtmp : Nline - 1 - jtmp) / Nline;
  }
}
