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
 *
 *****************************************************************************/

#include <visp3/ustk_core/usPreScanToPostScan3DConverter.h>

#ifdef USTK_HAVE_CUDA
void usPreScanToPostScan3DConverter::GPUDirectConversion()
{
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
    
    dim3 threadsPerBlock(8, 8, 8);
    dim3 numBlocks(m_nbX/threadsPerBlock.x, m_nbY/threadsPerBlock.y, m_nbZ/threadsPerBlock.z);
    usPreScanToPostScan3DConverter::kernelPostScanVoxelDirect<<<numBlocks,threadsPerBlock>>>(dataPost, dataPre, X, Y, Z, xmax, ymin, zmax);
}
    
__global__ void usPreScanToPostScan3DConverter::kernelPostScanVoxelDirect(unsigned char *dataPost, const unsigned char *dataPre, int X, int Y, int Z, double xmax_post, double ymin_post, double zmax_post)
{
    uint x = (blockIdx.x * blockDim.x) + threadIdx.x;
    uint y = (blockIdx.y * blockDim.y) + threadIdx.y;
    uint z = (blockIdx.z * blockDim.z) + threadIdx.z;
    
    if(x >= m_nbX  || y >= m_nbY || z >= m_nbZ) return;
  
    unsigned int nbXY = m_nbX * m_nbY;
    unsigned int XY = X * Y;

    double xx = m_resolution * x - xmax_post;
    double yy = ymin_post + m_resolution * y;
    double zz = m_resolution * z - zmax_post;

    double i, j, k;
    usPreScanToPostScan3DConverter::convertPostScanCoordToPreScanCoord(yy, xx, zz, &j, &i, &k, m_SweepInZdirection);
  
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
    
        double W[8] = { u1 * v1w1,
                        u * v1w1,
                        u1 * vw1,
                        u * vw1,
                        u1 * v1w,
                        u * v1w,
                        u1 * vw,
                        u * vw };
        
        double Xjj = X * jj;
        double Xjj1 = X * (jj + 1);
        double XYKK = XY * kk;
        double XYKK1 = XY * (kk + 1);
            
        unsigned int index[8] = { (unsigned int)(ii + Xjj + XYKK),
                                  (unsigned int)(ii + 1 + Xjj + XYKK),
                                  (unsigned int)(ii + Xjj1 + XYKK),
                                  (unsigned int)(ii + 1 + Xjj1 + XYKK),
                                  (unsigned int)(ii + Xjj + XYKK1),
                                  (unsigned int)(ii + 1 + Xjj + XYKK1),
                                  (unsigned int)(ii + Xjj1 + XYKK1),
                                  (unsigned int)(ii + 1 + Xjj1 + XYKK1)};
        
        double val = 0;
        for (int j = 0; j < 8; j++) val += W[j] * dataPre[index[j]];
        dataPost[x + m_nbX * y + nbXY * z] = (unsigned char)val;
    }
}

#endif
