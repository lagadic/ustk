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

#include <cuda.h>

#include <visp3/core/vpException.h>
    

__global__ void kernelPostScanVoxelDirect(unsigned char *dataPost, const unsigned char *dataPre, unsigned int m_nbX, unsigned int m_nbY, unsigned int m_nbZ, int X, int Y, int Z, double m_resolution, double xmax, double ymin, double zmax, unsigned int frameNumber, unsigned int scanLineNumber, double transducerRadius, double motorRadius, double scanLinePitch, double axialResolution, double framePitch, bool sweepInZdirection)
{
    uint x = (blockIdx.x * blockDim.x) + threadIdx.x;
    uint y = (blockIdx.y * blockDim.y) + threadIdx.y;
    uint z = (blockIdx.z * blockDim.z) + threadIdx.z;
    
    if(x >= m_nbX  || y >= m_nbY || z >= m_nbZ) return;
  
    unsigned int nbXY = m_nbX * m_nbY;
    unsigned int XY = X * Y;

    double xx = m_resolution * x - xmax;
    double yy = ymin + m_resolution * y;
    double zz = m_resolution * z - zmax;

    double i, j, k;

    double radiusOffset = transducerRadius - motorRadius;
    double rProbe = radiusOffset + sqrt(yy * yy + zz * zz);
    double r = sqrt(rProbe * rProbe + xx * xx);
    double phi = atan(xx / rProbe);
    double theta = atan(zz / yy);

    double itmp = phi / scanLinePitch + 0.5 * (scanLineNumber - 1);
    i = itmp;
    j = (r - transducerRadius) / axialResolution;
    k = (frameNumber * scanLineNumber - 1) * (0.5 / scanLineNumber + theta / (framePitch * frameNumber * scanLineNumber)) - (sweepInZdirection ? itmp : scanLineNumber-1-itmp) / scanLineNumber;
  
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
        for (int n = 0; n < 8; n++) val += W[n] * dataPre[index[n]];
        dataPost[x + m_nbX * y + nbXY * z] = (unsigned char)val;
    }
}

void GPUDirectConversionWrapper(unsigned char *dataPost, const unsigned char *dataPre, unsigned int m_nbX, unsigned int m_nbY, unsigned int m_nbZ, int X, int Y, int Z, double m_resolution, double xmax, double ymin, double zmax, unsigned int frameNumber, unsigned int scanLineNumber, double transducerRadius, double motorRadius, double scanLinePitch, double axialResolution, double framePitch, bool sweepInZdirection)
{   
    unsigned char *dataPostDevice;
    unsigned int sizePost = m_nbX*m_nbY*m_nbZ*sizeof(unsigned char);
    unsigned char *dataPreDevice;
    unsigned int sizePre = X*Y*Z*sizeof(unsigned char);

	cudaError_t codePost = cudaMalloc((void**)&dataPostDevice, sizePost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversionWrapper: GPU memory allocation error (%d Bytes)", sizePost);
    cudaError_t codePre = cudaMalloc((void**)&dataPreDevice, sizePre);
    if(codePre != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversionWrapper: GPU memory allocation error (%d Bytes)", sizePost);

    codePost = cudaMemset(dataPostDevice, 0, sizePost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversionWrapper: GPU memory set error");
    codePre = cudaMemcpy(dataPreDevice, dataPre, sizePre, cudaMemcpyHostToDevice);
    if(codePre != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversionWrapper: GPU memory copy error");

    unsigned int *count;
    count = new unsigned int;
    unsigned int *countDevice;
    cudaMalloc((void**)&countDevice, sizeof(unsigned int));
    cudaMemset(countDevice, 0, sizeof(unsigned int));

    dim3 threadsPerBlock(8, 8, 8);
    dim3 numBlocks((m_nbX+threadsPerBlock.x-1)/threadsPerBlock.x, (m_nbY+threadsPerBlock.y-1)/threadsPerBlock.y, (m_nbZ+threadsPerBlock.z-1)/threadsPerBlock.z);
std::cout << numBlocks.x << " " << numBlocks.y << " "  << numBlocks.z << std::endl;
    kernelPostScanVoxelDirect<<<numBlocks,threadsPerBlock>>>(dataPostDevice, dataPreDevice, m_nbX, m_nbY, m_nbZ, X, Y, Z, m_resolution, xmax, ymin, zmax, frameNumber, scanLineNumber, transducerRadius, motorRadius, scanLinePitch, axialResolution, framePitch, sweepInZdirection);

    codePost = cudaMemcpy(dataPost, dataPostDevice, sizePost, cudaMemcpyDeviceToHost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversionWrapper: GPU memory copy error");
    cudaFree(dataPostDevice);
    cudaFree(dataPreDevice);
}
