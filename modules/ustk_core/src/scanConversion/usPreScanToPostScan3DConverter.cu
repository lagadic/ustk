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
#include <visp3/ustk_core/usPreScanToPostScan3DConverter.h>
    
struct cudaVoxelWeightAndIndex
{
unsigned int m_outputIndex;
unsigned int m_inputIndex[8];
double m_W[8];
};

struct cudaVoxelWeightAndIndexReducedMemory
{
  unsigned int m_outputIndex;
  unsigned int m_inputIndex;
  double m_W[3];
};

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

__global__ void kernelPostScanVoxelFillFullLookUpTable(cudaVoxelWeightAndIndex* lookupTable1, cudaVoxelWeightAndIndex* lookupTable2, unsigned int m_nbX, unsigned int m_nbY, unsigned int m_nbZ, int X, int Y, int Z, double m_resolution, double xmax, double ymin, double zmax, unsigned int frameNumber, unsigned int scanLineNumber, double transducerRadius, double motorRadius, double scanLinePitch, double axialResolution, double framePitch)
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

    double i, j, k1, k2;

    double radiusOffset = transducerRadius - motorRadius;
    double rProbe = radiusOffset + sqrt(yy * yy + zz * zz);
    double r = sqrt(rProbe * rProbe + xx * xx);
    double phi = atan(xx / rProbe);
    double theta = atan(zz / yy);

    double itmp = phi / scanLinePitch + 0.5 * (scanLineNumber - 1);
    i = itmp;
    j = (r - transducerRadius) / axialResolution;

    k1 = (frameNumber * scanLineNumber - 1) * (0.5 / scanLineNumber + theta / (framePitch * frameNumber * scanLineNumber)) - (itmp) / scanLineNumber;
  
    k2 = (frameNumber * scanLineNumber - 1) * (0.5 / scanLineNumber + theta / (framePitch * frameNumber * scanLineNumber)) - (scanLineNumber-1-itmp) / scanLineNumber;
  
    double ii = floor(i);
    double jj = floor(j);
    double kk1 = floor(k1);
    double kk2 = floor(k2);
  
    if (ii >= 0 && jj >= 0 && kk1 >= 0 && ii + 1 < X && jj + 1 < Y && kk1 + 1 < Z) {
        cudaVoxelWeightAndIndex m;

        m.m_outputIndex = x + m_nbX * y + nbXY * z;

        double u = i - ii;
        double v = j - jj;
        double w = k1 - kk1;
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
        double XYKK = XY * kk1;
        double XYKK1 = XY * (kk1 + 1);

        m.m_inputIndex[0] = (unsigned int)(ii + Xjj + XYKK);
        m.m_inputIndex[1] = (unsigned int)(ii + 1 + Xjj + XYKK);
        m.m_inputIndex[2] = (unsigned int)(ii + Xjj1 + XYKK);
        m.m_inputIndex[3] = (unsigned int)(ii + 1 + Xjj1 + XYKK);
        m.m_inputIndex[4] = (unsigned int)(ii + Xjj + XYKK1);
        m.m_inputIndex[5] = (unsigned int)(ii + 1 + Xjj + XYKK1);
        m.m_inputIndex[6] = (unsigned int)(ii + Xjj1 + XYKK1);
        m.m_inputIndex[7] = (unsigned int)(ii + 1 + Xjj1 + XYKK1);

        lookupTable1[m.m_outputIndex] = m;
    }
    if (ii >= 0 && jj >= 0 && kk2 >= 0 && ii + 1 < X && jj + 1 < Y && kk2 + 1 < Z) {
        cudaVoxelWeightAndIndex m;

        m.m_outputIndex = x + m_nbX * y + nbXY * z;

        double u = i - ii;
        double v = j - jj;
        double w = k2 - kk2;
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
        double XYKK = XY * kk2;
        double XYKK1 = XY * (kk2 + 1);

        m.m_inputIndex[0] = (unsigned int)(ii + Xjj + XYKK);
        m.m_inputIndex[1] = (unsigned int)(ii + 1 + Xjj + XYKK);
        m.m_inputIndex[2] = (unsigned int)(ii + Xjj1 + XYKK);
        m.m_inputIndex[3] = (unsigned int)(ii + 1 + Xjj1 + XYKK);
        m.m_inputIndex[4] = (unsigned int)(ii + Xjj + XYKK1);
        m.m_inputIndex[5] = (unsigned int)(ii + 1 + Xjj + XYKK1);
        m.m_inputIndex[6] = (unsigned int)(ii + Xjj1 + XYKK1);
        m.m_inputIndex[7] = (unsigned int)(ii + 1 + Xjj1 + XYKK1);

        lookupTable2[m.m_outputIndex] = m;
    }
}

__global__ void kernelPostScanVoxelFullLookUpTable(unsigned char *dataPost, const unsigned char *dataPre, const cudaVoxelWeightAndIndex *lookupTable, long int size)
{
    uint index = (blockIdx.x * blockDim.x) + threadIdx.x;

    if(index >= size) return;
    
    double v = 0;
    for (int j = 0; j < 8; j++)
      v += lookupTable[index].m_W[j] * dataPre[lookupTable[index].m_inputIndex[j]];
    dataPost[lookupTable[index].m_outputIndex] = (unsigned char)v;
}

__global__ void kernelPostScanVoxelFillReducedLookUpTable(cudaVoxelWeightAndIndexReducedMemory* lookupTable1, cudaVoxelWeightAndIndexReducedMemory* lookupTable2, unsigned int m_nbX, unsigned int m_nbY, unsigned int m_nbZ, int X, int Y, int Z, double m_resolution, double xmax, double ymin, double zmax, unsigned int frameNumber, unsigned int scanLineNumber, double transducerRadius, double motorRadius, double scanLinePitch, double axialResolution, double framePitch)
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

    double i, j, k1, k2;

    double radiusOffset = transducerRadius - motorRadius;
    double rProbe = radiusOffset + sqrt(yy * yy + zz * zz);
    double r = sqrt(rProbe * rProbe + xx * xx);
    double phi = atan(xx / rProbe);
    double theta = atan(zz / yy);

    double itmp = phi / scanLinePitch + 0.5 * (scanLineNumber - 1);
    i = itmp;
    j = (r - transducerRadius) / axialResolution;

    k1 = (frameNumber * scanLineNumber - 1) * (0.5 / scanLineNumber + theta / (framePitch * frameNumber * scanLineNumber)) - (itmp) / scanLineNumber;
  
    k2 = (frameNumber * scanLineNumber - 1) * (0.5 / scanLineNumber + theta / (framePitch * frameNumber * scanLineNumber)) - (scanLineNumber-1-itmp) / scanLineNumber;
  
    double ii = floor(i);
    double jj = floor(j);
    double kk1 = floor(k1);
    double kk2 = floor(k2);
  
    if (ii >= 0 && jj >= 0 && kk1 >= 0 && ii + 1 < X && jj + 1 < Y && kk1 + 1 < Z) {
        cudaVoxelWeightAndIndexReducedMemory m;

        m.m_outputIndex = x + m_nbX * y + nbXY * z;

        m.m_W[0] = i - ii;
        m.m_W[1] = j - jj;
        m.m_W[2] = k1 - kk1;
        
        m.m_inputIndex = (unsigned int)(ii + X * jj + XY * kk1);

        lookupTable1[m.m_outputIndex] = m;
    }
    if (ii >= 0 && jj >= 0 && kk2 >= 0 && ii + 1 < X && jj + 1 < Y && kk2 + 1 < Z) {
        cudaVoxelWeightAndIndexReducedMemory m;

        m.m_outputIndex = x + m_nbX * y + nbXY * z;

        m.m_W[0] = i - ii;
        m.m_W[1] = j - jj;
        m.m_W[2] = k2 - kk2;

        m.m_inputIndex = (unsigned int)(ii + X * jj + XY * kk2);
        
        lookupTable2[m.m_outputIndex] = m;
    }
}

__global__ void kernelPostScanVoxelReducedLookUpTable(unsigned char *dataPost, const unsigned char *dataPre, const cudaVoxelWeightAndIndexReducedMemory *lookupTable, long int size, int X, int Y)
{
    uint index = (blockIdx.x * blockDim.x) + threadIdx.x;

    if(index >= size) return;

    const cudaVoxelWeightAndIndexReducedMemory &m = lookupTable[index];
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

    unsigned int XY = X*Y;
    unsigned int inputIndex[8] = {m.m_inputIndex,          m.m_inputIndex + 1,         m.m_inputIndex + X,
                               m.m_inputIndex + 1 + X,  m.m_inputIndex + XY,        m.m_inputIndex + 1 + XY,
                               m.m_inputIndex + X + XY, m.m_inputIndex + 1 + X + XY};

    double val = 0;
    for (int j = 0; j < 8; j++)
      val += W[j] * dataPre[inputIndex[j]];
    dataPost[m.m_outputIndex] = (unsigned char)val;    
}

void usPreScanToPostScan3DConverter::GPUDirectConversion(unsigned char *dataPost, const unsigned char *dataPre)
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
    
    unsigned char *dataPostDevice;
    unsigned int sizePost = m_nbX*m_nbY*m_nbZ*sizeof(unsigned char);
    unsigned char *dataPreDevice;
    unsigned int sizePre = X*Y*Z*sizeof(unsigned char);

	cudaError_t codePost = cudaMalloc((void**)&dataPostDevice, sizePost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversion: GPU post-scan memory allocation error (%d Bytes): %s", sizePost, cudaGetErrorString(codePost));
    cudaError_t codePre = cudaMalloc((void**)&dataPreDevice, sizePre);
    if(codePre != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversion: GPU pre-scan memory allocation error (%d Bytes): %s", sizePre, cudaGetErrorString(codePre));

    codePost = cudaMemset(dataPostDevice, 0, sizePost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversionWrapper: GPU post-scan memory set error: %s", cudaGetErrorString(codePost));
    codePre = cudaMemcpy(dataPreDevice, dataPre, sizePre, cudaMemcpyHostToDevice);
    if(codePre != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversionWrapper: GPU pre-scan memory copy error: %s", cudaGetErrorString(codePre));

    dim3 threadsPerBlock(8, 8, 8);
    dim3 numBlocks((m_nbX+threadsPerBlock.x-1)/threadsPerBlock.x, (m_nbY+threadsPerBlock.y-1)/threadsPerBlock.y, (m_nbZ+threadsPerBlock.z-1)/threadsPerBlock.z);
    kernelPostScanVoxelDirect<<<numBlocks,threadsPerBlock>>>(dataPostDevice, dataPreDevice, m_nbX, m_nbY, m_nbZ, X, Y, Z, m_resolution, xmax, ymin, zmax, m_VpreScan.getFrameNumber(), m_VpreScan.getScanLineNumber(), m_VpreScan.getTransducerRadius(), m_VpreScan.getMotorRadius(), m_VpreScan.getScanLinePitch(), m_VpreScan.getAxialResolution(), m_VpreScan.getFramePitch(), m_SweepInZdirection);

    codePost = cudaMemcpy(dataPost, dataPostDevice, sizePost, cudaMemcpyDeviceToHost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUDirectConversionWrapper: GPU post-scan memory copy error: %s", cudaGetErrorString(codePost));
    cudaFree(dataPostDevice);
    cudaFree(dataPreDevice);
}

void usPreScanToPostScan3DConverter::GPUFreeLookupTables()
{
    if(m_GPULookupTables[0] != NULL) cudaFree(m_GPULookupTables[0]);
    m_GPULookupTablesSize[0] = 0;
    if(m_GPULookupTables[1] != NULL) cudaFree(m_GPULookupTables[1]);
    m_GPULookupTablesSize[1] = 0;
}

void usPreScanToPostScan3DConverter::GPUAllocateFullLookupTables()
{
    if(m_GPULookupTables[0] != NULL || m_GPULookupTables[1] != NULL) this->GPUFreeLookupTables();

    m_GPULookupTablesSize[0] = (long int)m_nbX * (long int)m_nbY * (long int)m_nbZ;
    m_GPULookupTablesSize[1] = m_GPULookupTablesSize[0];
    long int LUTmaxSize = m_GPULookupTablesSize[0] * sizeof(cudaVoxelWeightAndIndex);

    cudaError_t code = cudaMalloc((void**)&(m_GPULookupTables[0]), LUTmaxSize);
    if(code != cudaSuccess) 
    {
        std::cout << "Warning: usPreScanToPostScan3DConverter::GPUAllocateFullLookupTables: GPU memory allocation error for table 1 (" << LUTmaxSize << " Bytes): " << cudaGetErrorString(code) << "\nwill try on RAM, performances can be impacted" << std::endl;
        code = cudaMallocManaged((void**)&(m_GPULookupTables[0]), LUTmaxSize);
        if(code != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUAllocateFullLookupTables: memory allocation error for table 1 (%d Bytes): %s", LUTmaxSize, cudaGetErrorString(code));
    }

    code = cudaMalloc((void**)&(m_GPULookupTables[1]), LUTmaxSize);
    if(code != cudaSuccess)
    {
                std::cout << "Warning: usPreScanToPostScan3DConverter::GPUAllocateFullLookupTables: GPU memory allocation error for table 2 (" << LUTmaxSize << " Bytes): " << cudaGetErrorString(code) << "\nwill try on RAM, performances can be impacted" << std::endl;
        code = cudaMallocManaged((void**)&(m_GPULookupTables[1]), LUTmaxSize);
        if(code != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUAllocateFullLookupTables: memory allocation error for table 2 (%d Bytes): %s", LUTmaxSize, cudaGetErrorString(code));
    }

    code = cudaMemset(m_GPULookupTables[0], 0, LUTmaxSize);
    if(code != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUAllocateFullLookupTables: GPU memory set error for table 1: %s", cudaGetErrorString(code));
    code = cudaMemset(m_GPULookupTables[1], 0, LUTmaxSize);
    if(code != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUAllocateFullLookupTables: GPU memory set error for table 2: %s", cudaGetErrorString(code));
}

void usPreScanToPostScan3DConverter::GPUFillFullLookupTables()
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
    dim3 numBlocks((m_nbX+threadsPerBlock.x-1)/threadsPerBlock.x, (m_nbY+threadsPerBlock.y-1)/threadsPerBlock.y, (m_nbZ+threadsPerBlock.z-1)/threadsPerBlock.z);
    kernelPostScanVoxelFillFullLookUpTable<<<numBlocks,threadsPerBlock>>>((cudaVoxelWeightAndIndex*)m_GPULookupTables[0], (cudaVoxelWeightAndIndex*)m_GPULookupTables[1], m_nbX, m_nbY, m_nbZ, X, Y, Z, m_resolution, xmax, ymin, zmax, m_VpreScan.getFrameNumber(), m_VpreScan.getScanLineNumber(), m_VpreScan.getTransducerRadius(), m_VpreScan.getMotorRadius(), m_VpreScan.getScanLinePitch(), m_VpreScan.getAxialResolution(), m_VpreScan.getFramePitch());

    cudaError_t code = cudaDeviceSynchronize();
    if(code != cudaSuccess) throw vpException(vpException::fatalError, "usPreScanToPostScan3DConverter::GPUFillFullLookupTables: %s", cudaGetErrorString(code));
}

void usPreScanToPostScan3DConverter::GPUFullLookupTableConversion(unsigned char *dataPost, const unsigned char *dataPre)
{   
    int X = m_VpreScan.getWidth();
    int Y = m_VpreScan.getHeight();
    int Z = m_VpreScan.getNumberOfFrames();
    
    unsigned char *dataPostDevice;
    unsigned int sizePost = m_nbX*m_nbY*m_nbZ*sizeof(unsigned char);
    unsigned char *dataPreDevice;
    unsigned int sizePre = X*Y*Z*sizeof(unsigned char);

	cudaError_t codePost = cudaMalloc((void**)&dataPostDevice, sizePost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUFullLookupTableConversion: GPU post-scan memory allocation error (%d Bytes)", sizePost);
    cudaError_t codePre = cudaMalloc((void**)&dataPreDevice, sizePre);
    if(codePre != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUFullLookupTableConversion: GPU pre-scan memory allocation error (%d Bytes)", sizePre);

    codePost = cudaMemset(dataPostDevice, 0, sizePost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUFullLookupTableConversion: GPU post-scan memory set error");
    codePre = cudaMemcpy(dataPreDevice, dataPre, sizePre, cudaMemcpyHostToDevice);
    if(codePre != cudaSuccess) {std::cout << codePre << std::endl;
throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUFullLookupTableConversion: GPU pre-scan memory copy error");
}
    unsigned int sweepIndex = m_SweepInZdirection?0:1;
    dim3 threadsPerBlock(512);
    dim3 numBlocks((m_GPULookupTablesSize[sweepIndex]+threadsPerBlock.x-1)/threadsPerBlock.x);
    kernelPostScanVoxelFullLookUpTable<<<numBlocks,threadsPerBlock>>>(dataPostDevice, dataPreDevice, (const cudaVoxelWeightAndIndex*)(m_GPULookupTables[sweepIndex]), m_GPULookupTablesSize[sweepIndex]);

    cudaError_t codeExec = cudaDeviceSynchronize();
    if(codeExec != cudaSuccess) throw vpException(vpException::fatalError, "usPreScanToPostScan3DConverter::GPUFullLookupTableConversion: %s", cudaGetErrorString(codeExec));

    codePost = cudaMemcpy(dataPost, dataPostDevice, sizePost, cudaMemcpyDeviceToHost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUFullLookupTableConversion: GPU post-scan memory copy error");
    cudaFree(dataPostDevice);
    cudaFree(dataPreDevice);
}

void usPreScanToPostScan3DConverter::GPUAllocateReducedLookupTables()
{
    if(m_GPULookupTables[0] != NULL || m_GPULookupTables[1] != NULL) this->GPUFreeLookupTables();

    m_GPULookupTablesSize[0] = (long int)m_nbX * (long int)m_nbY * (long int)m_nbZ;
    m_GPULookupTablesSize[1] = m_GPULookupTablesSize[0];
    long int LUTmaxSize = m_GPULookupTablesSize[0] * sizeof(cudaVoxelWeightAndIndexReducedMemory);

    cudaError_t code = cudaMalloc((void**)&(m_GPULookupTables[0]), LUTmaxSize);
    if(code != cudaSuccess) 
    {
        std::cout << "Warning: usPreScanToPostScan3DConverter::GPUAllocateReducedLookupTables: GPU memory allocation error for table 1 (" << LUTmaxSize << " Bytes): " << cudaGetErrorString(code) << "\nwill try on RAM, performances can be impacted" << std::endl;
        code = cudaMallocManaged((void**)&(m_GPULookupTables[0]), LUTmaxSize);
        if(code != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUAllocateReducedLookupTables: memory allocation error for table 1 (%d Bytes): %s", LUTmaxSize, cudaGetErrorString(code));
    }

    code = cudaMalloc((void**)&(m_GPULookupTables[1]), LUTmaxSize);
    if(code != cudaSuccess)
    {
                std::cout << "Warning: usPreScanToPostScan3DConverter::GPUAllocateReducedLookupTables: GPU memory allocation error for table 2 (" << LUTmaxSize << " Bytes): " << cudaGetErrorString(code) << "\nwill try on RAM, performances can be impacted" << std::endl;
        code = cudaMallocManaged((void**)&(m_GPULookupTables[1]), LUTmaxSize);
        if(code != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUAllocateReducedLookupTables: memory allocation error for table 2 (%d Bytes): %s", LUTmaxSize, cudaGetErrorString(code));
    }

    code = cudaMemset(m_GPULookupTables[0], 0, LUTmaxSize);
    if(code != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUAllocateReducedLookupTables: GPU memory set error for table 1: %s", cudaGetErrorString(code));
    code = cudaMemset(m_GPULookupTables[1], 0, LUTmaxSize);
    if(code != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUAllocateReducedLookupTables: GPU memory set error for table 2: %s", cudaGetErrorString(code));
}

void usPreScanToPostScan3DConverter::GPUFillReducedLookupTables()
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
    dim3 numBlocks((m_nbX+threadsPerBlock.x-1)/threadsPerBlock.x, (m_nbY+threadsPerBlock.y-1)/threadsPerBlock.y, (m_nbZ+threadsPerBlock.z-1)/threadsPerBlock.z);
    kernelPostScanVoxelFillReducedLookUpTable<<<numBlocks,threadsPerBlock>>>((cudaVoxelWeightAndIndexReducedMemory*)m_GPULookupTables[0], (cudaVoxelWeightAndIndexReducedMemory*)m_GPULookupTables[1], m_nbX, m_nbY, m_nbZ, X, Y, Z, m_resolution, xmax, ymin, zmax, m_VpreScan.getFrameNumber(), m_VpreScan.getScanLineNumber(), m_VpreScan.getTransducerRadius(), m_VpreScan.getMotorRadius(), m_VpreScan.getScanLinePitch(), m_VpreScan.getAxialResolution(), m_VpreScan.getFramePitch());

    cudaError_t code = cudaDeviceSynchronize();
    if(code != cudaSuccess) throw vpException(vpException::fatalError, "usPreScanToPostScan3DConverter::GPUFillReducedLookupTables: %s", cudaGetErrorString(code));
}

void usPreScanToPostScan3DConverter::GPUReducedLookupTableConversion(unsigned char *dataPost, const unsigned char *dataPre)
{   
    int X = m_VpreScan.getWidth();
    int Y = m_VpreScan.getHeight();
    int Z = m_VpreScan.getNumberOfFrames();
    
    unsigned char *dataPostDevice;
    unsigned int sizePost = m_nbX*m_nbY*m_nbZ*sizeof(unsigned char);
    unsigned char *dataPreDevice;
    unsigned int sizePre = X*Y*Z*sizeof(unsigned char);

	cudaError_t codePost = cudaMalloc((void**)&dataPostDevice, sizePost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUReducedLookupTableConversion: GPU post-scan memory allocation error (%d Bytes)", sizePost);
    cudaError_t codePre = cudaMalloc((void**)&dataPreDevice, sizePre);
    if(codePre != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUReducedLookupTableConversion: GPU pre-scan memory allocation error (%d Bytes)", sizePre);

    codePost = cudaMemset(dataPostDevice, 0, sizePost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUReducedLookupTableConversion: GPU post-scan memory set error");
    codePre = cudaMemcpy(dataPreDevice, dataPre, sizePre, cudaMemcpyHostToDevice);
    if(codePre != cudaSuccess) {std::cout << codePre << std::endl;
throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUReducedLookupTableConversion: GPU pre-scan memory copy error");
}
    unsigned int sweepIndex = m_SweepInZdirection?0:1;
    dim3 threadsPerBlock(512);
    dim3 numBlocks((m_GPULookupTablesSize[sweepIndex]+threadsPerBlock.x-1)/threadsPerBlock.x);
    kernelPostScanVoxelReducedLookUpTable<<<numBlocks,threadsPerBlock>>>(dataPostDevice, dataPreDevice, (const cudaVoxelWeightAndIndexReducedMemory*)(m_GPULookupTables[sweepIndex]), m_GPULookupTablesSize[sweepIndex], X, Y);

    cudaError_t codeExec = cudaDeviceSynchronize();
    if(codeExec != cudaSuccess) throw vpException(vpException::fatalError, "usPreScanToPostScan3DConverter::GPUReducedLookupTableConversion: %s", cudaGetErrorString(codeExec));

    codePost = cudaMemcpy(dataPost, dataPostDevice, sizePost, cudaMemcpyDeviceToHost);
    if(codePost != cudaSuccess) throw vpException(vpException::memoryAllocationError, "usPreScanToPostScan3DConverter::GPUReducedLookupTableConversion: GPU post-scan memory copy error");
    cudaFree(dataPostDevice);
    cudaFree(dataPreDevice);
}
