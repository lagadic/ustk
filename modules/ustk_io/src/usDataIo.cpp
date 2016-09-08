/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Authors:
 * Pierre Chatelain
 *
 *****************************************************************************/

#include <visp3/ustk_io/usDataIo.h>
#include <iostream>
#include <fstream>
#include <set>
#include <sstream>

#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>

#include <visp3/ustk_io/usVolumeIo.h>

void usDataIo::read(usData &data, const char *filename) {
  if (data.getMode() == PRESCAN_2D)
    read(dynamic_cast<usDataPrescan2D&>(data), filename);
  if (data.getMode() == PRESCAN_3D)
    read(dynamic_cast<usDataPrescan3D&>(data), filename);
  if (data.getMode() == POSTSCAN_2D)
    read(dynamic_cast<usDataPostscan2D&>(data), filename);
  if (data.getMode() == POSTSCAN_3D)
    read(dynamic_cast<usDataPostscan3D&>(data), filename);
  if (data.getMode() == RF_3D)
    read(dynamic_cast<usDataRF3D&>(data), filename);
}

void usDataIo::read(usDataPostscan2D &data, const char *filename) {
  vpImageIo::read(data, filename);
}

void usDataIo::read(usDataPostscan3D &data, const char *filename) {
  usVolumeIo::readMHD(data, filename);
}

void usDataIo::read(usDataPrescan2D &data, const char *filename) {
  vpImageIo::read(data, filename);
}

void usDataIo::read(usDataPrescan3D &data, const char *filename) {
  usVolume<unsigned char> V;
  usVolumeIo::readMHD(V, filename);
  data.resize(V.getDimX(), V.getDimY(), V.getDimZ());
  data.setData(V.getData());
}

void usDataIo::read(usDataRF3D &data, const char *filename) {
  usVolumeIo::readMHD(data, filename);
}

FILE *usDataIo::readPropelloHeader(propelloHeader &header, const char *filename)
{
  // Open file
  FILE *pFile = fopen(filename, "rb");
  if (pFile == NULL) {
    return NULL;
  }
  
  // Read header
  if (fread(&header, sizeof(header), 1, pFile) != 1) {
    std::cerr << "Error in readPropelloHeader: failed to read file header." << std::endl;
    fclose(pFile);
    return NULL;
  }
  
  return pFile;
}

FILE *usDataIo::readUlteriusHeader(uFileHeader &header, const char *filename)
{
  // Open file
  FILE *pFile = fopen(filename, "rb");
  if (pFile == NULL) {
    return NULL;
  }
  
  // Read header
  if (fread(&header, sizeof(header), 1, pFile) != 1) {
    std::cerr << "Error in readUlteriusHeader: failed to read file header." << std::endl;
    fclose(pFile);
    return NULL;
  }
  
  return pFile;
}

void usDataIo::write(const usData &data, const char *filename) {
  if (data.getMode() == PRESCAN_2D)
    write(dynamic_cast<const usDataPrescan2D&>(data), filename);
  if (data.getMode() == PRESCAN_3D)
    write(dynamic_cast<const usDataPrescan3D&>(data), filename);
  if (data.getMode() == POSTSCAN_2D)
    write(dynamic_cast<const usDataPostscan2D&>(data), filename);
  if (data.getMode() == POSTSCAN_3D)
    write(dynamic_cast<const usDataPostscan3D&>(data), filename);
  if (data.getMode() == RF_3D)
    write(dynamic_cast<const usDataRF3D&>(data), filename);
}

void usDataIo::write(const usDataPostscan2D &data, const char *filename) {
  vpImageIo::write(data, filename);
}

void usDataIo::write(const usDataPostscan3D &data, const char *filename) {
  usVolumeIo::writeMHD(data, filename);
}

void usDataIo::write(const usDataPrescan2D &data, const char *filename) {
  std::string ext = vpIoTools::getFileExtension(filename);

  // ViSP takes care of pgm, ppm, jpg, jpeg, png, jp2, rs, ras, tiff, tif, bmp and pbm
  std::set<std::string> vispExt;
  vispExt.insert(".pgm");
  vispExt.insert(".ppm");
  vispExt.insert(".jpg");
  vispExt.insert("jpeg");
  vispExt.insert(".png");
  vispExt.insert(".jp2");
  vispExt.insert(".rs");
  vispExt.insert(".ras");
  vispExt.insert(".tiff");
  vispExt.insert(".tif");
  vispExt.insert(".bmp");
  vispExt.insert(".pbm");
  if (vispExt.find(ext) != vispExt.end()) {
    vpImageIo::write(data, filename);
    return;
  }
  
  // MHD format
  if (ext.compare(".mhd") == 0) {
    writeMHD(data, filename);
    return;
  }

  // Format not supported
  std::cerr << "Error: In usDataIo::write(): "
	    << "Could not find a writer for extension " << ext << std::endl;
  exit(EXIT_FAILURE);
  
}

void usDataIo::write(const usDataPrescan3D &data, const char *filename) {
  usVolume<unsigned char> V;
  V.resize(data.getAN(), data.getLN(), data.getFN());
  V.setData(V.getData());
  usVolumeIo::writeMHD(V, filename);
}

void usDataIo::write(const usDataRF3D &data, const char *filename) {
  usVolumeIo::writeMHD(data, filename);
}

void usDataIo::writeMHD(const usDataPrescan2D &data, const char *filename)
{
  // Create filenames
  std::string pathname = vpIoTools::getParent(filename);
  std::string basename = vpIoTools::getNameWE(filename);
  std::string rawname = basename + ".raw";
  std::string mhdname = pathname + "/" + basename + ".mhd";

  // Get image properties
  unsigned int n = data.getSize();
  std::string objectType = "Image";
  int nDims = 2;
  int id = data.getDataIdx();
  std::string binaryData = "True";
  std::string elementByteOrderMSB = "False";
  std::string binaryDataByteOrderMSB = "False";
  float spacingX = data.getResolution();
  float spacingY = data.getLineAngle();
  int dimX = data.getAN();
  int dimY = data.getLN();
  int headerSize = 0;
  std::string modality = "MET_MOD_US";
  int elementNumberOfChannels = 1;
  float elementSizeX = data.getResolution();
  float elementSizeY = data.getLineAngle();
  std::string elementType = "MET_UCHAR";
  std::string elementDataFile = rawname;

  // US-specific fields
  std::string dataMode;
  switch(data.getMode()) {
  case PRESCAN_2D: dataMode = "PRESCAN_2D"; break;
  case PRESCAN_3D: dataMode = "PRESCAN_3D"; break;
  case POSTSCAN_2D: dataMode = "POSTSCAN_2D"; break;
  case POSTSCAN_3D: dataMode = "POSTSCAN_3D"; break;
  case RF_2D: dataMode = "RF_2D"; break;
  case RF_3D: dataMode = "RF_3D"; break;
  default: dataMode = "UNKNOWN"; break;
  }

  std::string probeType;
  switch(data.getProbeType()) {
  case US_4DC7: probeType = "US_4DC7"; break;
  case SS_C60: probeType = "SS_C60"; break;
  default: probeType = "UNKNOWN"; break;
  }

  // Write MHD file
  std::ofstream mhdfile(mhdname.c_str());
  mhdfile << "ObjectType = " << objectType << std::endl;
  mhdfile << "NDims = " << nDims << std::endl;
  mhdfile << "ID = " << id << std::endl;
  mhdfile << "BinaryData = " << binaryData << std::endl;
  mhdfile << "ElementByteOrderMSB = " << elementByteOrderMSB << std::endl;
  mhdfile << "BinaryDataByteOrderMSB = " << binaryDataByteOrderMSB << std::endl;
  mhdfile << "ElementSpacing = " << spacingX << " " << spacingY << std::endl;
  mhdfile << "DimSize = " << dimX << " " << dimY << std::endl;
  mhdfile << "HeaderSize = " << headerSize << std::endl;
  mhdfile << "Modality = " << modality << std::endl;
  mhdfile << "ElementNumberOfChannels = " << elementNumberOfChannels << std::endl;
  mhdfile << "ElementSize = " << elementSizeX << " " << elementSizeY << std::endl;
  mhdfile << "ElementType = " << elementType << std::endl;
  mhdfile << "ElementDataFile = " << elementDataFile << std::endl;

  // US-specific fields
  mhdfile << "UsDataMode = " << dataMode << std::endl;
  mhdfile << "UsProbeType = " << probeType << std::endl;

  mhdfile.close();

  // Write RAW file
  std::ofstream rawfile((pathname + "/" + rawname).c_str(), std::ios::out | std::ios::binary);
  if(rawfile.is_open()) {
    rawfile.write(reinterpret_cast<const char*>(data.bitmap), sizeof(unsigned char) * n);
    rawfile.close();
  }
}
