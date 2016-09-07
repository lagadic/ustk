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

/**
 * @file usVolumeIo.h
 * @brief Volume handling.
 * @author Pierre Chatelain
 */

#ifndef US_VOLUME_IO_H
#define US_VOLUME_IO_H

#include <cstdlib>
#include <string>
#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>

#include "usVolume.h"
#include "usVolumeTools.h"

/**
 * @namespace usVolumeIo
 * @brief Input/Output operations for the usVolume class.
 *
 * This namespace contains input/output operations for the usVolume class.
 * The currently supported storage format is MetaImage (mhd).
 */
namespace usVolumeIo
{
  /**
   * Read a 3D MHD image.
   */
  template<class DataType>
    usVolume<DataType>* readMHD(usVolume<DataType>* vol, const char* filename );
  
  /**
   * Read a specific dimension in a 4D MHD image.
   */
  template<class DataType>
    usVolume<DataType>* readMHD(usVolume<DataType>* vol, const char* filename, unsigned int label );
  
  /**
   * Read a 3D MHD image.
   */
  template<class DataType>
    void readMHD(usVolume<DataType>& vol, const char* filename );
  
  /**
   * Read a specific dimension in a 4D MHD image.
   */
  template<class DataType>
    void readMHD(usVolume<DataType>& vol, const char* filename, unsigned int label );
  
  /**
   * Write a MHD image. Path has to end with a slash ( '/' or '\\' ).
   * Alternatively, the full path including the filename can be parsed through the first parameter.
   */
  template<class DataType>
    void writeMHD(const usVolume<DataType>& im, const char* filename, const char* path="" );
  
  /**
   * Splits the full path into path, file, and MHD suffix (if existing).
   */
  void splitFileNameForMHD(const std::string &filename, std::string &path,
			   std::string &file, std::string &suffix);
  
  namespace Private
  {
    
    struct MHDHeader
    {
      std::string mhdFileName;
      std::string rawFileName;
      unsigned int  numberDimensions;
      int  numberChannels;
      ElementType elementType;
      unsigned int dim[4];
      float elementSpacing[4];
      float position[4];
      int headerSize;
      bool msb;
    };
    
    /**
     * Help functions.
     */
    bool readMHDHeader(const char * fileName, MHDHeader& header);
    template<class DataType>
      void readMHDRawDataToBuffer(const MHDHeader& header, DataType* buffer);
    template<class DataType>
      void readMHDRawDataToBuffer(const MHDHeader& header, DataType* buffer, int label);
    template<class DataType>
      void convertToMHD(DataType * dst, void * src, int N, bool swapBytes);
    void splitPathPrefixAndFileName(const std::string& fullPath, std::string& pathPrefix, std::string& fileName);
    ElementType mhdStringToElementType(const std::string& mhdElementType);
    std::string elementTypeToMHDString(ElementType elementType);
  }
  
  //------------------------------------------------------------------------------------------------//
  template<class DataType>
    usVolume<DataType>* readMHD( usVolume<DataType>* vol, const char* filename ) {
    Private::MHDHeader header; 
    bool ret = Private::readMHDHeader(filename, header);
    if(ret) {
      if (header.dim[2] == 1) {
	std::cerr << "Error: volume has less than 3 dimensions." << std::endl;
	return 0;
      }
      vol = new usVolume<DataType>(header.dim[0], header.dim[1], header.dim[2],
				 header.elementSpacing[0], header.elementSpacing[1],
				 header.elementSpacing[2]);
      if (header.elementType != vol->getElementType()) {
	if (((header.elementType != MET_FLOAT) && (header.elementType != MET_DOUBLE))
	    || vol->getElementType() == MET_VECTOR) {
	  std::cerr << "Error: type mismatch - unable to cast "
		    << Private::elementTypeToMHDString(header.elementType) << " to "
		    << Private::elementTypeToMHDString(vol->getElementType()) << "." << std::endl;
	  delete [] vol;
	  return 0;
	}
	std::cerr << "Warning: type mismatch - casting "
		  << Private::elementTypeToMHDString(header.elementType) << " to "
		  << Private::elementTypeToMHDString(vol->getElementType()) << "." << std::endl;
	if (header.elementType == MET_FLOAT) {
	  usVolume<float>* tmp = new usVolume<float>(header.dim[0], header.dim[1], header.dim[2],
						 header.elementSpacing[0], header.elementSpacing[1],
						 header.elementSpacing[2]);
	  Private::readMHDRawDataToBuffer(header, tmp->getData());
	  usVolumeTools::castFromFloat(tmp, vol);
	  tmp->~usVolume<float>();
	  tmp = NULL;
	}
	else if (header.elementType == MET_DOUBLE) {
	  usVolume<double>* tmp = new usVolume<double>(header.dim[0], header.dim[1], header.dim[2],
						   header.elementSpacing[0], header.elementSpacing[1],
						   header.elementSpacing[2]);
	  Private::readMHDRawDataToBuffer(header, tmp->getData());
	  usVolumeTools::castFromDouble(tmp, vol);
	  tmp->~usVolume<double>();
	  tmp = NULL;
	}
      }
      else
	Private::readMHDRawDataToBuffer(header, vol->getData());
    }
    return vol;
  }

  //------------------------------------------------------------------------------------------------//
  template<class DataType>
    usVolume<DataType>* readMHD( usVolume<DataType>* vol, const char* filename, unsigned int label ) {
    Private::MHDHeader header; 
    bool ret = Private::readMHDHeader(filename, header);
    if(ret) {
      if(header.dim[2] == 1)
	return NULL;
      if(header.dim[3] < label+1) {
	std::cerr << "Error reading " << filename << ": max label is " << header.dim[3] - 1
		  << std::endl;
	return NULL;
      }
      vol = new usVolume<DataType>(header.dim[0], header.dim[1], header.dim[2],
				 header.elementSpacing[0], header.elementSpacing[1],
				 header.elementSpacing[2]);
      if (header.elementType != vol->getElementType()) {
	std::cerr << "Error reading " << filename << ": wrong type." << std::endl;
	delete [] vol;
	return 0;
      }
      Private::readMHDRawDataToBuffer(header, vol->getData(), label);
    }
    return vol;
  }

  //------------------------------------------------------------------------------------------------//
  template<class DataType>
    void readMHD( usVolume<DataType>& vol, const char* filename ) {
    Private::MHDHeader header; 
    bool ret = Private::readMHDHeader(filename, header);
    if(ret) {
      if (header.dim[2] == 1) {
	std::cerr << "Error: In usVolumeIO::readMHD() - data seems to be 2D."  << std::endl;
	exit(EXIT_FAILURE);
      }
      /*
	vol = usVolume<DataType>(header.dim[0], header.dim[1], header.dim[2],
	header.elementSpacing[0], header.elementSpacing[1],
	header.elementSpacing[2]);
      */
      vol.resize(header.dim[0], header.dim[1], header.dim[2]);
      vol.setElementSpacingX(header.elementSpacing[0]);
      vol.setElementSpacingY(header.elementSpacing[1]);
      vol.setElementSpacingZ(header.elementSpacing[2]);

      if (header.elementType != vol.getElementType()) {
	std::cerr << "Error while reading " << filename << " - unexpected element type." << std::endl;
	vol.~usVolume<DataType>();
	return;
      }
      Private::readMHDRawDataToBuffer(header, vol.getData());
    }
  }

  //------------------------------------------------------------------------------------------------//
  template<class DataType>
    void readMHD( usVolume<DataType>& vol, const char* filename, unsigned int label ) {
    Private::MHDHeader header; 
    bool ret = Private::readMHDHeader(filename, header);
    if(ret) {
      if(header.dim[2] == 1)
	return;
      if(header.dim[3] < label+1) {
	std::cerr << "Error reading " << filename << ": max label is " << header.dim[3] - 1
		  << std::endl;
      }
      vol = usVolume<DataType>(header.dim[0], header.dim[1], header.dim[2],
			     header.elementSpacing[0], header.elementSpacing[1],
			     header.elementSpacing[2]);
      if (header.elementType != vol.getElementType()) {
	vol.~usVolume<DataType>();
	return;
      }
      Private::readMHDRawDataToBuffer(header, vol.getData(), label);
    }
  }
  
  //------------------------------------------------------------------------------------------------//
  template<class DataType>
    void writeMHD(const usVolume<DataType>& im, const char* filename, const char* path )
    {
      // create filenames
      std::string rawname;
      std::string mhdname;
      std::string pathname;
      std::string basename;

      if(std::string(path) == "")
	{
	  std::string s;
	  splitFileNameForMHD(std::string(filename),pathname,basename,s);
	}
      else
	{
	  basename = std::string(filename);
	  pathname = std::string(path);
	}

      rawname  = basename + std::string(".raw");
      mhdname  = pathname + basename + std::string(".mhd");		

      // get image properties
      const DataType* data = im.getConstData();

      int n  = im.getSize();
      int dx = im.getDimX();
      int dy = im.getDimY();
      int dz = im.getDimZ();

      float esx = im.getElementSpacingX();
      float esy = im.getElementSpacingY();
      float esz = im.getElementSpacingZ();

      // write data
      std::ofstream rawfile( (pathname+rawname).c_str() , std::ios::out | std::ios::binary );
      if(rawfile.is_open()) {
	rawfile.write(reinterpret_cast<const char*>(data), sizeof(DataType) * n );
	rawfile.close();
      }
      else {
	throw std::ios_base::failure("Could not open file.");
      }

      // write header
      std::ofstream mhdfile;

      mhdfile.open( mhdname.c_str() );

      mhdfile << "NDims = 3\n";
      mhdfile << "DimSize = " << dx << " " << dy << " " << dz << "\n";
      mhdfile << "ElementType = " << Private::elementTypeToMHDString(im.getElementType()) << "\n";
      mhdfile << "ElementSpacing = " << esx << " " << esy << " " << esz << "\n";
      mhdfile << "ElementByteOrderMSB = False\n";
      mhdfile << "ElementDataFile = " << rawname << "\n";

      mhdfile.close();
    }

  namespace Private
  {
    //------------------------------------------------------------------------------------------------//
    template<class DataType>
      void readMHDRawDataToBuffer(const MHDHeader& header, DataType* buffer)
      {
	int bytesPerElement = sizeof(DataType);
	int imgsize = header.dim[0] * header.dim[1] * header.dim[2] * header.dim[3];
	std::string path, base, suffix;
	splitFileNameForMHD(header.mhdFileName, path, base, suffix);
	std::stringstream fullname;
	fullname << path << header.rawFileName;
	std::ifstream  rawfile(fullname.str().c_str(), std::ios::in | std::ios::binary);
	if(!rawfile.good())
	  std::cout << "Warning: could not load image data! Reading error occurred" << std::endl;
	else
	  {
	    if(rawfile.is_open())
	      {
		std::vector<char> tmp(bytesPerElement * imgsize);
		rawfile.read(reinterpret_cast<char*>(&tmp[0]), static_cast<std::streamsize>(tmp.size()));
		convertToMHD(buffer,&tmp[0],imgsize,header.msb);
		rawfile.close();
	      }
	    else
	      std::cout << "Warning: could not load image data!" << std::endl;
	  }
	rawfile.close();
      }

    //------------------------------------------------------------------------------------------------//
    template<class DataType>
      void readMHDRawDataToBuffer(const MHDHeader& header, DataType* buffer, int label)
      {
	int bytesPerElement = sizeof(DataType);
	int imgsize = header.dim[0] * header.dim[1] * header.dim[2];
	std::string path, base, suffix;
	splitFileNameForMHD(header.mhdFileName, path, base, suffix);
	std::stringstream fullname;
	fullname << path << header.rawFileName;
	std::ifstream rawfile(fullname.str().c_str(), std::ios::in | std::ios::binary);
	if(!rawfile.good())
	  std::cout << "Warning: could not load image data! Reading error occurred" << std::endl;
	else
	  {
	    if(rawfile.is_open())
	      {
		std::vector<char> tmp(bytesPerElement * imgsize * header.dim[3]);
		rawfile.read(reinterpret_cast<char*>(&tmp[0]), static_cast<std::streamsize>(tmp.size()));
		convertToMHD(buffer,&tmp[bytesPerElement * imgsize * label],imgsize,header.msb);
		rawfile.close();
	      }
	    else
	      std::cout << "Warning: could not load image data!" << std::endl;
	  }
      }

    //------------------------------------------------------------------------------------------------//
    template<class DataType>
      void convertToMHD(DataType * dst, void * src, int N, bool swapBytes)
      {
	if (swapBytes)
	  {
	    unsigned char *data = (unsigned char *) src;
	    for (int i = 0; i < N; ++i) 
	      {
		unsigned char tmp;
		tmp = data[4*i]; data[4*i] = data[4*i+7]; data[4*i+7] = tmp;
		tmp = data[4*i+1]; data[4*i+1] = data[4*i+6]; data[4*i+6] = tmp;
		tmp = data[4*i+2]; data[4*i+2] = data[4*i+5]; data[4*i+5] = tmp;
		tmp = data[4*i+3]; data[4*i+3] = data[4*i+4]; data[4*i+4] = tmp;
	      }
	  }
	std::copy((DataType*) src, ((DataType*)src + N), dst);
      }
  }

	

  //------------------------------------------------------------------------------------------------//
  void splitFileNameForMHD(const std::string &filename, std::string &path, std::string &file, std::string &suffix)
  {
    std::string::size_type lastPosPath = filename.find_last_of("/");
    if(lastPosPath > filename.size())
      lastPosPath = filename.find_last_of("\\");
    path = "";
    if(lastPosPath < filename.size())
      path = std::string(filename.substr(0,lastPosPath+1));

    file = filename.substr(lastPosPath+1,filename.size());

    std::string::size_type firstPosSuffix = file.find_last_of(".");
    suffix = "";
    if(firstPosSuffix < file.size())
      suffix = file.substr(firstPosSuffix+1,file.size());

    if(firstPosSuffix < file.size() && suffix == "mhd")
      file = file.substr(0,firstPosSuffix);
  }

  namespace Private
  {
    const int ANA_DT_NONE			= 0;
    const int ANA_DT_UNKNOWN		= 0;
    const int ANA_DT_BINARY			= 1;
    const int ANA_DT_UNSIGNED_CHAR	= 2;
    const int ANA_DT_SIGNED_SHORT	= 4;
    const int ANA_DT_SIGNED_INT		= 8;
    const int ANA_DT_FLOAT			= 16;
    const int ANA_DT_COMPLEX		= 32;
    const int ANA_DT_DOUBLE			= 64;
    const int ANA_DT_RGB			= 128;
    const int ANA_DT_ALL			= 255;

    //------------------------------------------------------------------------------------------------//
    bool  readMHDHeader(const char * fileName, MHDHeader& header)
    {
      std::string pathPrefix;
      std::string tmp;

      splitPathPrefixAndFileName(std::string(fileName), pathPrefix, tmp);

      std::string keyword, keyval;

      header.numberDimensions = 0; 
      header.mhdFileName = fileName;
      header.rawFileName = "";
      header.numberChannels = 1;
      header.dim[0] = 1;
      header.dim[1] = 1;
      header.dim[2] = 1;
      header.dim[3] = 1;
      header.elementSpacing[0] = 1.0f;
      header.elementSpacing[1] = 1.0f;
      header.elementSpacing[2] = 1.0f;
      header.elementSpacing[3] = 1.0f;
      header.position[0] = 0.0f;
      header.position[1] = 0.0f;
      header.position[2] = 0.0f;
      header.position[3] = 0.0f;
      header.msb = false;
      header.headerSize = 0;

      std::ifstream file;
      file.open(fileName, std::ifstream::in);
      if (!file.good())
	return  false;

      std::string::iterator it;
      while (file.good())
	{
	  std::getline(file, keyword, '=');
	  it=keyword.end();
	  keyword.erase(std::remove(keyword.begin(),keyword.end(),' '),it);
	  if (keyword == "NDims") 
	    {
	      file >> header.numberDimensions;
	      std::getline(file, keyval, '\n');
	    } 
	  else if (keyword == "DimSize")
	    {
	      for (unsigned int i = 0; i < header.numberDimensions; i++) 
		file >> header.dim[i];
	      std::getline(file, keyval, '\n');
	    }
	  else if (keyword == "ElementSpacing") 
	    {
	      for (unsigned int i = 0; i < header.numberDimensions; i++)
		file >> header.elementSpacing[i];
	      std::getline(file, keyval, '\n');
	    }
	  else if (keyword == "Position")
	    {
	      for (unsigned int i = 0; i < header.numberDimensions; i++) 
		file >> header.position[i];
	      std::getline(file, keyval, '\n');
	    }
	  else if (keyword == "ImagePositionPatient")
	    {
	      std::getline(file, keyval, '\n');
	    }
	  else if (keyword == "ElementByteOrderMSB")
	    {
	      std::getline(file, keyval, '\n');
	      it=keyval.end();
	      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
	      it=keyval.end();
	      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
	      header.msb = ((keyval == "True") || (keyval == "1"));
	    } 
	  else if (keyword == "BinaryDataByteOrderMSB")
	    {
	      std::getline(file, keyval, '\n');
	      it=keyval.end();
	      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
	      it=keyval.end();
	      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
	      header.msb = ((keyval == "True") || (keyval == "1"));
	    } 
	  else if (keyword == "ElementNumberOfChannels")
	    {
	      file >> header.numberChannels;
	      std::getline(file, keyval, '\n');
	    }
	  else if (keyword == "ElementType")
	    {
	      std::getline(file, keyval, '\n');
	      it=keyval.end();
	      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
	      it=keyval.end();
	      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
	      header.elementType = mhdStringToElementType(keyval);
	    } 
	  else if (keyword == "HeaderSize")
	    {
	      file >> header.headerSize;
	      std::getline(file, keyval, '\n');
	      if ((header.headerSize) && (header.headerSize != -1)) {
		std::cout <<   "Warning: " << header.headerSize << " bytes header" << std::endl;
	      }
	    }
	  else if (keyword == "ElementDataFile")
	    {
	      std::getline(file, keyval, '\n');
	      it=keyval.end();
	      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
	      it=keyval.end();
	      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
	      header.rawFileName += keyval; 
	    }
	  else
	    {
	      if (keyword != "")
		{
		  std::getline(file, keyval, '\n');
		}
	    }
	}

      file.close();

      return true;
    }

    //------------------------------------------------------------------------------------------------//
    void splitPathPrefixAndFileName(const std::string& fullPath, std::string& pathPrefix, std::string& fileName)
    {
      std::string::size_type lastSlashSepPos = fullPath.find_last_of("/");
      std::string::size_type lastBackSlashSepPos = fullPath.find_last_of("\\");
      std::string::size_type lastPathSepPos = std::string::npos;

      if(lastSlashSepPos != std::string::npos)
	{
	  if(lastSlashSepPos != std::string::npos)
	    lastPathSepPos =  std::max(lastSlashSepPos, lastBackSlashSepPos);
	  else
	    lastPathSepPos =  lastSlashSepPos;
	}
      else
	{
	  if(lastBackSlashSepPos != std::string::npos)
	    lastPathSepPos = lastBackSlashSepPos;
	} 

      if(lastPathSepPos != std::string::npos)
	{
	  pathPrefix = fullPath.substr(0, lastPathSepPos + 1);
	  fileName = fullPath.substr(lastPathSepPos +1, fullPath.length());
	}
      else
	{
	  pathPrefix.clear();
	  fileName = fullPath;
	}
    }

    //------------------------------------------------------------------------------------------------//
    ElementType mhdStringToElementType(const std::string& mhdElementType)
    {
      if (!mhdElementType.compare("MET_UCHAR"))	return MET_UCHAR;
      if (!mhdElementType.compare("MET_CHAR"))	return MET_CHAR;
      if (!mhdElementType.compare("MET_USHORT"))	return MET_USHORT;
      if (!mhdElementType.compare("MET_SHORT"))	return MET_SHORT;
      if (!mhdElementType.compare("MET_UINT"))	return MET_UINT;
      if (!mhdElementType.compare("MET_INT")) 	return MET_INT;
      if (!mhdElementType.compare("MET_ULONG"))	return MET_ULONG;
      if (!mhdElementType.compare("MET_LONG")) 	return MET_LONG;
      if (!mhdElementType.compare("MET_FLOAT"))	return MET_FLOAT;
      if (!mhdElementType.compare("MET_DOUBLE"))	return MET_DOUBLE;
      if (!mhdElementType.compare("MET_VECTOR"))	return MET_VECTOR;
      return MET_UNKNOWN;
    }

    //------------------------------------------------------------------------------------------------//
    std::string elementTypeToMHDString(ElementType elementType)
      {
	switch(elementType)
	  {
	  case MET_CHAR:		return "MET_CHAR";
	  case MET_UCHAR:		return "MET_UCHAR";
	  case MET_SHORT:		return "MET_SHORT";
	  case MET_USHORT:	return "MET_USHORT";
	  case MET_INT:		return "MET_INT";
	  case MET_UINT:		return "MET_UINT";
	  case MET_LONG:		return "MET_LONG";
	  case MET_ULONG:		return "MET_ULONG";
	  case MET_FLOAT:		return "MET_FLOAT";
	  case MET_DOUBLE:	return "MET_DOUBLE";
	  case MET_VECTOR:	return "MET_VECTOR";
	  default:			return "";
	  }
      }
  }
}

#endif
