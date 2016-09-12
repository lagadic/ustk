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

#include <visp3/ustk_grabber/usGrabberFile.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <visp3/ustk_io/usDataIo.h>
#include <visp3/ustk_grabber/usGrabberException.h>
#include <visp3/core/vpIoTools.h>

usGrabberFile::usGrabberFile() {
  m_data = NULL;
  m_idx = -1;
  m_sequenceLength = -1;
  m_sequenceFileName = "";
  m_dirName = "";
  m_baseName = "";
  m_extension = "";
}

void usGrabberFile::start() {
  std::string fullSequenceFileName = vpIoTools::path(m_dirName + "/" + m_sequenceFileName);
  if (!vpIoTools::checkFilename(fullSequenceFileName)) {
    throw usGrabberException(usGrabberException::fileError, "Cannot read file.");
  }
  
  m_extension = getFileExtension(m_sequenceFileName);
  
  if (!m_extension.compare("vol")) {
    std::cout << "This is a Propello file." << std::endl;
    propelloHeader header;

    // Open file
    m_pFile = usDataIo::readPropelloHeader(header, fullSequenceFileName.c_str());
    if (m_pFile == NULL) {
      throw usGrabberException(usGrabberException::fileError, "Cannot read file.");
    }
    
    int type = header.type;
    m_sequenceLength = header.volumes;
    m_dimensionality = 3;
    m_idx = 0;
    int fpv = header.fpv;
    int width = header.w;
    int height = header.h;
    int sampleSize = header.ss;
    int degPerFrame = header.degPerFr;
    int frameSize = width * height;
    int volSize = frameSize * fpv;

    std::cout << "Parameters : " << std::endl
	      << "   type = " << type << std::endl
	      << "   numVols = " << m_sequenceLength << std::endl
	      << "   fpv = " << fpv << std::endl
	      << "   width = " << width << std::endl
	      << "   height = " << height << std::endl
	      << "   sampleSize = " << sampleSize << std::endl
	      << "   degPerFrame = " << degPerFrame << std::endl;

    // Initialize data
    switch (type) {
    case 0: // Pre-scan
      {
	m_dataSize = volSize * sizeof(unsigned char);
	m_mode = "prescan";
	m_data = new usDataPrescan3D(height, width, fpv);
	break;
      }
    case 1: // Post-scan
      {/*
	m_dataSize = volSize * sizeof(unsigned char);
	m_mode = "postscan";
	m_data = new usDataPostscan3D(height, width, fpv);
	if (fread(static_cast<usDataPostscan3D*>(m_data)->getData(), m_dataSize, 1, m_pFile) != 1)
	throw usGrabberException(usGrabberException::fileError, "Error reading data.");*/
	break;
      }
    case 2: // RF
      {
	m_dataSize = volSize * sizeof(short);
	m_mode = "rf";
	m_data = new usDataRF3D(height, width, fpv);
	break;
      }
    default:
	throw usGrabberException(usGrabberException::fileError, "Wrong data type.");      
    }

  }
  else if (!m_extension.compare("txt")) {
    std::cout << "Loading configuration file... " << std::flush;
    
    if (!vpIoTools::loadConfigFile(fullSequenceFileName))
      throw usGrabberException(usGrabberException::fileError, "Failed to load configuration file.");
    
    if (!vpIoTools::readConfigVar("nbImages", m_sequenceLength))
      throw usGrabberException(usGrabberException::missingConfigVar,
			       "Missing configuration variable nbImages.");
      
    if (!vpIoTools::readConfigVar("mode", m_mode))
      throw usGrabberException(usGrabberException::missingConfigVar,
			       "Missing configuration variable mode.");

    if (!vpIoTools::readConfigVar("dimensionality", m_dimensionality))
      throw usGrabberException(usGrabberException::missingConfigVar,
			       "Missing configuration variable dimensionality.");
      
    if (!vpIoTools::readConfigVar("baseName", m_baseName))
      throw usGrabberException(usGrabberException::missingConfigVar,
			       "Missing configuration variable baseName.");

    if (!vpIoTools::readConfigVar("extension", m_extension))
      throw usGrabberException(usGrabberException::missingConfigVar,
			       "Missing configuration variable extension.");

    // Optional parameters
    std::string probeName, scannerName;
    double resolution;
    
    if (!vpIoTools::readConfigVar("probeType", probeName))
      probeName = "";

    if (!vpIoTools::readConfigVar("scannerType", scannerName))
      scannerName = "";

    if (!vpIoTools::readConfigVar("resolution", resolution))
      resolution = 0.0;

    m_resolution = resolution;
    
    usProbeType probeType;
    usScannerType scannerType;

    if (!probeName.compare("US_4DC7"))
      probeType = US_4DC7;
    else if (!probeName.compare("SS_C60"))
      probeType = SS_C60;
    else
      probeType = UNKNOWN_PROBE;

    if (!scannerName.compare("SONIX_RP"))
      scannerType = SONIX_RP;
    else if (!scannerName.compare("SONIX_TOUCH"))
      scannerType = SONIX_TOUCH;
    else if (!scannerName.compare("SONOSITE"))
      scannerType = SONOSITE;
    else
      scannerType = UNKNOWN_SCANNER;
    
    m_idx = 0;
    
    if (!m_mode.compare("prescan")) {
      if (m_dimensionality == 2) {
	m_data = new usDataPrescan2D();
	if (!probeName.compare("US_4DC7")) {
	  dynamic_cast<usDataPrescan2D*>(m_data)->setProbeRadius(0.0398);
	  dynamic_cast<usDataPrescan2D*>(m_data)->setLineAngle(0.000425/0.0398);
	}
      } else if (m_dimensionality == 3) {
	m_data = new usDataPrescan3D();
      }
    } else if (!m_mode.compare("postscan")) {
      if (m_dimensionality == 2) {
	m_data = new usDataPostscan2D();
      } else if (m_dimensionality == 3) {
	int originX, originY, originZ;
	vpIoTools::readConfigVar("originX", originX);
	vpIoTools::readConfigVar("originY", originY);
	vpIoTools::readConfigVar("originZ", originZ);
	m_data = new usDataPostscan3D();
	m_data->setOrigin(originX, originY, originZ);
      }

      if (m_resolution == 0.0) {
	throw usGrabberException(usGrabberException::missingConfigVar,
				 "Missing configuration variable resolution.");
      }
    }

    m_data->setProbeType(probeType);
    m_data->setScannerType(scannerType);

    char fileName[256];

    // Try with 4 digits
    std::sprintf(fileName, "%s/%s%04d.%s", m_dirName.c_str(), m_baseName.c_str(),
		 m_idx, m_extension.c_str());
    
    if (!vpIoTools::checkFilename(fileName)) {
      // Try with 5 digits
      std::sprintf(fileName, "%s/%s%05d.%s", m_dirName.c_str(), m_baseName.c_str(),
		   m_idx, m_extension.c_str());
      
      if (!vpIoTools::checkFilename(fileName)) {
	throw usGrabberException(usGrabberException::fileError, "Cannot read file: %s", fileName);
      }

      m_digits = 5;
    } else {
      m_digits = 4;
    }
	
    usDataIo::read(*m_data, fileName);   
    double timestamp = vpTime::measureTimeMs();
    
    m_data->setTimestamp(timestamp);
    m_data->setDataIdx(m_idx); 
    
    std::cout << "done." << std::endl;
  }
  else {
    std::cerr << "Error: In usGrabberFile::start(): "
	      << "Extension " << m_extension << " not recognized." << std::endl;
  }
}

void usGrabberFile::grab() {
  // Check whether we are at the end of the sequence
  if (m_idx == m_sequenceLength) {
    throw usGrabberException(usGrabberException::endOfSequence, "End of sequence.");
  }
  // Read data
  if (!m_extension.compare("vol")) { // Propello
    if (!m_mode.compare("prescan")) {
      if (fread(static_cast<usDataPrescan3D*>(m_data)->getData(), m_dataSize, 1, m_pFile) != 1)
	throw usGrabberException(usGrabberException::fileError, "Error reading data.");
    }
    else if (!m_mode.compare("postscan")) {
      if (fread(static_cast<usDataPostscan3D*>(m_data)->getData(), m_dataSize, 1, m_pFile) != 1)
	throw usGrabberException(usGrabberException::fileError, "Error reading data.");
    }
    else if (!m_mode.compare("rf")) {
      if (fread(static_cast<usDataRF3D*>(m_data)->getData(), m_dataSize, 1, m_pFile) != 1)
	throw usGrabberException(usGrabberException::fileError, "Error reading data.");
    }
  }
  else { // File sequence
    char fileName[256];
    if (m_digits == 4) {
      std::sprintf(fileName, "%s/%s%04d.%s", m_dirName.c_str(), m_baseName.c_str(), m_idx,
		   m_extension.c_str());
    } else {
      std::sprintf(fileName, "%s/%s%05d.%s", m_dirName.c_str(), m_baseName.c_str(), m_idx,
		   m_extension.c_str());
    }
    if (!vpIoTools::checkFilename(fileName)) {
      throw usGrabberException(usGrabberException::fileError, "Cannot read file.");
    }
    usDataIo::read(*m_data, fileName);
  }
  
  // Set timestamp and index
  double timestamp = vpTime::measureTimeMs();
  m_data->setTimestamp(timestamp);
  m_data->setDataIdx(m_idx);
  m_idx++;
}

void usGrabberFile::stop() {
  if (m_data) {
    delete m_data;
    m_data = NULL;
  }
  m_idx = -1;
  m_sequenceLength = -1;
}

void usGrabberFile::setFileName(std::string dir, std::string file) {
  m_dirName = dir + getDirPath(file);
  m_sequenceFileName = getShortFilename(file);
}

std::string usGrabberFile::getFileName() {
  std::string fullSequenceFileName = vpIoTools::path(m_dirName + "/" + m_sequenceFileName);
  return fullSequenceFileName;
}

int usGrabberFile::getSequenceLength() {
  return m_sequenceLength;
}

std::string usGrabberFile::getFileExtension(const std::string &filename)
{
  if (filename.find_last_of(".") != std::string::npos)
    return filename.substr(filename.find_last_of(".") + 1);
  return "";
}

std::string usGrabberFile::getDirPath(const std::string &filename)
{
  if (filename.find_last_of("/\\") != std::string::npos)
    return filename.substr(0, filename.find_last_of("/\\"));
  return "";
}

std::string usGrabberFile::getShortFilename(const std::string &filename)
{
  if (filename.find_last_of("/\\") != std::string::npos)
    return filename.substr(filename.find_last_of("/\\") + 1);
  return filename;
}
