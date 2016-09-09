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
 * @file usDataIo.h
 * @brief Input/output operations for ultrasound data.
 * @author Pierre Chatelain
 */

#ifndef US_DATA_IO_H
#define US_DATA_IO_H

#include <string>

//#include <UsTk/usTkConfig.h>
#include <visp3/ustk_data/usDataPostscan2D.h>
#include <visp3/ustk_data/usDataPostscan3D.h>
#include <visp3/ustk_data/usDataPrescan2D.h>
#include <visp3/ustk_data/usDataPrescan3D.h>
#include <visp3/ustk_data/usDataRF3D.h>

/// Header for propello volume data
struct propelloHeader
{
  int type;	      ///< type of data (0 = pre-scan, 1 = post-scan, 2 = rf)
  int volumes;	      ///< volumes inside the file
  int fpv;	      ///< frames per volume
  int w; 	      ///< width of a frame (pixels for post-scan, scanlines for  pre-scan or rf data)
  int h; 	      ///< height of frame (pixels for post-scan, samples for pre-scan or rf data)
  int ss;	      ///< sample size in bits
  int degPerFr;       ///< degree step between frames
};

/// Header for ulterius data
struct uFileHeader
{
    /// data type - data types can also be determined by file extensions
    int type;
    /// number of frames in file
    int frames;
    /// width - number of vectors for raw data, image width for processed data
    int w;
    /// height - number of samples for raw data, image height for processed data
    int h;
    /// data sample size in bits
    int ss;
    /// roi - upper left (x)
    int ulx;
    /// roi - upper left (y)
    int uly;
    /// roi - upper right (x)
    int urx;
    /// roi - upper right (y)
    int ury;
    /// roi - bottom right (x)
    int brx;
    /// roi - bottom right (y)
    int bry;
    /// roi - bottom left (x)
    int blx;
    /// roi - bottom left (y)
    int bly;
    /// probe identifier - additional probe information can be found using this id
    int probe;
    /// transmit frequency
    int txf;
    /// sampling frequency
    int sf;
    /// data rate - frame rate or pulse repetition period in Doppler modes
    int dr;
    /// line density - can be used to calculate element spacing if pitch and native # elements is known
    int ld;
    /// extra information - ensemble for color RF
    int extra;
};

/**
 * @namespace usDataIo
 * @brief Input/output operations for ultrasound data.
 *
 * This namespace contains input/output operations for the derivates of usData.
 */
namespace usDataIo
{
  /**
   * Read ultrasound data.
   * @param[in,out] data The ultrasound data.
   * @param[in] filename Path to the data file.
   *
   * The type of data to read is deduced from the class of data.
   */
	VISP_EXPORT void read(usData &data, const char *filename);

  /**
   * Read 2D postscan ultrasound data.
   * @param[out] data The ultrasound data.
   * @param[in] Path to the data file.
   *
   * The file should be in an image format readable by ViSP.
   */
	VISP_EXPORT void read(usDataPostscan2D &data, const char *filename);

  /**
   * Read 3D postscan ultrasound data.
   * @param[out] data The ultrasound data.
   * @param[in] Path to the data file.
   *
   * The file should be in MetaImage format (.mhd).
   */
	VISP_EXPORT void read(usDataPostscan3D &data, const char *filename);

  /**
   * Read 2D prescan ultrasound data.
   * @param[out] data The ultrasound data.
   * @param[in] Path to the data file.
   *
   * The file should be in an image format reabable by ViSP.
   */
	VISP_EXPORT void read(usDataPrescan2D &data, const char *filename);

  /**
   * Read 2D postscan ultrasound data.
   * @param[out] data The ultrasound data.
   * @param[in] Path to the data file.
   *
   * The file should be in MetaImage format (.mhd).
   */
	VISP_EXPORT void read(usDataPrescan3D &data, const char *filename);

  /**
   * Read 3D RF ultrasound data.
   * @param[out] data The ultrasound data.
   * @param[in] Path to the data file.
   *
   * The file should be in MetaImage format (.mhd).
   */
	VISP_EXPORT void read(usDataRF3D &data, const char *filename);

  /**
   * Open a Propello file and read the header.
   * @param[out] header The header.
   * @param[in] filename Path to the file.
   * @return A pointer to the FILE object, or NULL if the file could not be opened or the header
   * could not be parsed correctly.
   * If the header parsing is successful, then the position indicator of the stream points to the
   * first data element.
   */
	VISP_EXPORT FILE* readPropelloHeader(propelloHeader& header, const char *filename);

  /**
   * Open an Ulterius file and read the header.
   * @param[out] header The header.
   * @param[in] filename Path to the file.
   * @return A pointer to the FILE object, or NULL if the file could not be opened or the header
   * could not be parsed correctly.
   * If the header parsing is successful, then the position indicator of the stream points to the
   * first data element.
   */
	VISP_EXPORT FILE* readUlteriusHeader(uFileHeader &header, const char *filename);

  /**
   * Write ultrasound data.
   * @param[in] data The ultrasound data.
   * @param[in] filename Path to the file to write.
   */
	VISP_EXPORT void write(const usData &data, const char *filename);

  /**
   * Write 2D postscan ultrasound data.
   * @param[in] data The ultrasound data.
   * @param[in] filename Path to the data file.
   */
	VISP_EXPORT void write(const usDataPostscan2D &data, const char *filename);

  /**
   * Write 3D postscan ultrasound data.
   * @param[in] data The ultrasound data.
   * @param[in] filename Path to the data file.
   */
	VISP_EXPORT  void write(const usDataPostscan3D &data, const char *filename);

  /**
   * Write 2D prescan ultrasound data.
   * @param[in] data The ultrasound data.
   * @param[in] filename Path to the data file.
   */
	VISP_EXPORT void write(const usDataPrescan2D &data, const char *filename);

  /**
   * Write 3D precan ultrasound data.
   * @param[in] data The ultrasound data.
   * @param[in] filename Path to the data file.
   */
   VISP_EXPORT void write(const usDataPrescan3D &data, const char *filename);

  /**
   * Write 3D RF ultrasound data.
   * @param[in] data The ultrasound data.
   * @param[in] filename Path to the data file.
   */
   VISP_EXPORT void write(const usDataRF3D &data, const char *filename);

  /**
   * Write 2D prescan ultrasound data as MHD.
   * @param[in] data The ultrasound data.
   * @param[in] filename File name.
   */
   VISP_EXPORT void writeMHD(const usDataPrescan2D &data, const char *filename);

}

#endif
