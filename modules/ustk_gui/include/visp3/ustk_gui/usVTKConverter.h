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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usVTKConverter.h
 * @brief Class to convert ustk image types to vtkImageData
 */

#ifndef US_VTK_CONVERTER
#define US_VTK_CONVERTER

//VISP includes
#include <visp3/core/vpConfig.h>

//USTK includes
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>

//VTK includes
#include <vtkSmartPointer.h>
#include <vtkImageData.h>

/**
 * @class usVTKConverter
 * @brief Class to convert ustk image types to vtkImageData
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usVTKConverter
{
public:
  static void convert(const usImagePostScan3D<unsigned char> &postScanImage, vtkSmartPointer<vtkImageData> &vtkPostScanImage);
  static void convert(const usImagePreScan3D<unsigned char> &preScanImage, vtkSmartPointer<vtkImageData> &vtkPreScanImage);

};

#endif // US_VTK_CONVERTER
