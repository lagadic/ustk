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

/**
 * @file usVTKConverter.h
 * @brief Class to convert ustk image types to vtkImageData
 */

#ifndef __usVTKConverter_h_
#define __usVTKConverter_h_

// VISP includes
#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

// USTK includes
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>

/**
 * @class usVTKConverter
 * @brief Class to convert ustk image types to vtkImageData
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usVTKConverter
{
public:
  static void convert(const usImagePostScan3D<unsigned char> &postScanImage, vtkSmartPointer<vtkImageData> &vtkPostScanImage, vtkSmartPointer<vtkImageImport> importer = NULL);
  static void convert(const usImagePreScan3D<unsigned char> &preScanImage,vtkSmartPointer<vtkImageData> &vtkPreScanImage, vtkImageImport* importer);

};
#endif
#endif // US_VTK_CONVERTER
