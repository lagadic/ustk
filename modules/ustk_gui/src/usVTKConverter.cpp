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
* @file usVTKConverter.cpp
* @brief
*/

#include <visp3/ustk_gui/usVTKConverter.h>
#include <vtkImageImport.h>

void usVTKConverter::convert(const usImagePostScan3D<unsigned char> &postScanImage,vtkSmartPointer<vtkImageData> &vtkPostScanImage)
{
  //vpTime::wait(3000);
  std::cout << "BEGIN COPY" << std::endl;
  double t0 = vpTime::measureTimeMs();
  vtkImageImport* importer = vtkImageImport::New();
  importer->SetDataScalarTypeToUnsignedChar();
  double t1 = vpTime::measureTimeMs();
  importer->SetImportVoidPointer((void *)postScanImage.getConstData());
  double t2 = vpTime::measureTimeMs();
  importer->SetWholeExtent(0,postScanImage.getDimX()-1,0, postScanImage.getDimY()-1, 0, postScanImage.getDimZ()-1);
  double t3 = vpTime::measureTimeMs();
  importer->SetDataExtentToWholeExtent();
  double t4 = vpTime::measureTimeMs();
  importer->SetNumberOfScalarComponents(1);
  double t5 = vpTime::measureTimeMs();
  importer->Update();
  double t6 = vpTime::measureTimeMs();

  vtkPostScanImage = importer->GetOutput();
  double t7 = vpTime::measureTimeMs();
  vtkPostScanImage->SetSpacing(postScanImage.getElementSpacingX(),postScanImage.getElementSpacingY(),postScanImage.getElementSpacingZ());
  double t8 = vpTime::measureTimeMs();


  std::cout << "t1 = " << t1-t0 << std::endl;
  std::cout << "t2 = " << t2-t0 << std::endl;
  std::cout << "t3 = " << t3-t0 << std::endl;
  std::cout << "t4 = " << t4-t0 << std::endl;
  std::cout << "t5 = " << t5-t0 << std::endl;
  std::cout << "t6 = " << t6-t0 << std::endl;
  std::cout << "t7 = " << t7-t0 << std::endl;
  std::cout << "t8 = " << t8-t0 << std::endl;
}

void usVTKConverter::convert(const usImagePreScan3D<unsigned char> &preScanImage,vtkSmartPointer<vtkImageData> &vtkPreScanImage)
{
  vtkImageImport* importer = vtkImageImport::New();
  importer->SetDataScalarTypeToUnsignedChar();
  importer->SetImportVoidPointer((void *)preScanImage.getConstData(),0);
  importer->SetWholeExtent(0,preScanImage.getDimX()-1,0, preScanImage.getDimY()-1, 0, preScanImage.getDimZ()-1);
  importer->SetDataExtentToWholeExtent();
  importer->SetNumberOfScalarComponents(1);
  importer->Update();

  vtkPreScanImage= importer->GetOutput();
}
