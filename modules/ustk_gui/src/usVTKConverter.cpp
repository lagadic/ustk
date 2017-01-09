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

void usVTKConverter::convert(const usImagePostScan3D<unsigned char> &postScanImage,vtkSmartPointer<vtkImageData> vtkPostScanImage) {

  std::cout << "toto" << std::endl;
  //set image settings (dims and spacings), and allocate memory
  vtkPostScanImage->SetDimensions(postScanImage.getDimX(), postScanImage.getDimY(),postScanImage.getDimZ());
  std::cout << "toto" << std::endl;
  vtkPostScanImage->AllocateScalars(VTK_UNSIGNED_CHAR,1);
  std::cout << "toto" << std::endl;
  vtkPostScanImage->SetSpacing(postScanImage.getElementSpacingX(),postScanImage.getElementSpacingY(),postScanImage.getElementSpacingZ());
  std::cout << "toto" << std::endl;

  unsigned char * dataToCopy = postScanImage.getConstData();

  for (int i = 0; i < postScanImage.getDimX(); i++) {
    for (int j = 0; j < postScanImage.getDimY(); j++) {
      for (int k = 0; k < postScanImage.getDimZ(); k++) {
        std::cout << "i = " << i << ", j = " << j << ", k = " << k << std::endl;
        // std::cout << "imageData [" << i << "] [" << j << "] [" << k << "] = " << reader->GetOutput()->GetScalarComponentAsDouble(i,j,k,0) << std::endl;
        *static_cast<unsigned char*>(vtkPostScanImage->GetScalarPointer(i,j,k)) = *dataToCopy++;
      }
    }
  }

}

void usVTKConverter::convert(const usImagePreScan3D<unsigned char> &preScanImage,vtkSmartPointer<vtkImageData> &vtkPreScanImage) {

  std::cout << "convert() data ptr = " << (void*)preScanImage.getConstData() << std::endl;
  vtkImageImport* importer = vtkImageImport::New();
  importer->SetDataScalarTypeToUnsignedChar();
  importer->SetImportVoidPointer((void *)preScanImage.getConstData(),0);
  importer->SetWholeExtent(0,preScanImage.getDimX()-1,0, preScanImage.getDimY()-1, 0, preScanImage.getDimZ()-1);
  importer->SetDataExtentToWholeExtent();
  importer->SetNumberOfScalarComponents(1);
  importer->Update();

  vtkPreScanImage= importer->GetOutput();

/*
  for (int i = 0; i < preScanImage.getDimX(); i++) {
    for (int j = 0; j < preScanImage.getDimY(); j++) {
      for (int k = 0; k < preScanImage.getDimZ(); k++) {
        //std::cout << "i = " << i << ", j = " << j << ", k = " << k << std::endl;
        std::cout << "imageData (" << i << ", " << j << ", " << k << ") = " << (int)  preScanImage(i,j,k) << std::endl;
      }
    }
  }

  std::cout << "*********************************************************" << std::endl<< std::endl;

  unsigned char * dataPtr = preScanImage.getConstData();
  int i=0;
  while(i<26) {
    std::cout << "imageData [" << i << "] =" << (int) *dataPtr << std::endl;
    i++;
    dataPtr++;
  }

  std::cout << "*********************************************************" << std::endl<< std::endl;


  for (int i = 0; i < preScanImage.getDimX(); i++) {
    for (int j = 0; j < preScanImage.getDimY(); j++) {
      for (int k = 0; k < preScanImage.getDimZ(); k++) {
        //std::cout << "i = " << i << ", j = " << j << ", k = " << k << std::endl;
        std::cout << "imageData [" << i << "] [" << j << "] [" << k << "] = " << vtkPreScanImage->GetScalarComponentAsDouble(i,j,k,0) << std::endl;
      }
    }
  }*/


/*
  std::cout << "toto" << std::endl;
  //set image settings (dims and spacings), and allocate memory
  int dims[3];
  dims[0] = 3;
  dims[1] = 3;
  dims[2] = 3;
  vtkPreScanImage->AllocateScalars(VTK_UNSIGNED_CHAR,1);
  std::cout << "toto" << std::endl;
  vtkPreScanImage->SetDimensions(dims);
  std::cout << "toto" << std::endl;
  unsigned char * dataToCopy = preScanImage.getConstData();

  for (int i = 0; i < preScanImage.getDimX(); i++) {
    for (int j = 0; j < preScanImage.getDimY(); j++) {
      for (int k = 0; k < preScanImage.getDimZ(); k++) {
        std::cout << "i = " << i << ", j = " << j << ", k = " << k << std::endl;
        // std::cout << "imageData [" << i << "] [" << j << "] [" << k << "] = " << reader->GetOutput()->GetScalarComponentAsDouble(i,j,k,0) << std::endl;
        *static_cast<unsigned char*>(vtkPreScanImage->GetScalarPointer(i,j,k)) = *dataToCopy++;
      }
    }
  }
*/
}
