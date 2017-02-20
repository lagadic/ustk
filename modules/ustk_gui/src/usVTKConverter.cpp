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
* @file usVTKConverter.cpp
* @brief Class of static methods to convert data between ViSP and VTK (3D images, homogeneous matrices).
*/

#include <visp3/ustk_gui/usVTKConverter.h>

#ifdef USTK_HAVE_VTK_QT

#include <vtkImageImport.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>

/**
* Converts a usImagePostScan3D to a vtkImageData.
*/
void usVTKConverter::convert(const usImagePostScan3D<unsigned char> &postScanImage, vtkSmartPointer<vtkImageData> &vtkPostScanImage, vtkSmartPointer<vtkImageImport> importer)
{
  if(importer==NULL) {
    importer = vtkSmartPointer<vtkImageImport>::New();
    importer->SetDataScalarTypeToUnsignedChar();
    importer->SetWholeExtent(0,postScanImage.getDimX()-1,0, postScanImage.getDimY()-1, 0, postScanImage.getDimZ()-1);
    importer->SetDataExtentToWholeExtent();
    importer->SetNumberOfScalarComponents(1);
    importer->SetImportVoidPointer((void *)postScanImage.getConstData(),1);
  }
  else
    importer->SetImportVoidPointer((void *)postScanImage.getConstData());

  importer->Update();

  vtkPostScanImage = importer->GetOutput();
  vtkPostScanImage->SetSpacing(postScanImage.getElementSpacingX(),postScanImage.getElementSpacingY(),postScanImage.getElementSpacingZ());
}

/**
* Converts a usImagePreScan3D to a vtkImageData.
*/
void usVTKConverter::convert(const usImagePreScan3D<unsigned char> &preScanImage,vtkSmartPointer<vtkImageData> &vtkPreScanImage, vtkSmartPointer<vtkImageImport> importer)
{
  if(importer==NULL) {
    importer = vtkSmartPointer<vtkImageImport>::New();
    importer->SetDataScalarTypeToUnsignedChar();
    importer->SetImportVoidPointer((void *)preScanImage.getConstData());
    importer->SetWholeExtent(0,preScanImage.getDimX()-1,0, preScanImage.getDimY()-1, 0, preScanImage.getDimZ()-1);
    importer->SetDataExtentToWholeExtent();
    importer->SetNumberOfScalarComponents(1);
  }
  else
    importer->SetImportVoidPointer((void *)preScanImage.getConstData());

  importer->Update();

  vtkPreScanImage = importer->GetOutput();
}

/**
* Converts a vtkImageData to a usImagePostScan2D. Usefull to extract a slice from a volume. Performs a deep copy.
*/
void usVTKConverter::convert(vtkSmartPointer<vtkImageData> &vtkPostScanImage, usImagePostScan2D<unsigned char> &postScanImage)
{
  //get dimentions/spacing of the 3D image.
  int imageDims[3];
  vtkPostScanImage->GetDimensions(imageDims);
  double spacing[3];
  vtkPostScanImage->GetSpacing(spacing);

  postScanImage.resize(imageDims[1],imageDims[0]);

  for (int i = 0; i < imageDims[0]; i++) {
    for (int j= 0; j< imageDims[1]; j++) {
      postScanImage[j][i] = (unsigned char) (vtkPostScanImage->GetScalarComponentAsDouble(i,j,0,0));
    }
  }

  postScanImage.setWidthResolution(spacing[0]);
  postScanImage.setHeightResolution(spacing[1]);
}

/**
* Converts a vpHomogeneousMatrix to a vtkMatrix4x4.
*/
void usVTKConverter::convert(const vpHomogeneousMatrix & vispMatrix, vtkMatrix4x4 * vtkMatrix) {
  for(int i=0; i<4; i++)
    for(int j=0;j<4; j++)
      vtkMatrix->SetElement(i,j,vispMatrix[i][j]);
}

/**
* Converts a vtkMatrix4x4 to a vpHomogeneousMatrix.
*/
void usVTKConverter::convert(vtkMatrix4x4 * vtkMatrix, vpHomogeneousMatrix & vispMatrix) {
  for(int i=0; i<4; i++)
    for(int j=0;j<4; j++)
      vispMatrix[i][j] = vtkMatrix->GetElement(i,j);
}

#endif
