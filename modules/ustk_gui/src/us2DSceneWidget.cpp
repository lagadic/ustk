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
 * @file us2DSceneWidget.cpp
 * @brief Class used to render a 3D vtk scene containing a vtkImageData in a QWidget (based on QVTKWidget)
 */

// VISP includes
#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_gui/usViewerWidget.h>

//USTK includes
#include <visp3/ustk_gui/us2DSceneWidget.h>

us2DSceneWidget::us2DSceneWidget(QWidget* parent, Qt::WindowFlags f) : usViewerWidget(parent,f) {

  m_imageData = NULL;
  m_resliceMatrix = NULL;

  m_reslicePlane = NULL;

  m_reslice = vtkImageReslice::New();

  m_renderer = vtkRenderer::New();

  m_table = vtkLookupTable::New();

  m_color = vtkImageMapToColors::New();

  m_actor = vtkImageActor::New();

  //m_mapper = vtkImageMapper3D::New();
}

void us2DSceneWidget::paintEvent( QPaintEvent* event ) {
  usViewerWidget::paintEvent(event);
}

vtkImageData* us2DSceneWidget::getImageData() {
  return m_imageData;
}

vtkMatrix4x4* us2DSceneWidget::getResliceMatrix() {
  return m_resliceMatrix;
}

void us2DSceneWidget::init() {

  //verify imageData and reslice matrix are set
  if(m_imageData == NULL)
    throw(vpException(vpException::fatalError,"No imageData provided in us2DSceneWidget"));

  if(m_resliceMatrix == NULL)
    throw(vpException(vpException::fatalError,"No reslice matrix provided in us2DSceneWidget"));

  m_reslice->SetInputData(m_imageData);
  m_reslice->SetOutputDimensionality(2);
  m_reslice->SetResliceAxes(m_resliceMatrix);
  m_reslice->SetInterpolationModeToLinear();

  //To ensure no part of the image will be cropped
  m_reslice->AutoCropOutputOn();

  // Create a greyscale lookup table
  m_table->SetRange(0, 255); // image intensity range
  m_table->SetValueRange(0.0, 1.0); // from black to white
  m_table->SetSaturationRange(0.0, 0.0); // no color saturation
  m_table->SetRampToLinear();
  m_table->Build();

  // Map the image through the lookup table
  m_color->SetLookupTable(m_table);
  m_color->SetInputConnection(m_reslice->GetOutputPort());

  // Display the image
  m_actor->GetMapper()->SetInputConnection(m_color->GetOutputPort());

  m_renderer->AddActor(m_actor);

  // Setup render window
  vtkRenderWindow* renderWindow = this->GetRenderWindow();
  renderWindow->AddRenderer(m_renderer);

  // Set up the interaction
  vtkSmartPointer<vtkInteractorStyleImage> imageStyle =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
    imageStyle->SetInteractionModeToImageSlicing();

  renderWindow->GetInteractor()->SetInteractorStyle(imageStyle);

  m_callback = vtkSmartPointer<usImage2DInteractionCallback>::New();
  m_callback->SetImageReslice(m_reslice);
  m_callback->SetInteractor(renderWindow->GetInteractor());
  m_callback->SetPlane(m_reslicePlane);

  imageStyle->AddObserver(vtkCommand::MouseWheelForwardEvent, m_callback);
  imageStyle->AddObserver(vtkCommand::MouseWheelBackwardEvent, m_callback);
}

void us2DSceneWidget::setImageData(vtkImageData* imageData) {
  m_imageData = imageData;
}

void us2DSceneWidget::setResliceMatrix(vtkMatrix4x4 *matrix, vtkPlane* plane) {
  m_resliceMatrix = matrix;

  //update plane origin
  vpHomogeneousMatrix hMat;
  usVTKConverter::convert(m_resliceMatrix,hMat);
  m_reslicePlane = plane;
  double origin[3];
  m_reslicePlane->GetOrigin(origin);
  std::cout << "old origin " << origin[0] << "," << origin[1] << "," << origin[2] << std::endl;
  m_reslicePlane->SetOrigin(hMat.getTranslationVector()[0],hMat.getTranslationVector()[1],hMat.getTranslationVector()[2]);
  m_reslicePlane->GetOrigin(origin);
  std::cout << "new origin " << origin[0] << "," << origin[1] << "," << origin[2] << std::endl;

  //update plane normal (in test)
  vpThetaUVector rotationVector;
  m_reslicePlane->GetNormal(rotationVector.data);
  std::cout << "old normal " << rotationVector[0] << "," << rotationVector[1] << "," << rotationVector[2] << std::endl;

  //rotationVector = hMat.getThetaUVector();

  m_reslicePlane->SetNormal(0,0.5,0.5);
  m_reslicePlane->GetNormal(rotationVector.data);
  std::cout << "new normal " << rotationVector[0] << "," << rotationVector[1] << "," << rotationVector[2] << std::endl;
  std::cout << std::endl;
}

void us2DSceneWidget::updateImageData(vtkImageData* imageData) {
  m_imageData = imageData;
}

void us2DSceneWidget::matrixChangedSlot(vtkMatrix4x4* matrix) {
  matrix->Print(std::cout);
}

vtkPlane* us2DSceneWidget::getReslicePlane() {
  return m_reslicePlane;
}

#endif //USTK_HAVE_VTK_QT
