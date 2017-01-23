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
* @file us3DSceneWidget.cpp
* @brief Graphical main window containing 4 vtk views.
*/
#include <visp3/core/vpException.h>
#include <visp3/ustk_gui/us3DSceneWidget.h>

#ifdef USTK_HAVE_VTK_QT

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkDistanceWidget.h>
#include <vtkResliceImageViewerMeasurements.h>
#include <vtkResliceImageViewer.h>
#include <vtkImagePlaneWidget.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkImageResliceMapper.h>

#include <QPainter>
#include <QPaintEngine>

/**
* Constructor.
*/
us3DSceneWidget::us3DSceneWidget(QWidget* parent, Qt::WindowFlags f) : usViewerWidget(parent,f)
{

  imageResliceMapper1 = vtkImageResliceMapper::New();
  imageResliceMapper2 = vtkImageResliceMapper::New();
  imageResliceMapper3 = vtkImageResliceMapper::New();

  plane1 = NULL;
  plane2 = NULL;
  plane3 = NULL;

  imageSlice1 = vtkImageSlice::New();
  imageSlice2 = vtkImageSlice::New();
  imageSlice3 = vtkImageSlice::New();

  renderer = vtkRenderer::New();

}

/**
* Qt paint event overload if needed to update Qt widget
* @param event QPaintEvent.
*/
void us3DSceneWidget::paintEvent( QPaintEvent* event )
{
  QVTKWidget::paintEvent( event );
}

/**
* Getter for image data pointer
* @return vtkImageData currently displayed
*/
vtkImageData* us3DSceneWidget::getImageData() {
  return this->imageData;
}

/**
* Init method : setup vtk pipeline. Make sure imageData and planes are set before calling init().
*/
void us3DSceneWidget::init() {
  if(this->imageData == NULL)
    throw(vpException(vpException::fatalError, "no vtk image provided"));

  //show the widget
  imageResliceMapper1->SetInputData(this->imageData);
  imageResliceMapper2->SetInputData(this->imageData);
  imageResliceMapper3->SetInputData(this->imageData);

  this->imageData->SetOrigin(0,0,0);

  if(plane1 == NULL || plane2 == NULL || plane3 == NULL)
    throw(vpException(vpException::fatalError, "no vtk planes provided"));

  //select plane
  imageResliceMapper1->SetSlicePlane(plane1);
  imageResliceMapper2->SetSlicePlane(plane2);
  imageResliceMapper3->SetSlicePlane(plane3);

  imageSlice1->SetMapper(imageResliceMapper1);
  imageSlice2->SetMapper(imageResliceMapper2);
  imageSlice3->SetMapper(imageResliceMapper3);

  // Setup renderers
  renderer = vtkRenderer::New();
  renderer->AddActor(imageSlice1);
  renderer->AddActor(imageSlice2);
  renderer->AddActor(imageSlice3);
  renderer->SetBackground(0.5, 0.5, 0.5);
  renderer->ResetCamera();

  // Setup render window
  vtkRenderWindow* renderWindow = this->GetRenderWindow();
  renderWindow->AddRenderer(renderer);

  // Interaction style with the 3D image
  vtkInteractorStyleTrackballCamera* style =
    vtkInteractorStyleTrackballCamera::New();

  renderWindow->GetInteractor()->SetInteractorStyle(style);
}

/**
* Image data setter.
* @param imageData Pointer on vtkImageData to display.
*/
void us3DSceneWidget::setImageData(vtkImageData* imageData) {
  this->imageData = imageData;
}

/**
* Plane 1 setter.
* @param plane Pointer on first vtkPlane.
*/
void us3DSceneWidget::setPlane1(vtkPlane* plane) {
  this->plane1 = plane;
}

/**
* Plane 2 setter.
* @param plane Pointer on second vtkPlane.
*/
void us3DSceneWidget::setPlane2(vtkPlane* plane) {
  this->plane2 = plane;
}

/**
* Plane 3 setter.
* @param plane Pointer on third vtkPlane.
*/
void us3DSceneWidget::setPlane3(vtkPlane* plane) {
  this->plane3 = plane;
}

/**
* All planes setter.
* @param plane1 Pointer on first vtkPlane.
* @param plane2 Pointer on second vtkPlane.
* @param plane3 Pointer on third vtkPlane.
*/
void us3DSceneWidget::setPlanes(vtkPlane* plane1,vtkPlane* plane2,vtkPlane* plane3) {
  this->plane1 = plane1;
  this->plane2 = plane2;
  this->plane3 = plane3;
}

/**
* Slot called to update image to display
*/
void us3DSceneWidget::updateImageData(vtkImageData* imageData) {
  this->imageData = imageData;
  this->update();
}

#endif
