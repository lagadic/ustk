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
#include <visp3/core/vpRotationMatrix.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_gui/usVTKConverter.h>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkDistanceWidget.h>
#include <vtkResliceImageViewerMeasurements.h>
#include <vtkResliceImageViewer.h>
#include <vtkImagePlaneWidget.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkImageResliceMapper.h>
#include <vtkArrowSource.h>
#include <vtkMatrix4x4.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkCubeSource.h>
#include <vtkPolyLine.h>
#include <vtkCellArray.h>
#include <vtkCutter.h>

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

  //add color spheres to planes
  vtkSphereSource* sphereSource = vtkSphereSource::New();
  sphereSource->SetRadius(0.01);
  sphereSource->Update();

  vtkPolyDataMapper* sphereMPapper = vtkPolyDataMapper::New();
  sphereMPapper->SetInputConnection(sphereSource->GetOutputPort());
  sphereActor = vtkActor::New();
  sphereActor->SetMapper(sphereMPapper);
  sphereActor->GetProperty()->SetColor(1.0, 0.0, 0.0);

  cellPicker = vtkCellPicker::New();
  cellPicker->SetTolerance(0.0005);
  m_pPressed = false;
}

/**
* Qt paint event overload if needed to update Qt widget
* @param event QPaintEvent.
*/
void us3DSceneWidget::paintEvent( QPaintEvent* event )
{
  usViewerWidget::paintEvent( event );
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

  sphereActor->SetPosition(plane1->GetOrigin());

  //Define image bounding box
  double bounds[6];
  imageData->GetBounds(bounds);
    // Create a cube.
  vtkSmartPointer<vtkCubeSource> cubeSource =
    vtkSmartPointer<vtkCubeSource>::New();
    cubeSource->SetBounds(bounds);

  //find intersections between planes and bounding box to draw colored lines to identify each plane
  //Plane border 1
  vtkSmartPointer<vtkCutter> cutter1 =
    vtkSmartPointer<vtkCutter>::New();
  cutter1->SetCutFunction(plane1);
  cutter1->SetInputConnection(cubeSource->GetOutputPort());
  cutter1->Update();
  vtkSmartPointer<vtkPolyDataMapper> cutterMapper1 =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  cutterMapper1->SetInputConnection( cutter1->GetOutputPort());
  vtkSmartPointer<vtkActor> planeBorder1 =
    vtkSmartPointer<vtkActor>::New();
  planeBorder1->GetProperty()->SetColor(1.0,0,0);
  planeBorder1->GetProperty()->SetOpacity(1.0);
  planeBorder1->GetProperty()->SetLighting(0);
  planeBorder1->GetProperty()->SetLineWidth(1);
  planeBorder1->SetMapper(cutterMapper1);

  //Plane border 2
  vtkSmartPointer<vtkCutter> cutter2 =
    vtkSmartPointer<vtkCutter>::New();
  cutter2->SetCutFunction(plane2);
  cutter2->SetInputConnection(cubeSource->GetOutputPort());
  cutter2->Update();
  vtkSmartPointer<vtkPolyDataMapper> cutterMapper2 =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  cutterMapper2->SetInputConnection( cutter2->GetOutputPort());
  vtkSmartPointer<vtkActor> planeBorder2 =
    vtkSmartPointer<vtkActor>::New();
  planeBorder2->GetProperty()->SetColor(0,1.0,0);
  planeBorder2->GetProperty()->SetOpacity(1.0);
  planeBorder2->GetProperty()->SetLighting(0);
  planeBorder2->GetProperty()->SetLineWidth(3);
  planeBorder2->SetMapper(cutterMapper2);

  //Plane border 3
  vtkSmartPointer<vtkCutter> cutter3 =
    vtkSmartPointer<vtkCutter>::New();
  cutter3->SetCutFunction(plane3);
  cutter3->SetInputConnection(cubeSource->GetOutputPort());
  cutter3->Update();
  vtkSmartPointer<vtkPolyDataMapper> cutterMapper3 =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  cutterMapper3->SetInputConnection( cutter3->GetOutputPort());
  vtkSmartPointer<vtkActor> planeBorder3 =
    vtkSmartPointer<vtkActor>::New();
  planeBorder3->GetProperty()->SetColor(0,0,1.0);
  planeBorder3->GetProperty()->SetOpacity(1.0);
  planeBorder3->GetProperty()->SetLighting(0);
  planeBorder3->GetProperty()->SetLineWidth(3);
  planeBorder3->SetMapper(cutterMapper3);

  //add axes in scene
  m_axesActor = vtkSmartPointer<vtkAxesActor>::New();

  //arrows of 1cm
  m_axesActor->SetXAxisLabelText("U");
  m_axesActor->SetYAxisLabelText("V");
  m_axesActor->SetZAxisLabelText("W");
  m_axesActor->SetTotalLength(0.01,0.01,0.01);

  // Setup renderers
  renderer = vtkRenderer::New();
  renderer->AddActor(m_axesActor);
  renderer->AddActor(imageSlice1);
  renderer->AddActor(imageSlice2);
  renderer->AddActor(imageSlice3);
  renderer->AddActor(planeBorder1);
  renderer->AddActor(planeBorder2);
  renderer->AddActor(planeBorder3);
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
* Plane 1 getter.
* @return Pointer on first vtkPlane.
*/
vtkPlane* us3DSceneWidget::getPlane1() {
  return this->plane1;
}

/**
* Plane 2 getter.
* @return Pointer on second vtkPlane.
*/
vtkPlane* us3DSceneWidget::getPlane2() {
  return this->plane2;
}

/**
* Plane 3 getter.
* @return  Pointer on third vtkPlane.
*/
vtkPlane* us3DSceneWidget::getPlane3() {
  return this->plane3;
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
* @param imageData Pointer to new image to display
*/
void us3DSceneWidget::updateImageData(vtkImageData* imageData) {
  this->imageData = imageData;
  this->update();
}

/**
* Slot called to update plane 1 with new RT matrix
* @param matrix Pointer to new orientation matrix.
*/
void us3DSceneWidget::updateMatrix1(vtkMatrix4x4* matrix) {

  if(plane1 == NULL) {
    plane1 = vtkPlane::New();
    //rotation (valid only for init at X normal)
    plane1->SetNormal(1,0,0);
  }
  //Translation
  double origin[3];
  origin[0] = matrix->Element[0][3];
  origin[1] = matrix->Element[1][3];
  origin[2] = matrix->Element[2][3];
  plane1->SetOrigin(origin);

  //Rotation
  vpRotationMatrix rMat;
  vpHomogeneousMatrix hMat;
  usVTKConverter::convert(matrix,hMat);
  hMat.extract(rMat);

  vpColVector normal(3);
  normal.data[0] = 0;
  normal.data[1] = 0;
  normal.data[2] = 1;

  normal = rMat * normal;

  plane1->SetNormal(normal.data[0], normal.data[1], normal.data[2]);

  sphereActor->SetPosition(imageSlice1->GetMinXBound(),
                           imageSlice1->GetMinYBound(),
                           imageSlice1->GetMinZBound());

  this->update();
}

/**
* Slot called to update plane 2 with new RT matrix
* @param matrix Pointer to new orientation matrix.
*/
void us3DSceneWidget::updateMatrix2(vtkMatrix4x4* matrix) {

  if(plane2 == NULL) {
    plane2 = vtkPlane::New();
    //rotation (valid only for init at Y normal)
    plane2->SetNormal(0,1,0);
  }
  //Translation
  double origin[3];
  origin[0] = matrix->Element[0][3];
  origin[1] = matrix->Element[1][3];
  origin[2] = matrix->Element[2][3];
  plane2->SetOrigin(origin);

  //Rotation
  vpRotationMatrix rMat;
  vpHomogeneousMatrix hMat;
  usVTKConverter::convert(matrix,hMat);
  hMat.extract(rMat);

  vpColVector normal(3);
  normal.data[0] = 0;
  normal.data[1] = 0;
  normal.data[2] = 1;

  normal = rMat * normal;

  plane2->SetNormal(normal.data[0], normal.data[1], normal.data[2]);

  this->update();
}

/**
* Slot called to update plane 3 with new RT matrix
* @param matrix Pointer to new orientation matrix.
*/
void us3DSceneWidget::updateMatrix3(vtkMatrix4x4* matrix) {

  if(plane3 == NULL) {
    plane3 = vtkPlane::New();
    //rotation (valid only for init at Z normal)
    plane3->SetNormal(0,0,1);
  }

  //Translation
  double origin[3];
  origin[0] = matrix->Element[0][3];
  origin[1] = matrix->Element[1][3];
  origin[2] = matrix->Element[2][3];
  plane3->SetOrigin(origin);

  //Rotation
  vpRotationMatrix rMat;
  vpHomogeneousMatrix hMat;
  usVTKConverter::convert(matrix,hMat);
  hMat.extract(rMat);

  vpColVector normal(3);
  normal.data[0] = 0;
  normal.data[1] = 0;
  normal.data[2] = 1;

  normal = rMat * normal;

  plane3->SetNormal(normal.data[0], normal.data[1], normal.data[2]);

  this->update();
}

/**
* Key press event catcher. P key to pick a voxel in a plane displayed in the scene.
*/
void us3DSceneWidget::keyPressEvent(QKeyEvent *event) {
  //for picking operations
  if(event->key() == Qt::Key_P) {
    m_pPressed = true;
  }
  event->accept();
}

/**
* Key release event catcher, to disable key pressed booleans.
*/
void us3DSceneWidget::keyReleaseEvent(QKeyEvent *event) {
  if(event->key() == Qt::Key_P) {
    m_pPressed = false;
  }
  event->accept();
}

void us3DSceneWidget::mousePressEvent(QMouseEvent *event) {
if(m_pPressed) {
  int x = event->pos().x();
  int y = event->pos().y();
  std::cout << "Pick (x,y) = (" << x << "," << y << ")" << std::endl;

  cellPicker->Pick(x,y,0,renderer);

  double p[3];
  cellPicker->GetPickPosition(p);
  std::cout << "Picked in plane coords = " << p[0] << " " << p[1] << " " << p[2]  << std::endl;
  /*vpTranslationVector picked;
  picked.data[0] = p[0];
  picked.data[1] = p[1];
  picked.data[2] = p[2];

  vpHomogeneousMatrix Mcurrent;
  usVTKConverter::convert(m_resliceMatrix,Mcurrent);

  vpTranslationVector tVec;
  Mcurrent.extract(tVec);

  //picked = picked - tVec;

  //picked = Mcurrent.inverse() * picked;
  //picked = Mcurrent * picked;

  std::cout << "Picked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;*/
  }
  usViewerWidget::mousePressEvent(event);
}
#endif
