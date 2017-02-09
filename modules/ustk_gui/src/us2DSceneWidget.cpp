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


/**
* Constructor.
* @param parent The QWidget parent.
* @param f Qt window flags.
*/
us2DSceneWidget::us2DSceneWidget(QWidget* parent, Qt::WindowFlags f) : usViewerWidget(parent,f) {

  m_imageData = NULL;
  m_resliceMatrix = NULL;

  m_reslice = vtkImageReslice::New();

  m_renderer = vtkRenderer::New();

  m_table = vtkLookupTable::New();

  m_color = vtkImageMapToColors::New();

  m_actor = vtkImageActor::New();

  m_cubeActor = vtkAnnotatedCubeActor::New();

  m_pointPicker = vtkPointPicker::New();

  m_rPressed = false;
  m_pPressed = false;
  m_mousePressed = false;

  //disable tracking to receive only mouse move events if a button is pressed
  setMouseTracking(false);
}

/**
* Paint event catcher.
* @param event The event caught.
*/
void us2DSceneWidget::paintEvent( QPaintEvent* event ) {

  usViewerWidget::paintEvent(event);
}

/**
* Image data getter.
* @return The vtkImageData in wich we are slicing.
*/
vtkImageData* us2DSceneWidget::getImageData() {
  return m_imageData;
}

/**
* Homogeneous matrix getter.
* @return The matrix defining the current slice (position and orientation).
*/
vtkMatrix4x4* us2DSceneWidget::getResliceMatrix() {
  return m_resliceMatrix;
}

/**
* Init method, to call after image and matrix setters. Initializes the vtk workflow to display the image slice.
*/
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
  //m_pointPicker->PickFromListOn();

  // Setup render window
  vtkRenderWindow* renderWindow = this->GetRenderWindow();
  renderWindow->AddRenderer(m_renderer);

  // Set up the interaction
  vtkSmartPointer<vtkInteractorStyleImage> imageStyle =
      vtkSmartPointer<vtkInteractorStyleImage>::New();

  renderWindow->GetInteractor()->SetInteractorStyle(imageStyle);
  imageStyle->EnabledOff();
}

/**
* Image setter.
* @param imageData The vtkImageData to display.
*/
void us2DSceneWidget::setImageData(vtkImageData* imageData) {
  m_imageData = imageData;
}

/**
* Orientation matrix setter.
* @param matrix The vtk matrix to place the reslice plane in the image coordinate system (rotation and translation).
*/
void us2DSceneWidget::setResliceMatrix(vtkMatrix4x4 *matrix) {
  m_resliceMatrix = matrix;
}

/**
* Image update slot.
* @param imageData The new vtkImageData to display.
*/
void us2DSceneWidget::updateImageData(vtkImageData* imageData) {
  m_imageData = imageData;
}

/**
* Orientation matrix update slot.
* @param matrix The new vtk matrix defining rotation and translation in image coordinates system.
*/
void us2DSceneWidget::matrixChangedSlot(vtkMatrix4x4* matrix) {
  m_resliceMatrix = matrix;
}

/**
* Mouse wheel event catcher. Updates the translation along the plane normal.
*/
void us2DSceneWidget::wheelEvent(QWheelEvent *event) {
  int increment = event->delta() / 120;

  // To improve : mean of the 3 spacings according to the plane orientation
  double sliceSpacing = m_imageData->GetSpacing()[2];

  vpTranslationVector tVec;
  tVec.data[0] = 0;
  tVec.data[1] = 0;
  tVec.data[2] = sliceSpacing * increment;

  vpHomogeneousMatrix MTrans;
  MTrans.buildFrom(tVec,vpThetaUVector(0,0,0));

  vpHomogeneousMatrix MCurrrent;
  usVTKConverter::convert(m_resliceMatrix, MCurrrent);

  vpHomogeneousMatrix Mnew = MCurrrent * MTrans;

  usVTKConverter::convert(Mnew,m_resliceMatrix);

  update();

  //emit signal to inform other views the reslice matrix changed
  emit(matrixChanged(m_resliceMatrix));

  event->accept();
}

/**
* Key press event catcher. R key to enable rotation mode, P key to pick a voxel.
*/
void us2DSceneWidget::keyPressEvent(QKeyEvent *event) {
  if(event->key() == Qt::Key_H) {
    m_rPressed = true;
  }
  else if(event->key() == Qt::Key_P) {
    m_pPressed = true;
  }
  event->accept();
}

/**
* Key press event catcher. R key to enable rotation mode, P key to pick a voxel.
*/
void us2DSceneWidget::keyReleaseEvent(QKeyEvent *event) {
  if(event->key() == Qt::Key_H) {
    m_rPressed = false;
  }
  else if(event->key() == Qt::Key_P) {
    m_pPressed = false;
  }
  event->accept();
}

/**
* Mouse move event catcher, used to calculate the rotation to apply on reslice view plane.
*/
void 	us2DSceneWidget::mouseMoveEvent(QMouseEvent * event) {
  if(m_rPressed) {
    int dx = m_lastmouserPosX - event->pos().x();
    int dy = m_lastmouserPosY - event->pos().y();

    vpHomogeneousMatrix currentMat;
    usVTKConverter::convert(m_resliceMatrix, currentMat);

    vpThetaUVector tuVec;

    //when we move along x we rotate around y (z is normal to the view).
    if(abs(dx) < 16) {
      tuVec.data[1] = vpMath::rad(dx*.1);
    }
    //when we move along y we rotate around x.
    if(abs(dy) < 16) {
      tuVec.data[0] = vpMath::rad(dy*.1);
    }

    vpHomogeneousMatrix MRot;
    MRot.buildFrom(vpTranslationVector(0,0,0),tuVec);

    vpHomogeneousMatrix finalMat = currentMat * MRot;

    usVTKConverter::convert(finalMat, m_resliceMatrix);

    emit(matrixChanged(m_resliceMatrix));
    update();

    m_lastmouserPosX = event->pos().x();
    m_lastmouserPosY = event->pos().y();
    event->accept();
  }
  else {
    //propagate event to allow colormap change in vtk
    //usViewerWidget::mouseMoveEvent(event);
  }
}


/**
* Slot to save the current image displayed in the view.
* @param filename Image Filen name without extention (.png added in this method).
*/
void 	us2DSceneWidget::saveViewSlot() {
    vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  std::string absFileName = us::getDataSetPath() + "/sceenshot.png";
  std::cout << "saving slice in file : " << absFileName;
  writer->SetFileName(absFileName.c_str());
  writer->SetInputConnection(m_reslice->GetOutputPort());
  writer->Write();
}

void us2DSceneWidget::mousePressEvent(QMouseEvent *event) {
if(m_pPressed) {
  int x = event->pos().x();
  int y = event->pos().y();
  std::cout << "Pick (x,y) = (" << x << "," << y << ")" << std::endl;




  m_pointPicker->Pick(x,y,0,m_renderer);
  std::cout << "vtk matrices for view zoom/translatons" << std::endl;
  m_renderer->Print(std::cout);

  double p[3];
  m_pointPicker->GetPickPosition(p);
  std::cout << "Picked in plane coords = " << p[0] << " " << p[1] << " " << p[2]  << std::endl;
  vpTranslationVector picked;
  picked.data[0] = p[0];
  picked.data[1] = p[1];
  picked.data[2] = p[2];

  vpHomogeneousMatrix Mcurrent;
  usVTKConverter::convert(m_resliceMatrix,Mcurrent);

  vpTranslationVector tVec;
  Mcurrent.extract(tVec);

  //picked = picked - tVec;

  picked = Mcurrent.inverse() * picked;
  //picked = Mcurrent * picked;

  std::cout << "Picked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;
  }
  //usViewerWidget::mousePressEvent(event);
}


void us2DSceneWidget::setColor(double r,double g,double b) {
  m_renderer->SetBackground(r,g,b);
}

#endif //USTK_HAVE_VTK_QT
