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

  m_rPressed = false;
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

  // Setup render window
  vtkRenderWindow* renderWindow = this->GetRenderWindow();
  renderWindow->AddRenderer(m_renderer);

  // Set up the interaction
  vtkSmartPointer<vtkInteractorStyleImage> imageStyle =
      vtkSmartPointer<vtkInteractorStyleImage>::New();
  imageStyle->SetInteractionModeToImageSlicing();

  renderWindow->GetInteractor()->SetInteractorStyle(imageStyle);
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
  std::cout << "wheel increment = " << increment << std::endl;

  // move the center point that we are slicing through
  double sliceSpacing = m_imageData->GetSpacing()[2];
  double point[4];
  double center[4];
  point[0] = 0.0;
  point[1] = 0.0;
  point[2] = - sliceSpacing * increment;
  point[3] = 1.0;
  std::cout << "wheel event new point : " << point[0] << "," << point[1] << "," << point[2] << std::endl;
  m_resliceMatrix->MultiplyPoint(point, center);

  std::cout << "wheel event new center : " << center[0] << "," << center[1] << "," << center[2] << std::endl;
  m_resliceMatrix->SetElement(0, 3, center[0]);
  m_resliceMatrix->SetElement(1, 3, center[1]);
  m_resliceMatrix->SetElement(2, 3, center[2]);

  //update this view
  update();

  //emit signal to inform other views the reslice matrix changed
  emit(matrixChanged(m_resliceMatrix));

  event->accept();
}

/**
* Key press event catcher (R key), to enable rotation mode.
*/
void us2DSceneWidget::keyPressEvent(QKeyEvent *event) {
  if(event->key() == Qt::Key_R) {
    m_rPressed = true;
  }
  event->accept();
}

/**
* Key press event catcher (R key), to disable rotation mode.
*/
void us2DSceneWidget::keyReleaseEvent(QKeyEvent *event) {
  if(event->key() == Qt::Key_R) {
    m_rPressed = false;
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

    //get rotations directions from plane orientation
    vpColVector colVecX(4);
    colVecX.data[0] = 0;
    colVecX.data[1] = 0;
    colVecX.data[2] = 0;
    colVecX.data[3] = 0;

    vpThetaUVector tuVecX;
    tuVecX.data[0] = 0;
    tuVecX.data[1] = 0;
    tuVecX.data[2] = 0;
    //when we move along x we rotate around y (z is normal to the view).
    if(abs(dy) < 16) {
      colVecX.data[0] = vpMath::rad(dy*.1);
      colVecX = currentMat*colVecX;
      tuVecX.data[0] = colVecX.data[0];
      tuVecX.data[1] = colVecX.data[1];
      tuVecX.data[2] = colVecX.data[2];
    }

    vpColVector colVecY(4);
    colVecY.data[0] = 0;
    colVecY.data[1] = 0;
    colVecY.data[2] = 0;
    colVecX.data[3] = 0;
    vpThetaUVector tuVecY;
    tuVecY.data[0] = 0;
    tuVecY.data[1] = 0;
    tuVecY.data[2] = 0;

    //when we move along y we rotate around x (z is normal to the view).
    if(abs(dx) < 16) {
      colVecY.data[1] = vpMath::rad(dx*.1);
      colVecY = currentMat*colVecY;

      tuVecY.data[0] = colVecY.data[0];
      tuVecY.data[1] = colVecY.data[1];
      tuVecY.data[2] = colVecY.data[2];
    }
    vpRotationMatrix rotX(tuVecX);
    vpHomogeneousMatrix hRotX;
    hRotX.eye();
    hRotX.buildFrom(vpTranslationVector(0,0,0),rotX);
    vpRotationMatrix rotY(tuVecY);
    vpHomogeneousMatrix hRotY;
    hRotY.eye();
    hRotY.buildFrom(vpTranslationVector(0,0,0),rotY);

    vpHomogeneousMatrix finalMat = (currentMat * hRotX) * hRotY;

    usVTKConverter::convert(finalMat, m_resliceMatrix);

    emit(matrixChanged(m_resliceMatrix));
    update();

    m_lastmouserPosX = event->pos().x();
    m_lastmouserPosY = event->pos().y();
    event->accept();
  }
  else {
    //propagate event to allow colormap change in vtk
    usViewerWidget::mouseMoveEvent(event);
  }
}


/**
* Slot to save the current image displayed in the view.
* @param filename Image Filen name without extention (.png added in this method).
*/
void 	us2DSceneWidget::saveViewSlot() {
  //m_reslice->GetOutput()->Print(std::cout);
    vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  std::string absFileName = us::getDataSetPath() + "/sceenshot.png";
  std::cout << "saving slice in file : " << absFileName;
  writer->SetFileName(absFileName.c_str());
  writer->SetInputConnection(m_reslice->GetOutputPort());
  writer->Write();
}

#endif //USTK_HAVE_VTK_QT
