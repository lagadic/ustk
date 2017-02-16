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
#include <visp3/ustk_core/usImagePostScan2D.h>


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
  //m_renderer->SetLayer(0);
  /*m_polyDataRenderer = vtkRenderer::New();
  m_polyDataRenderer->PreserveDepthBufferOn();
  m_polyDataRenderer->SetLayer(0);*/

  m_table = vtkLookupTable::New();

  m_color = vtkImageMapToColors::New();

  m_actor = vtkImageActor::New();

  m_polydataPlaneContour = vtkPolyData::New();
  m_polyDataPlaneContourMapper = vtkPolyDataMapper::New();
  m_polydataPlaneContourActor= vtkActor::New();

  m_polydataMeshContour = vtkPolyData::New();
  m_polyDataMeshContourMapper = vtkPolyDataMapper::New();
  m_polydataMeshContourActor= vtkActor::New();

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
* Polydata plane contour setter.
* @param polyData The vtk polydata representing the plane contour (image bounds).
*/
void us2DSceneWidget::setPolyDataPlaneContour(vtkPolyData *polyData) {
  m_polydataPlaneContour = polyData;

  //add polygon in scene

  m_polyDataPlaneContourMapper->SetInputData(m_polydataPlaneContour);
  m_polyDataPlaneContourMapper->SetScalarRange(m_polydataPlaneContour->GetScalarRange());

  m_polydataPlaneContourActor->GetProperty()->SetOpacity(1.0);
  m_polydataPlaneContourActor->GetProperty()->SetLighting(0);
  m_polydataPlaneContourActor->GetProperty()->SetLineWidth(1);

  vpHomogeneousMatrix mat;
  usVTKConverter::convert(m_resliceMatrix,mat);
  usVTKConverter::convert(mat.inverse(),m_resliceMatrix);
  m_polydataPlaneContourActor->SetUserMatrix(m_resliceMatrix);


  m_polydataPlaneContourActor->SetMapper(m_polyDataPlaneContourMapper);

  m_renderer->AddActor(m_polydataPlaneContourActor);

}

/**
* Polydata mesh contour setter.
* @param polyData The vtk polydata representing the mesh contour in the plane.
*/
void us2DSceneWidget::setPolyDataMeshContour(vtkPolyData *polyData) {
  m_polydataMeshContour = polyData;


  //add polygon in scene

  m_polyDataMeshContourMapper->SetInputData(m_polydataMeshContour);
  m_polyDataMeshContourMapper->SetScalarRange(m_polydataMeshContour->GetScalarRange());

  m_polydataMeshContourActor->GetProperty()->SetOpacity(1.0);
  m_polydataMeshContourActor->GetProperty()->SetLighting(0);
  m_polydataMeshContourActor->GetProperty()->SetLineWidth(1);
  m_polydataMeshContourActor->GetProperty()->SetColor(1.0,1.0,0.0);

  vpHomogeneousMatrix mat;
  usVTKConverter::convert(m_resliceMatrix,mat);
  usVTKConverter::convert(mat.inverse(),m_resliceMatrix);
  m_polydataMeshContourActor->SetUserMatrix(m_resliceMatrix);

  m_polydataMeshContourActor->SetMapper(m_polyDataMeshContourMapper);

  m_renderer->AddActor(m_polydataMeshContourActor);
}

/**
* Image update slot.
* @param imageData The new vtkImageData to display.
*/
void us2DSceneWidget::updateImageData(vtkImageData* imageData) {
  m_imageData = imageData;
}

/**
* Orientation matrix update slot for vpMatrix.
* @param matrix The new matrix defining rotation and translation in image coordinates system.
*/
void us2DSceneWidget::changeMatrix(vpHomogeneousMatrix matrix) {
  usVTKConverter::convert(matrix,m_resliceMatrix);
  update();
  emit(matrixChanged(m_resliceMatrix));
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

  //update contour polydata
  vtkMatrix4x4* vtkMat = vtkMatrix4x4::New();
  usVTKConverter::convert(Mnew.inverse(),vtkMat);
  m_polydataPlaneContourActor->SetUserMatrix(vtkMat);
  m_polydataMeshContourActor->SetUserMatrix(vtkMat);

  update();
  m_renderer->Render();

  //m_polyDataRenderer->Render();


  //emit signal to inform other views the reslice matrix changed
  emit(matrixChanged(m_resliceMatrix));


  event->accept();
}

/**
* Key press event catcher (H key), to enable rotation mode.
*/
void us2DSceneWidget::keyPressEvent(QKeyEvent *event) {
  if(event->key() == Qt::Key_H) {
    m_rPressed = true;
  }
  event->accept();
}

/**
* Key press event catcher (H key), to disable rotation mode.
*/
void us2DSceneWidget::keyReleaseEvent(QKeyEvent *event) {
  if(event->key() == Qt::Key_H) {
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

    //update contour polydata
    vtkMatrix4x4* vtkMat = vtkMatrix4x4::New();
    usVTKConverter::convert(finalMat.inverse(),vtkMat);
    m_polydataPlaneContourActor->SetUserMatrix(vtkMat);
  m_polydataMeshContourActor->SetUserMatrix(vtkMat);

    emit(matrixChanged(m_resliceMatrix));
    update();
    m_renderer->Render();


  //m_polyDataRenderer->Render();

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
*/
void 	us2DSceneWidget::saveViewSlot() {
    vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  std::string absFileName = us::getDataSetPath() + "/sceenshot.png";
  std::cout << "saving slice in file : " << absFileName << std::endl;
  writer->SetFileName(absFileName.c_str());
  writer->SetInputConnection(m_reslice->GetOutputPort());
  writer->Write();
}

/**
* Getter for current 2D slice.
*/
void 	us2DSceneWidget::getCurrentSlice(usImagePostScan2D<unsigned char> & image2D) {
  //convert current VTK slice to a usImagePostScan2D
  vtkSmartPointer<vtkImageData> vtkImage2D;
  vtkImage2D = m_reslice->GetOutput();

  usVTKConverter::convert(vtkImage2D,image2D);
}

void us2DSceneWidget::setColor(double r,double g,double b) {
  //m_renderer->SetBackground(r,g,b);
  m_polydataPlaneContourActor->GetProperty()->SetColor(r,g,b);
}

#endif //USTK_HAVE_VTK_QT
