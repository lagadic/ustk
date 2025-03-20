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
#include <visp3/ustk_core/usConfig.h>
#ifdef USTK_HAVE_VTK_QT
#include <visp3/ustk_gui/usViewerWidget.h>

// USTK includes
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_gui/us2DSceneWidget.h>

#include <QWheelEvent>

/**
* Constructor.
* @param parent The QWidget parent.
* @param f Qt window flags.
*/
us2DSceneWidget::us2DSceneWidget(QWidget *parent, Qt::WindowFlags f) : usViewerWidget(parent, f)
{

  m_imageData = NULL;
  m_resliceMatrix = NULL;

  m_reslice = vtkImageReslice::New();

  m_renderer = vtkRenderer::New();

  m_table = vtkLookupTable::New();

  m_color = vtkImageMapToColors::New();

  m_actor = vtkImageActor::New();

  m_polydataPlaneContour = vtkPolyData::New();
  m_polyDataPlaneContourMapper = vtkPolyDataMapper::New();
  m_polydataPlaneContourActor = vtkActor::New();

  m_polydataMeshContour = vtkPolyData::New();
  m_polyDataMeshContourMapper = vtkPolyDataMapper::New();
  m_polydataMeshContourActor = vtkActor::New();

  // Picker to pick pixels
  m_propPicker = vtkPropPicker::New();

  m_rPressed = false;
  m_pPressed = false;
  m_mousePressed = false;

  // disable tracking to receive only mouse move events if a button is pressed
  setMouseTracking(false);

  m_pickingState = false;
}

/**
* Paint event catcher.
* @param event The event caught.
*/
void us2DSceneWidget::paintEvent(QPaintEvent *event) { usViewerWidget::paintEvent(event); }

/**
* Image data getter.
* @return The vtkImageData in wich we are slicing.
*/
vtkImageData *us2DSceneWidget::getImageData() { return m_imageData; }

/**
* Homogeneous matrix getter.
* @return The matrix defining the current slice (position and orientation).
*/
vtkMatrix4x4 *us2DSceneWidget::getResliceMatrix() { return m_resliceMatrix; }

/**
* Init method, to call after image and matrix setters. Initializes the vtk workflow to display the image slice.
*/
void us2DSceneWidget::init()
{

  // verify imageData and reslice matrix are set
  if (m_imageData == NULL)
    throw(vpException(vpException::fatalError, "No imageData provided in us2DSceneWidget"));

  if (m_resliceMatrix == NULL)
    throw(vpException(vpException::fatalError, "No reslice matrix provided in us2DSceneWidget"));

  m_reslice->SetInputData(m_imageData);
  m_reslice->SetOutputDimensionality(2);
  m_reslice->SetResliceAxes(m_resliceMatrix);
  m_reslice->SetInterpolationModeToLinear();

  // To ensure no part of the image will be cropped
  m_reslice->AutoCropOutputOn();

  // Create a greyscale lookup table
  m_table->SetRange(0, 255);             // image intensity range
  m_table->SetValueRange(0.0, 1.0);      // from black to white
  m_table->SetSaturationRange(0.0, 0.0); // no color saturation
  m_table->SetRampToLinear();
  m_table->Build();

  // Map the image through the lookup table
  m_color->SetLookupTable(m_table);
  m_color->SetInputConnection(m_reslice->GetOutputPort());

  // Display the image
  m_actor->GetMapper()->SetInputConnection(m_color->GetOutputPort());
  m_actor->SetOpacity(0.7);

  m_renderer->AddActor(m_actor);

  // Setup render window
#if USTK_HAVE_VTK_VERSION < 0x090000
  vtkRenderWindow *renderWindow = this->GetRenderWindow();
#else
  vtkRenderWindow *renderWindow = this->renderWindow();
#endif
  renderWindow->AddRenderer(m_renderer);

  // Set up the interaction
  vtkSmartPointer<vtkInteractorStyleImage> imageStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
  imageStyle->SetInteractionModeToImageSlicing();
  renderWindow->GetInteractor()->SetInteractorStyle(imageStyle);
  // imageStyle->EnabledOff();

  // picker
  m_propPicker->PickFromListOn();

  // Give the picker a prop to pick
  m_propPicker->AddPickList(m_actor);
}

/**
* Image setter.
* @param imageData The vtkImageData to display.
*/
void us2DSceneWidget::setImageData(vtkImageData *imageData) { m_imageData = imageData; }

/**
* Orientation matrix setter.
* @param matrix The vtk matrix to place the reslice plane in the image coordinate system (rotation and translation).
*/
void us2DSceneWidget::setResliceMatrix(vtkMatrix4x4 *matrix) { m_resliceMatrix = matrix; }

/**
* Polydata plane contour setter.
* @param polyData The vtk polydata representing the plane contour (image bounds).
*/
void us2DSceneWidget::setPolyDataPlaneContour(vtkPolyData *polyData)
{
  m_polydataPlaneContour = polyData;

  // add polygon in scene

  m_polyDataPlaneContourMapper->SetInputData(m_polydataPlaneContour);
  m_polyDataPlaneContourMapper->SetScalarRange(m_polydataPlaneContour->GetScalarRange());

  m_polydataPlaneContourActor->GetProperty()->SetOpacity(1.0);
  m_polydataPlaneContourActor->GetProperty()->SetLighting(0);
  m_polydataPlaneContourActor->GetProperty()->SetLineWidth(1);
  m_polydataPlaneContourActor->GetProperty()->SetOpacity(1.0);

  vpHomogeneousMatrix mat;
  usVTKConverter::convert(m_resliceMatrix, mat);
  vtkMatrix4x4 *matrix = vtkMatrix4x4::New();
  usVTKConverter::convert(mat.inverse(), matrix);
  m_polydataPlaneContourActor->SetUserMatrix(matrix);

  m_polydataPlaneContourActor->SetMapper(m_polyDataPlaneContourMapper);

  m_renderer->AddActor(m_polydataPlaneContourActor);
}

/**
* Polydata mesh contour setter.
* @param polyData The vtk polydata representing the mesh contour in the plane.
*/
void us2DSceneWidget::setPolyDataMeshContour(vtkPolyData *polyData)
{
  m_polydataMeshContour = polyData;

  // add polygon in scene
  m_polyDataMeshContourMapper->SetInputData(m_polydataMeshContour);
  m_polyDataMeshContourMapper->SetScalarRange(m_polydataMeshContour->GetScalarRange());

  m_polydataMeshContourActor->GetProperty()->SetOpacity(1.0);
  m_polydataMeshContourActor->GetProperty()->SetLighting(0);
  m_polydataMeshContourActor->GetProperty()->SetLineWidth(1);
  m_polydataMeshContourActor->GetProperty()->SetColor(1.0, 1.0, 0.0);
  m_polydataMeshContourActor->GetProperty()->SetOpacity(1.0);

  vpHomogeneousMatrix mat;
  usVTKConverter::convert(m_resliceMatrix, mat);
  vtkMatrix4x4 *matrix = vtkMatrix4x4::New();
  usVTKConverter::convert(mat.inverse(), matrix);
  m_polydataMeshContourActor->SetUserMatrix(matrix);

  m_polydataMeshContourActor->SetMapper(m_polyDataMeshContourMapper);

  m_renderer->AddActor(m_polydataMeshContourActor);
}

/**
* Image update slot.
* @param imageData The new vtkImageData to display.
*/
void us2DSceneWidget::updateImageData(vtkImageData *imageData)
{

  // remove actor from view to avoid rendering errors during data switch
  m_renderer->RemoveActor(m_actor);

  // update internal pointer
  m_imageData = imageData;
  // update image reslice with the new image data
  m_reslice->SetInputData(imageData);

  // add the actor (modified with vtk because m_reslice input data has changed)
  m_renderer->AddActor(m_actor);

  // render the view with the new actor
#if USTK_HAVE_VTK_VERSION < 0x090000
  GetRenderWindow()->Render();
#else
  renderWindow()->Render();
#endif
}

/**
* Orientation matrix update slot for vpMatrix.
* @param matrix The new matrix defining rotation and translation in image coordinates system.
*/
void us2DSceneWidget::changeMatrix(vpHomogeneousMatrix matrix)
{
  usVTKConverter::convert(matrix, m_resliceMatrix);
  update();
  emit(matrixChanged(m_resliceMatrix));
}

/**
* Mouse wheel event catcher. Updates the translation along the plane normal.
*/
void us2DSceneWidget::wheelEvent(QWheelEvent *event)
{
#if USTK_HAVE_VTK_VERSION < 0x090000
  int increment = event->delta() / 120;
#else
  int increment = event->angleDelta().y() / 120;
#endif

  // To improve : mean of the 3 spacings according to the plane orientation
  double sliceSpacing = m_imageData->GetSpacing()[2];

  vpTranslationVector tVec;
  tVec.data[0] = 0;
  tVec.data[1] = 0;
  tVec.data[2] = sliceSpacing * increment;

  vpHomogeneousMatrix MTrans;
  MTrans.buildFrom(tVec, vpThetaUVector(0, 0, 0));

  vpHomogeneousMatrix MCurrrent;
  usVTKConverter::convert(m_resliceMatrix, MCurrrent);

  vpHomogeneousMatrix Mnew = MCurrrent * MTrans;

  usVTKConverter::convert(Mnew, m_resliceMatrix);

  // update contour polydata
  vtkMatrix4x4 *vtkMat = vtkMatrix4x4::New();
  usVTKConverter::convert(Mnew.inverse(), vtkMat);
  m_polydataPlaneContourActor->SetUserMatrix(vtkMat);
  m_polydataMeshContourActor->SetUserMatrix(vtkMat);

  update();
  m_renderer->Render();

  // emit signal to inform other views the reslice matrix changed
  emit(matrixChanged(m_resliceMatrix));

  event->accept();
}

/**
* Key press event catcher. H key to enable rotation mode, P key to pick a voxel.
*/
void us2DSceneWidget::keyPressEvent(QKeyEvent *event)
{
  if (event->key() == Qt::Key_H) {
    m_rPressed = true;
  }
  else if (event->key() == Qt::Key_P) {
    m_pPressed = true;
  }
  event->accept();
}

/**
* Key press event catcher. H key to enable rotation mode, P key to pick a voxel.
*/
void us2DSceneWidget::keyReleaseEvent(QKeyEvent *event)
{
  if (event->key() == Qt::Key_H) {
    m_rPressed = false;
  }
  else if (event->key() == Qt::Key_P) {
    m_pPressed = false;
  }
  event->accept();
}

/**
* Mouse move event catcher, used to calculate the rotation to apply on reslice view plane.
*/
void us2DSceneWidget::mouseMoveEvent(QMouseEvent *event)
{
  if (m_rPressed) {
    int dx = m_lastmouserPosX - event->pos().x();
    int dy = m_lastmouserPosY - event->pos().y();

    vpHomogeneousMatrix currentMat;
    usVTKConverter::convert(m_resliceMatrix, currentMat);

    vpThetaUVector tuVec;

    // when we move along x we rotate around y (z is normal to the view).
    if (abs(dx) < 16) {
      tuVec.data[1] = vpMath::rad(dx * .1);
    }
    // when we move along y we rotate around x.
    if (abs(dy) < 16) {
      tuVec.data[0] = vpMath::rad(dy * .1);
    }

    vpHomogeneousMatrix MRot;
    MRot.buildFrom(vpTranslationVector(0, 0, 0), tuVec);

    vpHomogeneousMatrix finalMat = currentMat * MRot;

    usVTKConverter::convert(finalMat, m_resliceMatrix);

    // update contour polydata
    vtkMatrix4x4 *vtkMat = vtkMatrix4x4::New();
    usVTKConverter::convert(finalMat.inverse(), vtkMat);
    m_polydataPlaneContourActor->SetUserMatrix(vtkMat);
    m_polydataMeshContourActor->SetUserMatrix(vtkMat);

    emit(matrixChanged(m_resliceMatrix));
    update();
    m_renderer->Render();

    m_lastmouserPosX = event->pos().x();
    m_lastmouserPosY = event->pos().y();
    event->accept();
  }
  else {
 // propagate event to allow colormap change in vtk
    usViewerWidget::mouseMoveEvent(event);
  }
}

/**
* Slot to save the current image displayed in the view.
*/
void us2DSceneWidget::saveViewSlot()
{
  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  std::string absFileName = us::getDataSetPath() + "/sceenshot.png";
  std::cout << "saving slice in file : " << absFileName << std::endl;
  writer->SetFileName(absFileName.c_str());
  writer->SetInputConnection(m_reslice->GetOutputPort());
  writer->Write();
}

/**
* Getter for current 2D slice.
* @param [out] image2D Slice to extract.
*/
void us2DSceneWidget::getCurrentSlice(usImagePostScan2D<unsigned char> &image2D)
{
  // Render to update the view and avoid getting an empty 2D image at reslice output
#if USTK_HAVE_VTK_VERSION < 0x090000
  this->GetRenderWindow()->Render();
#else
  renderWindow()->Render();
#endif

  // convert current VTK slice to a usImagePostScan2D
  vtkSmartPointer<vtkImageData> vtkImage2D;
  vtkImage2D = m_reslice->GetOutput();
  usVTKConverter::convert(vtkImage2D, image2D);
}

/**
* Mouse press event filter. Used to pick voxels.
*/
void us2DSceneWidget::mousePressEvent(QMouseEvent *event)
{
  if (m_pPressed || m_pickingState) {
    int x = event->pos().x();
    int y = this->height() - event->pos().y(); // change for VTK window coordinate system, Y axis is inverted

    m_propPicker->Pick(x, y, 0.0, m_renderer);

    if (m_propPicker->GetPath()) {
      double p[3];
      m_propPicker->GetPickPosition(p);

      // transform in 3D image coordinate system
      vpHomogeneousMatrix MCurrrent;
      usVTKConverter::convert(m_resliceMatrix, MCurrrent);

      vpColVector vector(4);
      vector.data[0] = p[0];
      vector.data[1] = p[1];
      vector.data[2] = p[2];
      vector.data[3] = 1;

      vector = MCurrrent * vector;
      std::cout << "Picked value = " << vector.data[0] << " " << vector.data[1] << " " << vector.data[2] << std::endl;
      m_pickedVoxel = vector;
      emit(voxelPicked(vector));
    }
    else
      std::cout << "Pick out of image" << std::endl;
  }
  usViewerWidget::mousePressEvent(event);
}

void us2DSceneWidget::setColor(double r, double g, double b)
{
  // m_renderer->SetBackground(r,g,b);
  m_polydataPlaneContourActor->GetProperty()->SetColor(r, g, b);
}

/**
* Draw a red line between (u1,v1,w1) and (u2,v2,w2) over the image.
*/
void us2DSceneWidget::drawLine(double u1, double v1, double w1, double u2, double v2, double w2)
{
  // Setup points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(u1, v1, w1);
  points->InsertNextPoint(u2, v2, w2);

  // create a line between each pair of points
  vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
  line0->GetPointIds()->SetId(0, 0);
  line0->GetPointIds()->SetId(1, 1);

  // create a cell array to store the line in
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(line0);

  // create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  // add the points and lines to the polydata
  polydata->SetPoints(points);
  polydata->SetLines(lines);

  // add polygon in scene
  vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  lineMapper->SetInputData(polydata);
  lineMapper->SetScalarRange(polydata->GetScalarRange());

  vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
  lineActor->GetProperty()->SetOpacity(1.0);
  lineActor->GetProperty()->SetLighting(0);
  lineActor->GetProperty()->SetLineWidth(1);
  lineActor->GetProperty()->SetColor(1.0, 0.0, 0.0);
  lineActor->GetProperty()->SetOpacity(1.0);
  lineActor->SetMapper(lineMapper);

  vpHomogeneousMatrix mat;
  usVTKConverter::convert(m_resliceMatrix, mat);
  vtkMatrix4x4 *matrix = vtkMatrix4x4::New();
  usVTKConverter::convert(mat.inverse(), matrix);
  lineActor->SetUserMatrix(matrix);

  m_renderer->AddActor(lineActor);
#if USTK_HAVE_VTK_VERSION < 0x090000
  GetRenderWindow()->Render();
#else
  renderWindow()->Render();
#endif
}

/**
* Slot used to recompute the view (if something shared changed in another view)
*/
#if USTK_HAVE_VTK_VERSION < 0x090000
void us2DSceneWidget::updateView() { GetRenderWindow()->Render(); }
#else
void us2DSceneWidget::updateView() { renderWindow()->Render(); }
#endif

/**
* Blocking getClick method : waits for user to pick a voxel, and return the voxel coordinates in (u,v,w) coordinates
* system.
*/
void us2DSceneWidget::getClick(vpColVector &vec)
{
  m_pickingState = true;
  QEventLoop loop;
  connect(this, SIGNAL(voxelPicked(vpColVector)), &loop, SLOT(quit()));
  loop.exec();

  vec = m_pickedVoxel;
  m_pickingState = false;
}

#endif // USTK_HAVE_VTK_QT
