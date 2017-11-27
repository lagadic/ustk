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
* @file usVirtualNeedle.cpp
* @brief Graphical main window containing 4 vtk views.
*/

#include "usVirtualNeedle.h"

#ifdef USTK_HAVE_VTK_QT

/**
* Constructor.
*/
usVirtualNeedle::usVirtualNeedle(QWidget *parent, Qt::WindowFlags f) : usViewerWidget(parent, f)
{
  // POLYDATA
  m_meshPolyData = vtkPolyData::New();
  // Create a cylinder for the needle
  vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
  cylinderSource->SetCenter(0, 0, 0);
  cylinderSource->SetRadius(0.0001);
  cylinderSource->SetHeight(0.01);
  cylinderSource->SetResolution(100);
  cylinderSource->Update();
  m_meshNeedle = cylinderSource->GetOutput();

  // MAPPERS - ACTORS
  vtkSmartPointer<vtkPolyDataMapper> cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  cylinderMapper->SetInputConnection(cylinderSource->GetOutputPort());
  m_needleActor = vtkSmartPointer<vtkActor>::New();
  m_needleActor->GetProperty()->SetColor(1.0, 0, 0); // red
  m_needleActor->GetProperty()->SetOpacity(1.0);
  m_needleActor->SetMapper(cylinderMapper);

  // Axes
  m_axesActor = vtkSmartPointer<vtkAxesActor>::New();

  // arrows of 1cm
  m_axesActor->SetXAxisLabelText("X");
  m_axesActor->SetYAxisLabelText("Y");
  m_axesActor->SetZAxisLabelText("Z");
  m_axesActor->SetTotalLength(0.01, 0.01, 0.01); // 10cm each

  // Setup renderer
  renderer = vtkRenderer::New();
  renderer->AddActor(m_axesActor);
  renderer->AddActor(m_needleActor);
  renderer->SetBackground(0.5, 0.5, 0.5);
  renderer->ResetCamera();

  // Setup render window
  vtkRenderWindow *renderWindow = this->GetRenderWindow();
  renderWindow->AddRenderer(renderer);
}
/*
usVirtualNeedle::~usVirtualNeedle()
{

}*/

/**
* Qt paint event overload if needed to update Qt widget
* @param event QPaintEvent.
*/
void usVirtualNeedle::paintEvent(QPaintEvent *event) { usViewerWidget::paintEvent(event); }

/**
* Qt key press event catcher, used to move the virtual needle in the scene.
* @param event QKeyEvent.
*/
void usVirtualNeedle::keyPressEvent(QKeyEvent *event)
{
  if (event->key() == Qt::Key_Left) {
    vpHomogeneousMatrix transform;
    transform.eye();
    transform[0][3] = 0.001; // add 1mm on X axis
    updateNeedlePosition(transform);
  } else if (event->key() == Qt::Key_Right) {
    vpHomogeneousMatrix transform;
    transform.eye();
    transform[0][3] = -0.001; // minus 1mm on X axis
    updateNeedlePosition(transform);
  } else if (event->key() == Qt::Key_Up) {
    vpHomogeneousMatrix transform;
    transform.eye();
    transform[1][3] = 0.001; // add 1mm on Y axis
    updateNeedlePosition(transform);
  } else if (event->key() == Qt::Key_Down) {
    vpHomogeneousMatrix transform;
    transform.eye();
    transform[1][3] = -0.001; // remove 1mm on Y axis
    updateNeedlePosition(transform);
  } else if (event->key() == Qt::Key_PageUp) {
    vpHomogeneousMatrix transform;
    transform.eye();
    transform[2][3] = 0.001; // remove 1mm on Z axis
    updateNeedlePosition(transform);
  } else if (event->key() == Qt::Key_PageDown) {
    vpHomogeneousMatrix transform;
    transform.eye();
    transform[2][3] = -0.001; // remove 1mm on Z axis
    updateNeedlePosition(transform);
  } else if (event->key() == Qt::Key_Space) { // move first point of the mesh of 1mm along Z
    double *point1 = m_meshPolyData->GetPoints()->GetPoint(0);
    point1[2] += 0.001;
    m_meshPolyData->GetPoints()->SetPoint(0, point1);
    m_meshPolyData->GetPoints()->Modified();
    this->GetRenderWindow()->Render();
  } else if (event->key() == Qt::Key_0) { // move first point of the mesh of - 1mm along Z
    double *point1 = m_meshPolyData->GetPoints()->GetPoint(0);
    point1[2] -= 0.001;
    m_meshPolyData->GetPoints()->SetPoint(0, point1);
    m_meshPolyData->GetPoints()->Modified();
    this->GetRenderWindow()->Render();
  } else {
    usViewerWidget::keyPressEvent(event);
  }
}

/**
* Setter for the mesh to introcuce in the scene.
* @param mesh The mesh, under vtkPolydataFormat.
*/
void usVirtualNeedle::setMeshInScene(vtkPolyData *mesh)
{
  m_meshPolyData = mesh;

  vtkSmartPointer<vtkPolyDataMapper> meshMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  meshMapper->SetInputData(mesh);
  m_meshActor = vtkSmartPointer<vtkActor>::New();
  m_meshActor->GetProperty()->SetColor(0, 0, 1.0); // blue
  m_meshActor->SetMapper(meshMapper);

  renderer->AddActor(m_meshActor);
}

/**
* Slot to call every time you want to update the virtual needle positon.
* @param transform The homogeneous matrix of the needle movement since last call (can be considered as a "delta"
* movement).
*/
void usVirtualNeedle::updateNeedlePosition(vpHomogeneousMatrix transform)
{

  if (m_needleActor->GetUserMatrix() == NULL) { // init case
    vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    usVTKConverter::convert(transform, vtkMatrix);
    m_needleActor->SetUserMatrix(vtkMatrix);
  } else {
    // get current matrix
    vpHomogeneousMatrix currentTransform;
    currentTransform.eye();
    usVTKConverter::convert(m_needleActor->GetUserMatrix(), currentTransform);

    // Conversion, taking in account current transform of the needle
    vpHomogeneousMatrix newTransform = currentTransform * transform;

    // set new position
    vtkMatrix4x4 *vtkNewtransform = vtkMatrix4x4::New();
    usVTKConverter::convert(newTransform, vtkNewtransform);

    m_needleActor->SetUserMatrix(vtkNewtransform);
    this->GetRenderWindow()->Render();
  }
}

/**
* Point set of the mesh getter. To update a point position: call GetPoint(int ptIndex), update the coordinates, and then
* call SetPoint(ptIndex,yourPoint) followed by Modified() to update vtk object.
* @return The points of the mesh (pointer);
*/
vtkPoints *usVirtualNeedle::getMeshPoints() { return m_meshPolyData->GetPoints(); }

/**
* To render the scene, after some updates done on objects.
*/
void usVirtualNeedle::render() { this->GetRenderWindow()->Render(); }

#endif
