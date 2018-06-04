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
* @file usMeshDeformation.cpp
*/

#include "usMeshDeformation.h"

#ifdef USTK_HAVE_VTK_QT

/**
* Constructor.
*/
usMeshDeformation::usMeshDeformation(QWidget *parent, Qt::WindowFlags f) : usViewerWidget(parent, f)
{
  // POLYDATA
  m_meshPolyData = vtkUnstructuredGrid::New();

  // Setup renderer
  renderer = vtkRenderer::New();
  renderer->SetBackground(0.3, 0.3, 0.3);
  renderer->ResetCamera();

  // Setup render window
  vtkRenderWindow *renderWindow = this->GetRenderWindow();
  renderWindow->AddRenderer(renderer);
}

usMeshDeformation::~usMeshDeformation() {}

/**
* Qt paint event overload if needed to update Qt widget
* @param event QPaintEvent.
*/
void usMeshDeformation::paintEvent(QPaintEvent *event) { usViewerWidget::paintEvent(event); }

/**
* Qt key press event catcher, used int this example to move the first point of the mesh.
* @param event QKeyEvent.
*/
void usMeshDeformation::keyPressEvent(QKeyEvent *event)
{
  if (event->key() == Qt::Key_Up) { // move first point of the mesh along Y
    double *point1 = m_meshPolyData->GetPoints()->GetPoint(0);
    point1[1] += 1;
    m_meshPolyData->GetPoints()->SetPoint(0, point1);
    m_meshPolyData->GetPoints()->Modified();
    this->GetRenderWindow()->Render();
  } else if (event->key() == Qt::Key_Down) { // move first point of the mesh of along Y
    double *point1 = m_meshPolyData->GetPoints()->GetPoint(0);
    point1[1] -= 1;
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
void usMeshDeformation::setMeshInScene(vtkUnstructuredGrid *mesh)
{
  m_meshPolyData = mesh;

  vtkSmartPointer<vtkDataSetMapper> meshMapper = vtkSmartPointer<vtkDataSetMapper>::New();
  meshMapper->SetInputData(mesh);
  m_meshActor = vtkSmartPointer<vtkActor>::New();
  m_meshActor->GetProperty()->SetColor(0, 0, 1.0); // blue
  m_meshActor->SetMapper(meshMapper);

  renderer->AddActor(m_meshActor);
}

/**
* Slot to call every time you want to update the mesh position.
* @param transform The homogeneous matrix of the mesh displacement to apply on current mesh position (can be considered
* as a movement).
*/
void usMeshDeformation::updateMeshPosition(vpHomogeneousMatrix transform)
{
  if (m_meshActor->GetUserMatrix() == NULL) { // init case
    vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    usVTKConverter::convert(transform, vtkMatrix);
    m_meshActor->SetUserMatrix(vtkMatrix);
  } else {
    // get current matrix
    vpHomogeneousMatrix currentTransform;
    currentTransform.eye();
    usVTKConverter::convert(m_meshActor->GetUserMatrix(), currentTransform);

    // Conversion, taking in account current transform
    vpHomogeneousMatrix newTransform = currentTransform * transform;

    // set new position
    vtkMatrix4x4 *vtkNewtransform = vtkMatrix4x4::New();
    usVTKConverter::convert(newTransform, vtkNewtransform);

    m_meshActor->SetUserMatrix(vtkNewtransform);
    this->GetRenderWindow()->Render();
  }
}

/**
* Point set of the mesh getter. To update a point position: call GetPoint(int ptIndex), update the coordinates, and then
* call SetPoint(ptIndex,yourPoint) followed by Modified() to update vtk object.
* @return The points of the mesh (pointer).
*/
vtkPoints *usMeshDeformation::getMeshPoints() { return m_meshPolyData->GetPoints(); }

/**
* To render the scene, after some updates done on objects.
*/
void usMeshDeformation::render() { this->GetRenderWindow()->Render(); }

#endif
