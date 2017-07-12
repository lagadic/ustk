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
#include <visp3/core/vpException.h>
#include <visp3/ustk_gui/usVirtualNeedle.h>
#include <visp3/core/vpRotationMatrix.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_gui/usVTKConverter.h>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkArrowSource.h>
#include <vtkMatrix4x4.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkCylinderSource.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>

#include <QPainter>
#include <QPaintEngine>

/**
* Constructor.
*/
usVirtualNeedle::usVirtualNeedle(QWidget* parent, Qt::WindowFlags f) : usViewerWidget(parent,f)
{
  // POLYDATA

  // Create a cylinder for the needle
  vtkSmartPointer<vtkCylinderSource> cylinderSource =
    vtkSmartPointer<vtkCylinderSource>::New();
  cylinderSource->SetCenter(0,0,0);
  cylinderSource->SetRadius(0.0001);
  cylinderSource->SetHeight(0.01);
  cylinderSource->SetResolution(100);
  cylinderSource->Update();
  m_meshNeedle = cylinderSource->GetOutput();

  // MAPPERS - ACTORS
  vtkSmartPointer<vtkPolyDataMapper> cylinderMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  cylinderMapper->SetInputConnection( cylinderSource->GetOutputPort());
  m_needleActor = vtkSmartPointer<vtkActor>::New();
  m_needleActor->GetProperty()->SetColor(1.0,0,0); //red
  m_needleActor->GetProperty()->SetOpacity(1.0);
  m_needleActor->SetMapper(cylinderMapper);

  //Axes
  m_axesActor = vtkSmartPointer<vtkAxesActor>::New();

  //arrows of 1cm
  m_axesActor->SetXAxisLabelText("X");
  m_axesActor->SetYAxisLabelText("Y");
  m_axesActor->SetZAxisLabelText("Z");
  m_axesActor->SetTotalLength(0.01,0.01,0.01); // 10cm each

  // Setup renderer
  renderer = vtkRenderer::New();
  renderer->AddActor(m_axesActor);
  renderer->AddActor(m_needleActor);
  renderer->SetBackground(0.5, 0.5, 0.5);
  renderer->ResetCamera();

  // Setup render window
  vtkRenderWindow* renderWindow = this->GetRenderWindow();
  renderWindow->AddRenderer(renderer);
}

/**
* Qt paint event overload if needed to update Qt widget
* @param event QPaintEvent.
*/
void usVirtualNeedle::paintEvent( QPaintEvent* event )
{
  usViewerWidget::paintEvent( event );
}

/**
* Setter for the mesh to introcuce in the scene.
* @param mesh The mesh, under vtkPolydataFormat.
*/
void usVirtualNeedle::setMeshInScene(vtkPolyData* mesh) {
  m_meshPolyData = mesh;

  vtkSmartPointer<vtkPolyDataMapper> meshMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  meshMapper->SetInputData(mesh);
  m_meshActor = vtkSmartPointer<vtkActor>::New();
  m_meshActor->GetProperty()->SetColor(0,0,1.0); //blue
  m_meshActor->SetMapper(meshMapper);

  renderer->AddActor(m_meshActor);
}

/**
* Slot to call every time you want to set
* @param mesh The mesh, under vtkPolydataFormat.
*/
void usVirtualNeedle::updateNeedlePosition(vpHomogeneousMatrix transform) {
  // get current matrix
  vpHomogeneousMatrix currentTransform;
  usVTKConverter::convert(m_needleActor->GetUserMatrix(),currentTransform);

  // Conversion
  // To check !
  vpHomogeneousMatrix newTransform = currentTransform * transform; // or transform.inverse ?

  // set new position
  vtkMatrix4x4* vtkNewtransform = vtkMatrix4x4::New();
  usVTKConverter::convert(newTransform,vtkNewtransform);
  m_needleActor->SetUserMatrix(vtkNewtransform);
}

#endif
