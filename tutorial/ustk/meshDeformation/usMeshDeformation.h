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
 * @file usMeshDeformation.h
 * @brief Class used to render a 3D mesh in a 3D environment.
 */

#ifndef __usMeshDeformation_h_
#define __usMeshDeformation_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/core/vpHomogeneousMatrix.h>

#include <visp3/ustk_gui/usViewerWidget.h>

#include <visp3/ustk_gui/usVTKConverter.h>

// VTK includes
#include <vtkArrowSource.h>
#include <vtkCylinderSource.h>
#include <vtkDataSetMapper.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

#include <QPaintEngine>
#include <QPainter>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

// Qt includes
#if defined(USTK_HAVE_VTK_QT4)
#include <QApplication>
#include <QKeyEvent>
#include <QVTKWidget.h>
#include <QtGui/QGridLayout>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#elif defined USTK_HAVE_VTK_QT5
#include <QApplication>
#include <QKeyEvent>
#include <QVTKWidget.h>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#endif

/**
 * @class usMeshDeformation
 * @brief Class used to render a 3D mesh in a 3D environment.
 * @ingroup module_ustk_gui
 */

class usMeshDeformation : public usViewerWidget
{
  Q_OBJECT
public:
  // Constructor/Destructor
  usMeshDeformation(QWidget *parent = NULL, Qt::WindowFlags f = 0);
  ~usMeshDeformation();

  void keyPressEvent(QKeyEvent *event);

  vtkPoints *getMeshPoints();

  void setMeshInScene(vtkUnstructuredGrid *mesh);

  // Catch paint events, in case we want to display some informations (writing in this widget) over the vtk scene
  void paintEvent(QPaintEvent *event);

  void render();

public slots:
  void updateMeshPosition(vpHomogeneousMatrix transform);

private:
  // mesh polydata
  vtkUnstructuredGrid *m_meshPolyData;

  // actor
  vtkSmartPointer<vtkActor> m_meshActor;

  // vtk renderer
  vtkRenderer *renderer;
};
#endif
#endif // __usMeshDeformation_h_
