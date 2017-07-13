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
 * @file usVirtualNeedle.h
 * @brief Class used to render a virtual needle movable in a 3D environment.
 */

#ifndef __usVirtualNeedle_h_
#define __usVirtualNeedle_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/core/vpHomogeneousMatrix.h>

#include <visp3/ustk_gui/usViewerWidget.h>

//VTK includes
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkAxesActor.h>
#include <vtkPolyDataMapper.h>

// Qt includes
#if defined(USTK_HAVE_VTK_QT4)
#  include <QApplication>
#  include <QtGui/QMainWindow>
#  include <QtGui/QGridLayout>
#  include <QtGui/QPushButton>
#  include <QKeyEvent>
#include <QVTKWidget.h>
#elif defined USTK_HAVE_VTK_QT5
#  include <QApplication>
#  include <QtWidgets/QMainWindow>
#  include <QtWidgets/QGridLayout>
#  include <QtWidgets/QPushButton>
#  include <QKeyEvent>
#include <QVTKWidget.h>
#endif

/**
 * @class usVirtualNeedle
 * @brief Class used to render a virtual needle movable in a 3D environment.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usVirtualNeedle : public usViewerWidget
{
  Q_OBJECT
public:

  // Constructor/Destructor
  usVirtualNeedle(QWidget* parent = NULL, Qt::WindowFlags f = 0);
  ~usVirtualNeedle() {}

  void setMeshInScene(vtkPolyData* mesh);

  //Catch paint events, in case we want to display some informations (writing in this widget) over the vtk scene
  void paintEvent( QPaintEvent* event );

  void keyPressEvent(QKeyEvent *event);

public slots:
  void updateNeedlePosition(vpHomogeneousMatrix transform);

signals:

private:
  //mesh polydata
  vtkPolyData* m_meshPolyData;
  //needle mesh
  vtkPolyData* m_meshNeedle;

  //actors
  vtkSmartPointer<vtkActor> m_meshActor;
  vtkSmartPointer<vtkActor> m_needleActor;

  //axes representation
  vtkSmartPointer<vtkAxesActor> m_axesActor;

  //vtk renderer
  vtkRenderer* renderer;
};
#endif
#endif // __usVirtualNeedle_h_
