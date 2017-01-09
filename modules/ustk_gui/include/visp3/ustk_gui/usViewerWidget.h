/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usViewerWidget.h
 * @brief View used to render a vtk scene in a QWidget (based on QVTKWidget)
 */


#ifndef US_VIEWER_WIDGET
#define US_VIEWER_WIDGET

//VISP includes
#include <visp3/core/vpConfig.h>

//USTK includes


//VTK includes
#include <vtkSmartPointer.h>
#include <vtkResliceImageViewer.h>
#include <vtkImagePlaneWidget.h>
#include <vtkDistanceWidget.h>
#include <vtkResliceImageViewerMeasurements.h>

//Qt includes
#include <QtGui/QApplication>
#include <QtGui/QMainWindow>
#include <QtGui/QGridLayout>
#include <QtGui/QPushButton>

#include <QVTKWidget.h>

/**
 * @class usViewerWidget
 * @brief View used to render a vtk scene in a QWidget (based on QVTKWidget)
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usViewerWidget : public QVTKWidget
{
  Q_OBJECT
public:

  // Constructor/Destructor
  usViewerWidget(QWidget* parent = NULL, Qt::WindowFlags f = 0);
  ~usViewerWidget() {}

  void paintEvent( QPaintEvent* event );
};

#endif // US_VIEWER_WIDGET
