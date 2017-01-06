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
 * @file usMedicalImageViewer.h
 * @brief Graphical main window containing 4 vtk views.
 */


#ifndef US_MEDICAL_IMAGE_VIEWER
#define US_MEDICAL_IMAGE_VIEWER

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
 * @class usMedicalImageViewer
 * @brief Graphical main window containing 4 vtk views.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usMedicalImageViewer : public QMainWindow
{
  Q_OBJECT
public:

  // Constructor/Destructor
  usMedicalImageViewer(std::string imageFileName );
  ~usMedicalImageViewer() {}

  void resizeEvent(QResizeEvent* event);

public slots:

  virtual void ResetViews();
  virtual void ResetColorMap();
  virtual void Render();
  virtual void AddDistanceMeasurementToView1();
  virtual void AddDistanceMeasurementToView( int );
  virtual void slotExit();

protected:
  vtkSmartPointer< vtkResliceImageViewer > riw[3];
  vtkSmartPointer< vtkImagePlaneWidget > planeWidget[3];
  vtkSmartPointer< vtkDistanceWidget > DistanceWidget[3];
  vtkSmartPointer< vtkResliceImageViewerMeasurements > ResliceMeasurements[3];

protected slots:

private:
    void setupUi();

    QAction *actionOpenFile;
    QAction *actionExit;
    QAction *actionPrint;
    QAction *actionHelp;
    QAction *actionSave;
    QWidget *centralwidget;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_2;
    QVTKWidget *view2;
    QVTKWidget *view4;
    QVTKWidget *view3;
    QVTKWidget *view1;
    QPushButton *resetButton;
    QPushButton *resetColorsButton;
    QPushButton *AddDistance1Button;
};

#endif // US_MEDICAL_IMAGE_VIEWER
