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
 * @file usMedicalImageViewer.h
 * @brief Graphical main window containing 4 vtk views.
 */


#ifndef __usMedicalImageViewer_h_
#define __usMedicalImageViewer_h_

// VISP includes
#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

// USTK includes

#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_gui/usViewerWidget.h>
#include <visp3/ustk_gui/us3DSceneWidget.h>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkPlane.h>
#include <vtkResliceImageViewer.h>
#include <vtkImagePlaneWidget.h>
#include <vtkDistanceWidget.h>
#include <vtkResliceImageViewerMeasurements.h>

// Qt includes
#if defined(USTK_HAVE_VTK_QT4)
#  include <QtGui/QApplication>
#  include <QtGui/QMainWindow>
#  include <QtGui/QGridLayout>
#  include <QtGui/QPushButton>
#elif defined(USTK_HAVE_VTK_QT4) //  QT 5 ?
#  include <QtWidgets/QApplication>
#  include <QtWidgets/QMainWindow>
#  include <QtWidgets/QGridLayout>
#  include <QtWidgets/QPushButton>
#endif

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
  usMedicalImageViewer(std::string imageFileName);
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
    usViewerWidget *view2;
    us3DSceneWidget *view4;
    usViewerWidget *view3;
    usViewerWidget *view1;
    QPushButton *resetButton;
    QPushButton *resetColorsButton;
    QPushButton *AddDistance1Button;

    //VTK planes
    vtkPlane *plane1;
    vtkPlane *plane2;
    vtkPlane *plane3;

    //image
    usImagePostScan3D<unsigned char> postScanImage;
    vtkSmartPointer<vtkImageData> vtkImage;
};
#endif
#endif // US_MEDICAL_IMAGE_VIEWER
