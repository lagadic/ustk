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
 * @file usResliceMatrixViewer.h
 * @brief Graphical main window containing 2 vtk views : a 2D slice and the representation in 3D world.
 */


#ifndef __usResliceMatrixViewer_h_
#define __usResliceMatrixViewer_h_

// VISP includes
#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

// USTK includes

#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_gui/us2DSceneWidget.h>
#include <visp3/ustk_gui/us3DSceneWidget.h>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkPlane.h>
#include <vtkResliceImageViewer.h>
#include <vtkImagePlaneWidget.h>
#include <vtkDistanceWidget.h>
#include <vtkResliceImageViewerMeasurements.h>
#include <vtkImageMapper3D.h>


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
 * @class usResliceMatrixViewer
 * @brief Graphical main window containing 4 vtk views.
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usResliceMatrixViewer : public QMainWindow
{
  Q_OBJECT
public:

  // Constructor/Destructor
  usResliceMatrixViewer(std::string imageFileName);
  ~usResliceMatrixViewer() {}

  void resizeEvent(QResizeEvent* event);

public slots:

  virtual void ResetViews();
  virtual void Render();
  virtual void slotExit();

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
    us2DSceneWidget *view1;
    us2DSceneWidget *view4;
    us2DSceneWidget *view3;
    us3DSceneWidget *view2;

    QPushButton *resetButton;
    QPushButton *saveView1Button;

    //VTK planes
    vtkPlane *plane1;
    vtkPlane *plane2;
    vtkPlane *plane3;

    //transformation
    vtkMatrix4x4 *vtkMatrix1;
    vtkMatrix4x4 *vtkMatrix2;
    vtkMatrix4x4 *vtkMatrix3;

    //image
    usImagePostScan3D<unsigned char> postScanImage;
    vtkSmartPointer<vtkImageData> vtkImage;
};
#endif
#endif // __usResliceMatrixViewer_h_
