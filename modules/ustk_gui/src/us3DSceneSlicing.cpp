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
* @file us3DSceneSlicing.cpp
* @brief Graphical main window containing 4 vtk views.
*/

#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_gui/usVTKConverter.h>
#include <visp3/ustk_gui/us3DSceneSlicing.h>

#ifdef USTK_HAVE_VTK_QT

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkResliceImageViewer.h>
#include <vtkResliceCursorLineRepresentation.h>
#include <vtkResliceCursorThickLineRepresentation.h>
#include <vtkResliceCursorWidget.h>
#include <vtkResliceCursorActor.h>
#include <vtkResliceCursorPolyDataAlgorithm.h>
#include <vtkResliceCursor.h>
#include <vtkMetaImageReader.h>
#include <vtkCellPicker.h>
#include <vtkProperty.h>
#include <vtkPlane.h>
#include <vtkImageData.h>
#include <vtkCommand.h>
#include <vtkPlaneSource.h>
#include <vtkLookupTable.h>
#include <vtkImageMapToWindowLevelColors.h>
#include <vtkInteractorStyleImage.h>
#include <vtkImageSlabReslice.h>
#include <vtkBoundedPlanePointPlacer.h>
#include <vtkDistanceWidget.h>
#include <vtkDistanceRepresentation.h>
#include <vtkHandleRepresentation.h>
#include <vtkResliceImageViewerMeasurements.h>
#include <vtkDistanceRepresentation2D.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPointHandleRepresentation2D.h>
#include <vtkCamera.h>
#include <vtkRendererCollection.h>
#include <vtkMatrix4x4.h>
#include <vtkAbstractTransform.h>
#include <vtkImageActor.h>
#include <vtkMapper.h>
#include <vtkActor2D.h>
#include <vtkActor.h>

#include <vtkHomogeneousTransform.h>

#include <QDesktopWidget>
#include <QResizeEvent>

/**
* Constructor.
* @param imageFileName the mhd file to read.
*/
us3DSceneSlicing::us3DSceneSlicing(std::string imageFileName )
{
  this->setupUi();
  usImageIo::read(postScanImage,imageFileName);
  usVTKConverter::convert(postScanImage,vtkImage);

  int imageDims[3];
  double spacing[3];
  vtkImage->GetDimensions(imageDims);
  vtkImage->GetSpacing(spacing);

  plane1 = vtkPlane::New();
  plane1->SetOrigin(imageDims[0]*spacing[0]/2,0,0);
  plane1->SetNormal(1,0,0);

  plane2 = vtkPlane::New();
  plane2->SetNormal(0,1,0);
  plane2->SetOrigin(0,imageDims[1]*spacing[1]/2,0);

  plane3 = vtkPlane::New();
  plane3->SetNormal(0,0,1);
  plane3->SetOrigin(0,0,imageDims[2]*spacing[2]/2);

  this->view->setImageData(vtkImage);
  this->view->setPlanes(plane1,plane2,plane3);
  this->view->init();


  sliderX->setMaximum(imageDims[0]*spacing[0]*1000);
  sliderY->setMaximum(imageDims[1]*spacing[1]*1000);
  sliderZ->setMaximum(imageDims[2]*spacing[2]*1000);

  // Set up action signals and slots
  connect(this->sliderX, SIGNAL(valueChanged(int)), this, SLOT(updateX(int)));
  connect(this->sliderY, SIGNAL(valueChanged(int)), this, SLOT(updateY(int)));
  connect(this->sliderZ, SIGNAL(valueChanged(int)), this, SLOT(updateZ(int)));

  ResetViews();
}

/**
* Exit slot, to exit the QApplication.
*/
void us3DSceneSlicing::slotExit()
{
  qApp->exit();
}

/**
* Reset views slot : reset the planes positions at the middle of the volume.
*/
void us3DSceneSlicing::ResetViews()
{
  this->Render();
}


/**
* Render slot, to recompute all the views.
*/
void us3DSceneSlicing::Render()
{
  this->view->GetRenderWindow()->Render();

  this->view->update();
}

/**
* Setup all the widgets in the window.
*/
void us3DSceneSlicing::setupUi() {
  this->setMinimumSize(640,480);
  QRect screenRect = QApplication::desktop()->screenGeometry();
  this->resize(screenRect.size());

  gridLayoutWidget = new QWidget(this);
  gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
  gridLayoutWidget->setGeometry(QRect(10, 10, screenRect.width() - 200, screenRect.height() - 40));
  gridLayout_2 = new QGridLayout(gridLayoutWidget);
  gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
  gridLayout_2->setContentsMargins(0, 0, 0, 0);
  view = new us3DSceneWidget(gridLayoutWidget);
  view->setObjectName(QString::fromUtf8("view"));

  gridLayout_2->addWidget(view, 0, 0, 1, 1);

  sliderX =  new QSlider(this);
  sliderX->setObjectName(QString::fromUtf8("sliderX"));
  sliderX->setMinimum(0);
  sliderX->setMaximum(100);
  sliderX->setGeometry(QRect(screenRect.width() - 180, 30, 20, 300));

  sliderY =  new QSlider(this);
  sliderY->setObjectName(QString::fromUtf8("sliderY"));
  sliderY->setMinimum(0);
  sliderY->setMaximum(100);
  sliderY->setGeometry(QRect(screenRect.width() - 140, 30, 20, 300));

  sliderZ =  new QSlider(this);
  sliderZ->setObjectName(QString::fromUtf8("sliderZ"));
  sliderZ->setMinimum(0);
  sliderZ->setMaximum(100);
  sliderZ->setGeometry(QRect(screenRect.width() - 100, 30, 20, 300));

}

/**
* Get the resize event of the window, to re-comute size and positions of all widgets/layouts.
*/
void us3DSceneSlicing::resizeEvent(QResizeEvent* event)
{
  //Min size : 640*480
  if(event->size().width() >= 640 && event->size().height() >= 480) {
    QMainWindow::resizeEvent(event);
    gridLayoutWidget->setGeometry(QRect(10, 10, event->size().width() - 220, event->size().height() - 20));
    sliderX->setGeometry(QRect(event->size().width() - 180, 30, 20, 300));
    sliderY->setGeometry(QRect(event->size().width() - 140, 30, 20, 300));
    sliderZ->setGeometry(QRect(event->size().width() - 100, 30, 20, 300));
  }
}


void us3DSceneSlicing::updateX(int x) {
  double origin[3];
  plane1->GetOrigin(origin);
  origin[0] = x/1000.0;
  plane1->SetOrigin(origin);
  view->update();
}

void us3DSceneSlicing::updateY(int y) {
  double origin[3];
  plane2->GetOrigin(origin);
  origin[1] = y/1000.0;
  plane2->SetOrigin(origin);
  view->update();
}

void us3DSceneSlicing::updateZ(int z) {
  double origin[3];
  plane3->GetOrigin(origin);
  origin[2] = z/1000.0;
  plane3->SetOrigin(origin);
  view->update();
}

#endif
