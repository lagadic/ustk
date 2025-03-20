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
* @file usResliceMatrixViewer.cpp
* @brief Graphical main window containing 4 vtk views.
*/

#include <visp3/ustk_gui/usResliceMatrixViewer.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_gui/usVTKConverter.h>

#include <vtkAbstractTransform.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkBoundedPlanePointPlacer.h>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkDistanceRepresentation.h>
#include <vtkDistanceRepresentation2D.h>
#include <vtkDistanceWidget.h>
#include <vtkHandleRepresentation.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkImageMapToWindowLevelColors.h>
#include <vtkImageSlabReslice.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLookupTable.h>
#include <vtkMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkMetaImageReader.h>
#include <vtkPlane.h>
#include <vtkPlaneSource.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkResliceImageViewer.h>
#include <vtkResliceImageViewerMeasurements.h>

#include <vtkHomogeneousTransform.h>

#if defined(USTK_HAVE_VTK_QT6)
#include <QScreen>
#else
#include <QDesktopWidget> // deprecated in Qt5 and removed in Qt6
#endif
#include <QFileDialog>
#include <QResizeEvent>

/**
* Constructor.
* @param imageFileName the mhd file to read.
*/
usResliceMatrixViewer::usResliceMatrixViewer(std::string imageFileName)
{
  this->setupUi();
  usImageIo::read(postScanImage, imageFileName);
  usVTKConverter::convert(postScanImage, vtkImage);

  int imageDims[3];
  double spacing[3];
  vtkImage->GetDimensions(imageDims);
  vtkImage->GetSpacing(spacing);

  // matrix representing plane 3
  vpHomogeneousMatrix matrix3;
  matrix3.eye();
  matrix3[0][3] = imageDims[0] * spacing[0] / 2;
  matrix3[1][3] = imageDims[1] * spacing[1] / 2;
  matrix3[2][3] = imageDims[2] * spacing[2] / 2;
  matrix3[0][0] = 1;
  matrix3[1][1] = -1;
  matrix3[2][2] = -1;
  vtkMatrix3 = vtkMatrix4x4::New();
  usVTKConverter::convert(matrix3, vtkMatrix3);

  // matrix representing plane 2
  vpHomogeneousMatrix matrix2;
  matrix2.eye();
  matrix2[0][3] = imageDims[0] * spacing[0] / 2;
  matrix2[1][3] = imageDims[1] * spacing[1] / 2;
  matrix2[2][3] = imageDims[2] * spacing[2] / 2;
  matrix2[0][0] = 1;
  matrix2[2][1] = 1;
  matrix2[1][1] = 0;
  matrix2[1][2] = -1;
  matrix2[2][2] = 0;

  vtkMatrix2 = vtkMatrix4x4::New();
  usVTKConverter::convert(matrix2, vtkMatrix2);

  // matrix representing plane 1
  vpHomogeneousMatrix matrix1;
  matrix1.eye();
  matrix1[0][3] = imageDims[0] * spacing[0] / 2;
  matrix1[1][3] = imageDims[1] * spacing[1] / 2;
  matrix1[2][3] = imageDims[2] * spacing[2] / 2;
  matrix1[0][0] = 0;
  matrix1[0][2] = 1;
  matrix1[1][1] = -1;
  matrix1[2][0] = 1;
  matrix1[2][2] = 0;
  vtkMatrix1 = vtkMatrix4x4::New();
  usVTKConverter::convert(matrix1, vtkMatrix1);

  view2->setImageData(vtkImage);
  view2->updateMatrix1(vtkMatrix1);
  view2->updateMatrix2(vtkMatrix2);
  view2->updateMatrix3(vtkMatrix3);
  view2->init();

  view1->setImageData(vtkImage);
  view1->setResliceMatrix(vtkMatrix1);
  view1->setPolyDataPlaneContour(view2->getContour1());
  view1->setPolyDataMeshContour(view2->getMeshInPlane1());
  view1->init();
  view1->setColor(1.0, 0, 0);

  view4->setImageData(vtkImage);
  view4->setResliceMatrix(vtkMatrix2);
  view4->setPolyDataPlaneContour(view2->getContour2());
  view4->setPolyDataMeshContour(view2->getMeshInPlane2());
  view4->init();
  view4->setColor(0, 1.0, 0);

  view3->setImageData(vtkImage);
  view3->setResliceMatrix(vtkMatrix3);
  view3->setPolyDataPlaneContour(view2->getContour3());
  view3->setPolyDataMeshContour(view2->getMeshInPlane3());
  view3->init();
  view3->setColor(0, 0, 1.0);

  // Set up action signals and slots
  connect(this->resetButton, SIGNAL(pressed()), this, SLOT(ResetViews()));
  // connect(this->saveView1Button, SIGNAL(pressed()), view1, SLOT(saveViewSlot()));
  connect(this->openImageButton, SIGNAL(pressed()), this, SLOT(openPostScan3D()));
  connect(this->saveView1Button, SIGNAL(pressed()), this, SLOT(getView1Slice()));
  connect(this->saveView4Button, SIGNAL(pressed()), view4, SLOT(saveViewSlot()));
  connect(this->saveView3Button, SIGNAL(pressed()), view3, SLOT(saveViewSlot()));

  // updates of planes in 3D scene
  connect(view1, SIGNAL(matrixChanged(vtkMatrix4x4 *)), view2, SLOT(updateMatrix1(vtkMatrix4x4 *)));
  connect(view4, SIGNAL(matrixChanged(vtkMatrix4x4 *)), view2, SLOT(updateMatrix2(vtkMatrix4x4 *)));
  connect(view3, SIGNAL(matrixChanged(vtkMatrix4x4 *)), view2, SLOT(updateMatrix3(vtkMatrix4x4 *)));
  // update back 2D views (for polydata intersections)
  connect(view2, SIGNAL(plane2Changed()), view1, SLOT(updateView()));
  connect(view2, SIGNAL(plane2Changed()), view4, SLOT(updateView()));
  connect(view2, SIGNAL(plane2Changed()), view3, SLOT(updateView()));

  ResetViews();
}

/**
* Exit slot, to exit the QApplication.
*/
void usResliceMatrixViewer::slotExit() { qApp->exit(); }

/**
* Reset views slot : reset the planes positions at the middle of the volume.
*/
void usResliceMatrixViewer::ResetViews()
{
  int imageDims[3];
  double spacing[3];
  vtkImage->GetDimensions(imageDims);
  vtkImage->GetSpacing(spacing);

  // matrix representing plane 3
  vpHomogeneousMatrix matrix3;
  matrix3.eye();
  matrix3[0][3] = imageDims[0] * spacing[0] / 2;
  matrix3[1][3] = imageDims[1] * spacing[1] / 2;
  matrix3[2][3] = imageDims[2] * spacing[2] / 2;
  matrix3[0][0] = 1;
  matrix3[1][1] = -1;
  matrix3[2][2] = -1;
  usVTKConverter::convert(matrix3, vtkMatrix3);

  // matrix representing plane 2
  vpHomogeneousMatrix matrix2;
  matrix2.eye();
  matrix2[0][3] = imageDims[0] * spacing[0] / 2;
  matrix2[1][3] = imageDims[1] * spacing[1] / 2;
  matrix2[2][3] = imageDims[2] * spacing[2] / 2;
  matrix2[0][0] = 1;
  matrix2[2][1] = 1;
  matrix2[1][1] = 0;
  matrix2[1][2] = -1;
  matrix2[2][2] = 0;
  usVTKConverter::convert(matrix2, vtkMatrix2);

  // matrix representing plane 1
  vpHomogeneousMatrix matrix1;
  matrix1.eye();
  matrix1[0][3] = imageDims[0] * spacing[0] / 2;
  matrix1[1][3] = imageDims[1] * spacing[1] / 2;
  matrix1[2][3] = imageDims[2] * spacing[2] / 2;
  matrix1[0][0] = 0;
  matrix1[0][2] = 1;
  matrix1[1][1] = -1;
  matrix1[2][0] = 1;
  matrix1[2][2] = 0;
  usVTKConverter::convert(matrix1, vtkMatrix1);

  view1->setResliceMatrix(vtkMatrix1);
  view4->setResliceMatrix(vtkMatrix2);
  view3->setResliceMatrix(vtkMatrix3);

  view2->updateMatrix1(vtkMatrix1);
  view2->updateMatrix2(vtkMatrix2);
  view2->updateMatrix3(vtkMatrix3);

  this->Render();
}

/**
* Render slot, to recompute all the views.
*/
void usResliceMatrixViewer::Render()
{
#if defined(USTK_HAVE_VTK_QT6)
  this->view1->renderWindow()->Render();
  this->view2->renderWindow()->Render();
  this->view3->renderWindow()->Render();
  this->view4->renderWindow()->Render();
#else
  this->view1->GetRenderWindow()->Render();
  this->view2->GetRenderWindow()->Render();
  this->view3->GetRenderWindow()->Render();
  this->view4->GetRenderWindow()->Render();
#endif
}

/**
* Setup all the widgets in the window.
*/
void usResliceMatrixViewer::setupUi()
{
  this->setMinimumSize(640, 480);
#if defined(USTK_HAVE_VTK_QT6)
  QRect screenRect = QGuiApplication::primaryScreen()->geometry();
#else
  QRect screenRect = QApplication::desktop()[0]->screenGeometry();
#endif
  this->resize(screenRect.size());

  gridLayoutWidget = new QWidget(this);
  gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
  gridLayoutWidget->setGeometry(QRect(10, 10, screenRect.width() - 200, screenRect.height() - 40));
  gridLayout_2 = new QGridLayout(gridLayoutWidget);
  gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
  gridLayout_2->setContentsMargins(0, 0, 0, 0);
  view1 = new us2DSceneWidget(gridLayoutWidget);
  view1->setObjectName(QString::fromUtf8("view1"));

  gridLayout_2->addWidget(view1, 0, 0, 1, 1);

  view2 = new us3DSceneWidget(gridLayoutWidget);
  view2->setObjectName(QString::fromUtf8("view1"));

  gridLayout_2->addWidget(view2, 1, 0, 1, 1);

  view3 = new us2DSceneWidget(gridLayoutWidget);
  view3->setObjectName(QString::fromUtf8("view1"));

  gridLayout_2->addWidget(view3, 0, 1, 1, 1);

  view4 = new us2DSceneWidget(gridLayoutWidget);
  view4->setObjectName(QString::fromUtf8("view2"));

  gridLayout_2->addWidget(view4, 1, 1, 1, 1);

  resetButton = new QPushButton(this);
  resetButton->setObjectName(QString::fromUtf8("resetButton"));
  resetButton->setText(QString::fromUtf8("Reset views"));
  resetButton->setGeometry(QRect(screenRect.width() - 180, 30, 160, 31));

  openImageButton = new QPushButton(this);
  openImageButton->setObjectName(QString::fromUtf8("openImageButton"));
  openImageButton->setText(QString::fromUtf8("Open new image"));
  openImageButton->setGeometry(QRect(screenRect.width() - 180, 80, 160, 31));

  saveView1Button = new QPushButton(this);
  saveView1Button->setObjectName(QString::fromUtf8("saveView1Button"));
  saveView1Button->setText(QString::fromUtf8("Save view 1"));
  saveView1Button->setGeometry(QRect(screenRect.width() - 180, 130, 160, 31));

  saveView4Button = new QPushButton(this);
  saveView4Button->setObjectName(QString::fromUtf8("saveView4Button"));
  saveView4Button->setText(QString::fromUtf8("Save view 4"));
  saveView4Button->setGeometry(QRect(screenRect.width() - 180, 180, 160, 31));

  saveView3Button = new QPushButton(this);
  saveView3Button->setObjectName(QString::fromUtf8("saveView3Button"));
  saveView3Button->setText(QString::fromUtf8("Save view 3"));
  saveView3Button->setGeometry(QRect(screenRect.width() - 180, 230, 160, 31));
}

/**
* Get the resize event of the window, to re-comute size and positions of all widgets/layouts.
*/
void usResliceMatrixViewer::resizeEvent(QResizeEvent *event)
{
  // Min size : 640*480
  if (event->size().width() >= 640 && event->size().height() >= 480) {
    QMainWindow::resizeEvent(event);
    gridLayoutWidget->setGeometry(QRect(10, 10, event->size().width() - 220, event->size().height() - 20));
    resetButton->setGeometry(QRect(event->size().width() - 180, 30, 160, 31));
    openImageButton->setGeometry(QRect(event->size().width() - 180, 80, 160, 31));
    saveView1Button->setGeometry(QRect(event->size().width() - 180, 130, 160, 31));
    saveView4Button->setGeometry(QRect(event->size().width() - 180, 180, 160, 31));
    saveView3Button->setGeometry(QRect(event->size().width() - 180, 230, 160, 31));
  }
}

/**
* Getter of view 1 current slice.
*/
void usResliceMatrixViewer::getView1Slice()
{
  usImagePostScan2D<unsigned char> postScanSlice;
  view1->getCurrentSlice(postScanSlice);
  usImageIo::write(postScanSlice, "sliceView1.xml");
}

/**
* Open new image slot.
*/
void usResliceMatrixViewer::openPostScan3D()
{
  // open file dialog to let the user select the new mhd file to display
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open Image"), us::getDataSetPath().c_str(),
                                                  tr("Meta header files (*.mhd)"));

  if (fileName.size() == 0)
    return;

  double t0 = vpTime::measureTimeMs();

  // read the image and convert it to vtkImageData
  usImageIo::read(postScanImage, fileName.toStdString());
  double t1 = vpTime::measureTimeMs();
  std::cout << "read image time (ms) = " << t1 - t0 << std::endl;
  usVTKConverter::convert(postScanImage, vtkImage);

  // update the views
  view2->updateImageData(vtkImage);
  view1->updateImageData(vtkImage);
  view3->updateImageData(vtkImage);
  view4->updateImageData(vtkImage);

  std::cout << "convert/update image time (ms) = " << vpTime::measureTimeMs() - t1 << std::endl;
}
#endif
