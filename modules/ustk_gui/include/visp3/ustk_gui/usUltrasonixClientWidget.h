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
 * Pedro Patlan Rosales
 *
 *****************************************************************************/

/**
 * @file usUltrasonixClientWidget.h
 * @brief Qt widget used to control the ultrasonix station using ustk-server application, through a network connection.
 */

#ifndef __usUltrasonixClientWidget_h_
#define __usUltrasonixClientWidget_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_VTK_QT) && defined(VISP_HAVE_MODULE_USTK_GRABBER)

#include <QApplication>
#include <QComboBox>
#include <QGridLayout>
#include <QHostAddress>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QWidget>
#include <visp3/ustk_grabber/usNetworkGrabber.h>

class VISP_EXPORT usUltrasonixClientWidget : public QWidget
{
  Q_OBJECT
public:
  usUltrasonixClientWidget();
  virtual ~usUltrasonixClientWidget();

signals:
  void connectToServer(QHostAddress adress);
  void initAcquisition(usNetworkGrabber::usInitHeaderSent initHeader);
  void center3DProbeMotor();
  void runAcquisition();
  void stopAcquisition();

private slots:
  void initAcquisitionSlot();
  void connectToServerSlot();

private:
  // Labels
  QLabel *serverIpAdressLabel;
  QLabel *probeSelectionLabel;

  // Connect, init, run, stop buttons
  QPushButton *connectPushButton;
  QPushButton *initPushButton;
  QPushButton *center3Dprobe;
  QPushButton *startPushButton;
  QPushButton *stopPushButton;

  // selection
  QLineEdit *ipTextEdit;
  QComboBox *probeSelectComboBox;

  QRegExpValidator *ipValidator;

  // Layouts
  QGridLayout *Layout;

  QHostAddress hostAddress;
  usNetworkGrabber::usInitHeaderSent initHeader;
};
#endif // USTK_HAVE_VTK_QT && VISP_HAVE_VIPER850
#endif // __usUltrasonixClientWidget_h_
