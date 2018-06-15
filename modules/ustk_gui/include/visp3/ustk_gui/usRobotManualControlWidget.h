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
 * @file usRobotManualControlWidget.h
 * @brief Qt widget used to control manually robots with sliders for angular and linear velocities or positions.
 */

#ifndef __usRobotManualControlWidget_h_
#define __usRobotManualControlWidget_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>

class VISP_EXPORT usRobotManualControlWidget : public QGroupBox
{
  Q_OBJECT
public:
  usRobotManualControlWidget();
  ~usRobotManualControlWidget();

private:
  void setUpConnections();
  // Labels
  QLabel *txLabel;
  QLabel *tyLabel;
  QLabel *tzLabel;
  QLabel *wxLabel;
  QLabel *wyLabel;
  QLabel *wzLabel;
  // Sliders
  QSlider *txSlider;
  QSlider *tySlider;
  QSlider *tzSlider;
  QSlider *wxSlider;
  QSlider *wySlider;
  QSlider *wzSlider;
  // Start / stop button
  QPushButton *initPushButton;
  QPushButton *startStopPushButton;
  QLabel *labelRobotState;
  QPushButton *automaticForceButton;
  // Layouts
  QGridLayout *L;

  bool automaticForceControl;

signals:
  // Linear velocities
  void changeVX(int);
  void changeVY(int);
  void changeVZ(int);
  // Angular velocities
  void changeWX(int);
  void changeWY(int);
  void changeWZ(int);

  void initRobot();
  void startRobot();
  void stopRobot();

  void activateAutomaticForceControl();
  void disableAutomaticForceControl();

public slots:
  void robotErrorSlot();
  void robotInitialized(void);
  void releaseSlider(void);
  void setRobotState(QString text);
  void setRobotActivation();

private slots:
  void activateAutomaticForceControlSlot();
};

#endif // USTK_HAVE_VTK_QT
#endif // __usRobotManualControlWidget_h_
