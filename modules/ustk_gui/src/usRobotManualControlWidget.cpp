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
* @file usRobotManualControlWidget.cpp
* @brief Qt widget used to control manually robots with sliders for angular and linear velocities or positions.
*/

#include <iostream>
#include <visp3/ustk_gui/usRobotManualControlWidget.h>

#ifdef USTK_HAVE_VTK_QT

usRobotManualControlWidget::usRobotManualControlWidget()
{
  this->setStyleSheet("QGroupBox {  border: 2px solid Lightgray;}");

  this->setMinimumWidth(160);
  this->setMinimumHeight(200);

  txLabel = new QLabel("vx: ", this);
  tyLabel = new QLabel("vy: ", this);
  tzLabel = new QLabel("vz: ", this);
  wxLabel = new QLabel("wx: ", this);
  wyLabel = new QLabel("wy: ", this);
  wzLabel = new QLabel("wz: ", this);

  txSlider = new QSlider(Qt::Horizontal, this);
  txSlider->setValue(0);
  txSlider->setRange(-100, 100);
  tySlider = new QSlider(Qt::Horizontal, this);
  tySlider->setValue(0);
  tySlider->setRange(-100, 100);
  tzSlider = new QSlider(Qt::Horizontal, this);
  tzSlider->setValue(0);
  tzSlider->setRange(-100, 100);

  wxSlider = new QSlider(Qt::Horizontal, this);
  wxSlider->setValue(0);
  wxSlider->setRange(-80, 80);
  wySlider = new QSlider(Qt::Horizontal, this);
  wySlider->setValue(0);
  wySlider->setRange(-80, 80);
  wzSlider = new QSlider(Qt::Horizontal, this);
  wzSlider->setValue(0);
  wzSlider->setRange(-80, 80);

  initPushButton = new QPushButton(this);
  initPushButton->setText(QString("Init robot"));
  initPushButton->setEnabled(true);
  startStopPushButton = new QPushButton(this);
  startStopPushButton->setText(QString("Start robot"));
  startStopPushButton->setEnabled(false);

  labelRobotState = new QLabel(this);
  labelRobotState->setText("Press security handle, then click Init robot");

  txSlider->setMinimumWidth(150);
  tySlider->setMinimumWidth(150);
  tzSlider->setMinimumWidth(150);
  wxSlider->setMinimumWidth(150);
  wySlider->setMinimumWidth(150);
  wzSlider->setMinimumWidth(150);

  automaticForceButton = new QPushButton(this);
  automaticForceButton->setText(QString("Enable automatic force control"));
  automaticForceButton->setEnabled(true);

  L = new QGridLayout;
  L->addWidget(txLabel, 0, 0, Qt::AlignRight);
  L->addWidget(tyLabel, 1, 0, Qt::AlignRight);
  L->addWidget(tzLabel, 2, 0, Qt::AlignRight);
  L->addWidget(wxLabel, 3, 0, Qt::AlignRight);
  L->addWidget(wyLabel, 4, 0, Qt::AlignRight);
  L->addWidget(wzLabel, 5, 0, Qt::AlignRight);
  L->addWidget(initPushButton, 6, 0, 1, 2, Qt::AlignCenter);
  L->addWidget(startStopPushButton, 7, 0, 1, 2, Qt::AlignCenter);
  L->addWidget(labelRobotState, 8, 0, 1, 2, Qt::AlignCenter);
  L->addWidget(automaticForceButton, 9, 0, 1, 2, Qt::AlignCenter);

  L->addWidget(txSlider, 0, 1, Qt::AlignLeft);
  L->addWidget(tySlider, 1, 1, Qt::AlignLeft);
  L->addWidget(tzSlider, 2, 1, Qt::AlignLeft);
  L->addWidget(wxSlider, 3, 1, Qt::AlignLeft);
  L->addWidget(wySlider, 4, 1, Qt::AlignLeft);
  L->addWidget(wzSlider, 5, 1, Qt::AlignLeft);

  this->setLayout(L);
  // Setting up connections
  setUpConnections();
}

usRobotManualControlWidget::~usRobotManualControlWidget()
{
  delete txLabel;
  delete tyLabel;
  delete tzLabel;
  delete wxLabel;
  delete wyLabel;
  delete wzLabel;
  // Sliders
  delete txSlider;
  delete tySlider;
  delete tzSlider;
  delete wxSlider;
  delete wySlider;
  delete wzSlider;

  delete startStopPushButton;
  delete labelRobotState;

  delete automaticForceButton;

  // Layouts
  delete L;
}

void usRobotManualControlWidget::setUpConnections()
{
  // Connection slider release action with release slot
  connect(txSlider, SIGNAL(sliderReleased()), SLOT(releaseSlider()));
  connect(tySlider, SIGNAL(sliderReleased()), SLOT(releaseSlider()));
  connect(tzSlider, SIGNAL(sliderReleased()), SLOT(releaseSlider()));
  connect(wxSlider, SIGNAL(sliderReleased()), SLOT(releaseSlider()));
  connect(wySlider, SIGNAL(sliderReleased()), SLOT(releaseSlider()));
  connect(wzSlider, SIGNAL(sliderReleased()), SLOT(releaseSlider()));
  // Connection Signal-signal to use in a main GUI
  connect(txSlider, SIGNAL(valueChanged(int)), SIGNAL(changeVX(int)));
  connect(tySlider, SIGNAL(valueChanged(int)), SIGNAL(changeVY(int)));
  connect(tzSlider, SIGNAL(valueChanged(int)), SIGNAL(changeVZ(int)));
  connect(wxSlider, SIGNAL(valueChanged(int)), SIGNAL(changeWX(int)));
  connect(wySlider, SIGNAL(valueChanged(int)), SIGNAL(changeWY(int)));
  connect(wzSlider, SIGNAL(valueChanged(int)), SIGNAL(changeWZ(int)));
  // signal-signal connection for start / stop robot buttons
  connect(initPushButton, SIGNAL(clicked()), SIGNAL(initRobot()));
  automaticForceButton->setText(QString("Enable automatic force control"));
  automaticForceButton->setEnabled(false);

  // gui updates
  connect(initPushButton, SIGNAL(clicked()), this, SLOT(robotInitialized()));
  connect(startStopPushButton, SIGNAL(clicked()), this, SLOT(setRobotActivation()));

  connect(automaticForceButton, SIGNAL(clicked()), this, SLOT(activateAutomaticForceControlSlot()));
}

void usRobotManualControlWidget::releaseSlider()
{
  QSlider *slider = dynamic_cast<QSlider *>(sender());
  slider->setValue(0);
}

void usRobotManualControlWidget::robotInitialized(void)
{
  startStopPushButton->setEnabled(true);
  startStopPushButton->setText(QString("Start robot"));
  labelRobotState->setText(QString("Now press the viper start button (flashing)"));
  this->update();
}

void usRobotManualControlWidget::setRobotActivation()
{
  if (startStopPushButton->text() == QString("Start robot")) {
    emit(startRobot());
    startStopPushButton->setText(QString("Stop robot"));
    labelRobotState->setText(QString("Robot OK, you can move it or stop it"));
    automaticForceButton->setEnabled(true);
    this->update();
  }
  else if (startStopPushButton->text() == QString("Stop robot")) {
    emit(stopRobot());
    startStopPushButton->setText(QString("Start robot"));
    labelRobotState->setText(QString("Robot stopped"));
    automaticForceButton->setEnabled(false);
    this->update();
  }
}

void usRobotManualControlWidget::setRobotState(QString text)
{
  labelRobotState->setText(text);
  this->update();
}

void usRobotManualControlWidget::robotErrorSlot()
{
  startStopPushButton->setEnabled(false);
  labelRobotState->setText("Robot Error ! To restart, press security handle, then click Init robot");
  automaticForceButton->setText(QString("Enable automatic force control"));
  automaticForceButton->setEnabled(false);
  this->update();
}

void usRobotManualControlWidget::activateAutomaticForceControlSlot()
{

  bool activate = automaticForceButton->text() == QString("Enable automatic force control");

  txSlider->setEnabled(!activate);
  tySlider->setEnabled(!activate);
  automaticForceButton->setText(QString("Enable automatic force control"));
  tzSlider->setEnabled(!activate);
  wxSlider->setEnabled(!activate);
  wySlider->setEnabled(!activate);
  wzSlider->setEnabled(!activate);
  if (activate) {
    automaticForceButton->setText(QString("Disable automatic force control"));
    labelRobotState->setText("Automatic force control enabled");
    emit(activateAutomaticForceControl());
  }
  else {
    automaticForceButton->setText(QString("Enable automatic force control"));
    labelRobotState->setText("Manual robot control");
    emit(disableAutomaticForceControl());
  }
}

#endif
