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

#include <visp3/ustk_gui/usRobotManualControlWidget.h>

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

  txSlider->setMinimumWidth(150);
  tySlider->setMinimumWidth(150);
  tzSlider->setMinimumWidth(150);
  wxSlider->setMinimumWidth(150);
  wySlider->setMinimumWidth(150);
  wzSlider->setMinimumWidth(150);

  L = new QGridLayout;
  L->addWidget(txLabel, 0, 0, Qt::AlignRight);
  L->addWidget(tyLabel, 1, 0, Qt::AlignRight);
  L->addWidget(tzLabel, 2, 0, Qt::AlignRight);
  L->addWidget(wxLabel, 3, 0, Qt::AlignRight);
  L->addWidget(wyLabel, 4, 0, Qt::AlignRight);
  L->addWidget(wzLabel, 5, 0, Qt::AlignRight);

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
  // Connection Signal-to slot to double signal to use in a main GUI
  connect(txSlider, SIGNAL(valueChanged(int)), SLOT(changeTxDouble(int)));
  connect(tySlider, SIGNAL(valueChanged(int)), SLOT(changeTyDouble(int)));
  connect(tzSlider, SIGNAL(valueChanged(int)), SLOT(changeTzDouble(int)));
  connect(wxSlider, SIGNAL(valueChanged(int)), SLOT(changeRxDouble(int)));
  connect(wySlider, SIGNAL(valueChanged(int)), SLOT(changeRyDouble(int)));
  connect(wzSlider, SIGNAL(valueChanged(int)), SLOT(changeRzDouble(int)));
}

void usRobotManualControlWidget::releaseSlider()
{
  QSlider *slider = dynamic_cast<QSlider *>(sender());
  slider->setValue(0);
}
