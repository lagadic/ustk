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
 * @file usViper850WrapperVelocityControl.h
 * @brief Qt widget used to control manually robots with sliders for angular and linear velocities or positions.
 */

#ifndef __usViper850WrapperVelocityControl_h_
#define __usViper850WrapperVelocityControl_h_

// VISP includes
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_VTK_QT) && defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_MODULE_ROBOT)

#include <QApplication>
#include <QObject>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/robot/vpRobotViper850.h>

// For 4DC7 probe usage //
#define VIPER_PROBE_4DC7_TX 0.00479375
#define VIPER_PROBE_4DC7_TY 0.142001
#define VIPER_PROBE_4DC7_TZ -0.00113622
#define VIPER_PROBE_4DC7_RZ 1.63126 * M_PI / 180.0
#define VIPER_PROBE_4DC7_RY -0.781128 * M_PI / 180.0
#define VIPER_PROBE_4DC7_RX 1.68121 * M_PI / 180.0

class VISP_EXPORT usViper850WrapperVelocityControl : public QObject
{
  Q_OBJECT
public:
  usViper850WrapperVelocityControl();
  virtual ~usViper850WrapperVelocityControl();

public slots:
  void init();
  void run();
  void stop();

  void startAutomaticForceControl();
  void stopAutomaticForceControl();

  // Manual velocity control in US probe contact frame
  void setXVelocity(int xVelocity);
  void setYVelocity(int yVelocity);
  void setZVelocity(int zVelocity);
  void setXAngularVelocity(int xAngularVelocity);
  void setYAngularVelocity(int yAngularVelocity);
  void setZAngularVelocity(int zAngularVelocity);

  void moveLeft();
  void moveRight();
  void stopMove();

signals:
  void startControlLoop();
  void startControlLoopAutomatic();
  void robotError();
private slots:
  void controlLoop();
  void controlLoopAutomatic();

private:
  bool m_initialized;
  bool m_run;
  vpRobotViper850 *viper;
  void startRobot(void);

  vpMatrix eJe; // robot jacobian

  vpVelocityTwistMatrix eVp; // change velocity from ultrasound probe contact force to end-effector frame

  vpColVector velocityProbeContact; // velocity in ultrasound probe contact point
  vpColVector q_dot;
  vpColVector ve; // velocity of end-effector
};
#endif // USTK_HAVE_VTK_QT && VISP_HAVE_VIPER850
#endif // __usViper850WrapperVelocityControl_h_
